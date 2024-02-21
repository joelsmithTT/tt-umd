#include "tt_pci_device.h"

#include <dirent.h>
#include <fcntl.h>
#include <linux/pci.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ioctl.h"


static uint32_t GS_BAR0_WC_MAPPING_SIZE = (156<<20) + (10<<21) + (18<<24);
static const uint16_t WORMHOLE_DEVICE_ID = 0x401e;

static int find_device(const uint16_t device_id) {
    const char device_name_pattern[] = "/dev/tenstorrent/%u";
    // returns device id if found, otherwise -1
    char device_name[sizeof(device_name_pattern) + std::numeric_limits<unsigned int>::digits10];
    std::snprintf(device_name, sizeof(device_name), device_name_pattern, (unsigned int)device_id);
    int device_fd = ::open(device_name, O_RDWR | O_CLOEXEC);
    return device_fd;
}

TTDevice::TTDevice(uint32_t device_id)
    : index(device_id)
    , device_fd(find_device(index))
{
    if (device_fd == -1) {
        throw std::runtime_error(std::string("Failed opening a handle for device ") + std::to_string(index));
    }

    tenstorrent_get_device_info device_info;
    memset(&device_info, 0, sizeof(device_info));
    device_info.in.output_size_bytes = sizeof(device_info.out);

    if (ioctl(device_fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &device_info) == -1) {
        throw std::runtime_error(std::string("Get device info failed on device ") + std::to_string(index) + ".");
    }

    this->device_info = device_info.out;

    max_dma_buf_size_log2 = device_info.out.max_dma_buf_size_log2;

    struct {
        tenstorrent_query_mappings query_mappings;
        tenstorrent_mapping mapping_array[8];
    } mappings;

    memset(&mappings, 0, sizeof(mappings));
    mappings.query_mappings.in.output_mapping_count = 8;

    if (ioctl(device_fd, TENSTORRENT_IOCTL_QUERY_MAPPINGS, &mappings.query_mappings) == -1) {
        throw std::runtime_error(std::string("Query mappings failed on device ") + std::to_string(index) + ".");
    }

    tenstorrent_mapping bar0_uc_mapping;
    tenstorrent_mapping bar0_wc_mapping;
    tenstorrent_mapping bar2_uc_mapping;
    tenstorrent_mapping bar2_wc_mapping;

    memset(&bar0_uc_mapping, 0, sizeof(bar0_uc_mapping));
    memset(&bar0_wc_mapping, 0, sizeof(bar0_wc_mapping));
    memset(&bar2_uc_mapping, 0, sizeof(bar2_uc_mapping));
    memset(&bar2_wc_mapping, 0, sizeof(bar2_wc_mapping));

    for (unsigned int i = 0; i < mappings.query_mappings.in.output_mapping_count; i++) {
        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE0_UC) {
            bar0_uc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE0_WC) {
            bar0_wc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE2_UC) {
            bar2_uc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE2_WC) {
            bar2_wc_mapping = mappings.mapping_array[i];
        }
    }

    if (bar0_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE0_UC) {
        throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR0 UC mapping.");
    }

    // Attempt WC mapping first so we can fall back to all-UC if it fails.
    if (bar0_wc_mapping.mapping_id == TENSTORRENT_MAPPING_RESOURCE0_WC) {
        bar0_wc_size = std::min<size_t>(bar0_wc_mapping.mapping_size, GS_BAR0_WC_MAPPING_SIZE);
        bar0_wc = mmap(NULL, bar0_wc_size, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, bar0_wc_mapping.mapping_base);
        if (bar0_wc == MAP_FAILED) {
            bar0_wc_size = 0;
            bar0_wc = nullptr;
        }
    }

    if (bar0_wc) {
        // The bottom part of the BAR is mapped WC. Map the top UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size - GS_BAR0_WC_MAPPING_SIZE;
        bar0_uc_offset = GS_BAR0_WC_MAPPING_SIZE;
    } else {
        // No WC mapping, map the entire BAR UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size;
        bar0_uc_offset = 0;
    }

    bar0_uc = mmap(
        NULL,
        bar0_uc_size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        device_fd,
        bar0_uc_mapping.mapping_base + bar0_uc_offset);

    if (bar0_uc == MAP_FAILED) {
        throw std::runtime_error(
            std::string("BAR0 UC memory mapping failed for device ") + std::to_string(index) + ".");
    }

    if (!bar0_wc) {
        bar0_wc = bar0_uc;
    }

    if (device_info.out.device_id == WORMHOLE_DEVICE_ID) {
        if (bar2_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE2_UC) {
            throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR4 UC mapping.");
        }

        this->system_reg_mapping_size = bar2_uc_mapping.mapping_size;

        this->system_reg_mapping = mmap(
            NULL,
            bar2_uc_mapping.mapping_size,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            device_fd,
            bar2_uc_mapping.mapping_base);

        if (this->system_reg_mapping == MAP_FAILED) {
            throw std::runtime_error(
                std::string("BAR4 UC memory mapping failed for device ") + std::to_string(index) + ".");
        }

        this->system_reg_start_offset = (512 - 16) * 1024 * 1024;
        this->system_reg_offset_adjust = (512 - 32) * 1024 * 1024;
    }
    pci_domain = device_info.out.pci_domain;
    pci_bus = device_info.out.bus_dev_fn >> 8;
    pci_device = PCI_SLOT(device_info.out.bus_dev_fn);
    pci_function = PCI_FUNC(device_info.out.bus_dev_fn);
}

TTDevice::~TTDevice() {
    if (device_fd != -1) {
        close(device_fd);
    }

    if (bar0_wc != nullptr && bar0_wc != MAP_FAILED && bar0_wc != bar0_uc) {
        munmap(bar0_wc, bar0_wc_size);
    }

    if (bar0_uc != nullptr && bar0_uc != MAP_FAILED) {
        munmap(bar0_uc, bar0_uc_size);
    }

    if (system_reg_mapping != nullptr && system_reg_mapping != MAP_FAILED) {
        munmap(system_reg_mapping, system_reg_mapping_size);
    }

    for (auto &&buf : dma_buffer_mappings) {
        munmap(buf.pBuf, buf.size);
    }

    if (sysfs_config_fd != -1) {
        close(sysfs_config_fd);
    }
}

// TODO(jms): Doesn't seem to be necessary anymore?
// Open a unique device_id per host memory channel (workaround for ttkmd < 1.21 support for more than 1 pin per fd)
void TTDevice::open_hugepage_per_host_mem_ch(uint32_t num_host_mem_channels) {
    for (int ch = 0; ch < num_host_mem_channels; ch++) {
        int device_fd_for_host_mem = find_device(index);
        if (device_fd_for_host_mem == -1) {
            throw std::runtime_error(
                std::string("Failed opening a host memory device handle for device ") + std::to_string(index));
        }
        device_fd_per_host_ch.push_back(device_fd_for_host_mem);
    }
}


static int get_revision_id(TTDevice *dev) {

    static const char pattern[] = "/sys/bus/pci/devices/%04x:%02x:%02x.%u/revision";
    char buf[sizeof(pattern)];
    std::snprintf(buf, sizeof(buf), pattern,
    (unsigned int)dev->pci_domain, (unsigned int)dev->pci_bus, (unsigned int)dev->pci_device, (unsigned int)dev->pci_function);

    std::ifstream revision_file(buf);
    std::string revision_string;
    if (std::getline(revision_file, revision_string)) {
        return std::stoi(revision_string, nullptr, 0);
    } else {
        throw std::runtime_error("Revision ID read failed for device");
    }
}

static int get_config_space_fd(TTDevice *dev) {
    if (dev->sysfs_config_fd == -1) {
        static const char pattern[] = "/sys/bus/pci/devices/0000:%02x:%02x.%u/config";
        char buf[sizeof(pattern)];
        std::snprintf(buf, sizeof(buf), pattern,
                      (unsigned int)dev->pci_bus, (unsigned int)dev->pci_device, (unsigned int)dev->pci_function);
        dev->sysfs_config_fd = open(buf, O_RDWR);

        if (dev->sysfs_config_fd == -1) {
            dev->sysfs_config_fd = open(buf, O_RDONLY);
        }
    }

    return dev->sysfs_config_fd;
}

static uint64_t read_bar0_base(TTDevice *dev) {
    const std::uint64_t bar_address_mask = ~(std::uint64_t)0xF;
    unsigned int bar0_config_offset = 0x10;

    std::uint64_t bar01;
    if (pread(get_config_space_fd(dev), &bar01, sizeof(bar01), bar0_config_offset) != sizeof(bar01)) {
        return 0;
    }

    return bar01 & bar_address_mask;
}

// TODO(jms): why can't PCIDevice and TTDevice be the same thing?
// Do we not have enough classes with the word device in the name?
PCIdevice ttkmd_open(DWORD device_id)
{
    TTDevice *ttdev = new TTDevice(device_id);

    PCIdevice device;
    device.id = device_id;
    device.hdev = ttdev;
    device.vendor_id = ttdev->device_info.vendor_id;
    device.device_id = ttdev->device_info.device_id;
    device.subsystem_vendor_id = ttdev->device_info.subsystem_vendor_id;
    device.subsystem_id = ttdev->device_info.subsystem_id;
    device.dwBus = ttdev->pci_bus;
    device.dwSlot = ttdev->pci_device;
    device.dwFunction = ttdev->pci_function;
    device.BAR_addr = read_bar0_base(ttdev);
    device.BAR_size_bytes = ttdev->bar0_uc_size;
    device.revision_id = get_revision_id(ttdev);

    return device;
}

static bool is_char_dev(const dirent *ent, const char *parent_dir) {
    if (ent->d_type == DT_UNKNOWN || ent->d_type == DT_LNK) {
        char name[2 * NAME_MAX + 2];
        strcpy(name, parent_dir);
        strcat(name, "/");
        strcat(name, ent->d_name);

        struct stat stat_result;
        if (stat(name, &stat_result) == -1) {
            return false;
        }

        return ((stat_result.st_mode & S_IFMT) == S_IFCHR);
    } else {
        return (ent->d_type == DT_CHR);
    }
}


std::vector<int> ttkmd_scan() {

    static const char dev_dir[] = "/dev/tenstorrent";

    std::vector<int> found_devices;

    DIR *d = opendir(dev_dir);
    if (d != nullptr) {
        while (true) {
            const dirent *ent = readdir(d);
            if (ent == nullptr) {
                break;
            }

            // strtoul allows initial whitespace, +, -
            if (!std::isdigit(ent->d_name[0])) {
                continue;
            }

            if (!is_char_dev(ent, dev_dir)) {
                continue;
            }

            char *endptr = nullptr;
            errno = 0;
            auto index = std::strtoul(ent->d_name, &endptr, 10);
            if (index == std::numeric_limits<unsigned int>::max() && errno == ERANGE) {
                continue;
            }
            if (*endptr != '\0') {
                continue;
            }

            found_devices.push_back((int)index);
        }
        closedir(d);
    }

    std::sort(found_devices.begin(), found_devices.end());

    return found_devices;
}