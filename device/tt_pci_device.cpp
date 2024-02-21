#include "tt_pci_device.h"

#include <fcntl.h>
#include <linux/pci.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdexcept>
#include <cstring>
#include <vector>

#include "ioctl.h"


static uint32_t GS_BAR0_WC_MAPPING_SIZE = (156<<20) + (10<<21) + (18<<24);
static const uint16_t WORMHOLE_DEVICE_ID = 0x401e;

// TODO(jms): are we scared of just using the constructor??
TTDevice TTDevice::open(unsigned int device_id) {
    TTDevice ttdev;
    static int unique_id = 0;
    ttdev.index = device_id;
    ttdev.do_open();

    return ttdev;
}

// TODO(jms): This is a strange way to spell "open".
// TODO(jms): There's another one of these in tt_silicon_driver.cpp
static int find_device(const uint16_t device_id) {
    const char device_name_pattern[] = "/dev/tenstorrent/%u";
    // returns device id if found, otherwise -1
    char device_name[sizeof(device_name_pattern) + std::numeric_limits<unsigned int>::digits10];
    std::snprintf(device_name, sizeof(device_name), device_name_pattern, (unsigned int)device_id);
    int device_fd = ::open(device_name, O_RDWR | O_CLOEXEC);
    return device_fd;
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

inline void TTDevice::do_open() {
    device_fd = find_device(index);
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

void TTDevice::reset() {
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

    drop();
}

void TTDevice::drop() {
    device_fd = -1;
    bar0_uc = nullptr;
    bar0_wc = nullptr;
    system_reg_mapping = nullptr;
    dma_buffer_mappings.clear();
    sysfs_config_fd = -1;
}
