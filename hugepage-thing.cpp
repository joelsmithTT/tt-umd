#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <iostream>

#include "device/grayskull_implementation.h"
#include "device/ioctl.h"
#include "driver_atomics.h"
#include "tt_device.h"

#include "fmt/format.h"

static const tt_cxy_pair PCIE_GS(0, 0, 4);
static const size_t ONE_GIG = 1024 * 1024 * 1024;
static const size_t ONE_MEG = 1024 * 1024;
static const size_t ONE_KB = 1024;

class Timestamp
{
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
public:
    Timestamp() : start(std::chrono::high_resolution_clock::now()) {}

    void reset()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    uint64_t milliseconds()
    {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
        reset();
        return ms;
    }

    uint64_t microseconds()
    {
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
        reset();
        return us;
    }

    void bench(const std::string &task, size_t bytes)
    {
        auto us = microseconds();
        auto rate = bytes / (double)us;
        rate /= 1000.;
        std::cout << task << " took " << std::dec << us << " us for " << std::dec << bytes << " bytes ";
        std::cout << "(" << rate << " GB/s)" << std::endl;
        reset();
    }

    void bench(const std::string &task)
    {
        auto ms = milliseconds();
        std::cout << task << " took " << ms << " ms " << std::endl;
        reset();
    }
};

void sysmem_via_pcie_write32(tt_SiliconDevice &device, uint32_t addr, uint32_t val)
{
    device.write_to_device(&val, sizeof(val), PCIE_GS, addr, "REG_TLB");
}

uint32_t sysmem_via_pcie_read32(tt_SiliconDevice &device, uint32_t addr)
{
    uint32_t val;
    device.read_from_device(&val, PCIE_GS, addr, sizeof(val), "REG_TLB");
    return val;
}

void dump_iatu_config(tt_SiliconDevice &device, int region)
{
    auto read_iatu_reg = [&](uint32_t addr) {
        uint32_t v;
        device.read_from_device(&v, PCIE_GS, addr, 4, "REG_TLB");
        return v;
    };

    auto ARC_RESET = 0x1FF30000;
    auto PCI_RESERVED = 0x0078;
    auto DBI = ARC_RESET | PCI_RESERVED;
    uint64_t regs = 0x300000 + (0x200 * region);

    device.bar_write32(0, DBI, 0x00200000);
    device.bar_write32(0, DBI + 4, 0x00200000);

    auto ctrl1 = read_iatu_reg(regs + 0x00);
    auto crtl2 = read_iatu_reg(regs + 0x04);
    auto lower_base = read_iatu_reg(regs + 0x08);
    auto upper_base = read_iatu_reg(regs + 0x0C);
    auto limit = read_iatu_reg(regs + 0x10);
    auto lower_target = read_iatu_reg(regs + 0x14);
    auto upper_target = read_iatu_reg(regs + 0x18);
    auto base = (uint64_t)upper_base << 32 | lower_base;
    auto target = (uint64_t)upper_target << 32 | lower_target;

    std::cout << "iATU Region " << region << " configuration:" << std::endl;
    std::cout << "\tCTRL1: 0x" << std::hex << ctrl1 << std::endl;
    std::cout << "\tCTRL2: 0x" << std::hex << crtl2 << std::endl;
    std::cout << "\tLIMIT: 0x" << std::hex << limit << std::endl;
    std::cout << "\tTRGET: 0x" << std::hex << target << std::endl;
    std::cout << "\t BASE: 0x" << std::hex << base << std::endl;

    device.bar_write32(0, DBI, 0);
    device.bar_write32(0, DBI + 4, 0);
    std::cout << "\n";
}

void program_iatu(tt_SiliconDevice &device, int region, uint64_t bus_address, uint64_t chip_address, size_t limit)
{
    auto write_iatu_reg = [&](uint32_t addr, uint32_t val) {
        device.write_to_device(&val, sizeof(val), PCIE_GS, addr, "REG_TLB");
    };

    auto ARC_RESET = 0x1FF30000;
    auto PCI_RESERVED = 0x0078;
    auto DBI = ARC_RESET | PCI_RESERVED;
    uint64_t regs = 0x300000 + (0x200 * region);

    device.bar_write32(0, DBI, 0x00200000);
    device.bar_write32(0, DBI + 4, 0x00200000);

    uint32_t ctrl1 = 0x00000000;
    uint32_t crtl2 = limit == 0 ? 0 : 0x88280000;
    uint32_t lower_base = chip_address & 0xFFFFFFFF;
    uint32_t upper_base = (chip_address >> 32) & 0xFFFFFFFF;
    limit = limit == 0 ? 0x0 : limit - 1;
    uint32_t lower_target = bus_address & 0xFFFFFFFF;
    uint32_t upper_target = (bus_address >> 32) & 0xFFFFFFFF;

    write_iatu_reg(regs + 0x00, ctrl1);
    write_iatu_reg(regs + 0x04, crtl2);
    write_iatu_reg(regs + 0x08, lower_base);
    write_iatu_reg(regs + 0x0C, upper_base);
    write_iatu_reg(regs + 0x10, limit);
    write_iatu_reg(regs + 0x14, lower_target);
    write_iatu_reg(regs + 0x18, upper_target);

    device.bar_write32(0, DBI, 0);
    device.bar_write32(0, DBI + 4, 0);
}

std::unique_ptr<tt_SiliconDevice> lol_umd()
{
    std::set<chip_id_t> target_devices = {0};
    uint32_t num_host_mem_ch_per_mmio_device = 4;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config;
    dynamic_tlb_config["REG_TLB"] = tt::umd::grayskull::REG_TLB;
    tt_device_params default_params;
    auto device = std::make_unique<tt_SiliconDevice>("../tests/soc_descs/grayskull_10x12.yaml",
                                            "",
                                            target_devices,
                                            num_host_mem_ch_per_mmio_device,
                                            dynamic_tlb_config,
                                            false,
                                            true,
                                            true);
    device->start_device(default_params);
    device->deassert_risc_reset();
    return device;
}

void* map_buffer()
{
    int fd = open("/dev/tenstorrent/0", O_RDWR | O_CLOEXEC);

    if (fd < 0) {
        throw std::runtime_error("Failed to open device");
    }

    size_t size = 4ULL * ONE_GIG;
    off_t offset = (6ULL) << 32;
    void *mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);

    if (mem == MAP_FAILED) {
        throw std::runtime_error("Failed to mmap device");
    }

    std::memset(mem, 0xff, size);

    return mem;
}

inline uint64_t rand64(void)
{
    static uint64_t x = 123456789;
    static uint64_t y = 362436069;
    static uint64_t z = 521288629;

    uint64_t t;
    x ^= x << 16;
    x ^= x >> 5;
    x ^= x << 1;

    t = x;
    x = y;
    y = z;
    z = t ^ x ^ y;

    return z;
}

void fill_buffer_with_random_numbers_fast(void *buffer, size_t size)
{
    uint64_t *base = reinterpret_cast<uint64_t*>(buffer);
    size_t count = size / sizeof(uint64_t);
    std::fill(base, base + count, rand64());
}

void iatu_helper(tt_SiliconDevice &device, uint64_t bus_addr, uint64_t chip_addr, size_t size, size_t num_regions_to_use)
{
    size_t chunk_size = size / num_regions_to_use;
    for (size_t i = 0; i < num_regions_to_use; ++i) {
        size_t limit = chunk_size * (i + 1);
        program_iatu(device, i, bus_addr + (i * chunk_size), chip_addr + (i * chunk_size), limit);
    }
    for (size_t i = 0; i < num_regions_to_use; ++i) {
        dump_iatu_config(device, i);
    }
}

void do_hugepage_things(int dev_fd, const std::string &name, size_t num_hugepages)
{
    std::vector<void*> mappings;

    auto old_umask = umask(0);
    for (size_t i = 0; i < num_hugepages; ++i) {
        auto filename = fmt::format("/dev/hugepages-1G/device_{}_channel_{}_tenstorrent", name, i);
        int fd = open(filename.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH );
        if (fd == -1 && errno == EACCES) {
            unlink(filename.data());
            fd = open(filename.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH );

        }

        if (fd) {
            void *mapping = mmap(nullptr, ONE_GIG, PROT_READ|PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd, 0);
            close(fd);

            if (mapping != MAP_FAILED) {
                mappings.push_back(mapping);
            }
        }
    }
    umask(old_umask);

    struct tenstorrent_hugepage_setup s{};
    s.num_hugepages = mappings.size();
    for (size_t i = 0; i < mappings.size(); ++i) {
        s.virt_addrs[i] = reinterpret_cast<uint64_t>(mappings[i]);
    }

    int r = ioctl(dev_fd, TENSTORRENT_IOCTL_HUGEPAGE_SETUP, &s);
    if (r) {
        std::cerr << "TENSTORRENT_IOCTL_HUGEPAGE_SETUP failed" << std::endl;
    }

    for (auto mapping : mappings) {
        munmap(mapping, ONE_GIG);
    }
}

void huge_for_each_card_seen_by_ttkmd()
{
    for (const auto &entry : std::filesystem::directory_iterator("/dev/tenstorrent")) {
        int fd = open(entry.path().c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            std::cout << "Failed to open " << entry.path() << std::endl;
            continue;
        }

        // name is the last part of the path, e.g. the 0 in /dev/tenstorrent/0
        std::string name = entry.path().filename().string();
        do_hugepage_things(fd, name, 4);
    }
}

int main(int argc, char **argv)
{
    huge_for_each_card_seen_by_ttkmd();
    return 0;
}
