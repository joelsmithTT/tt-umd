#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <iostream>

#include "tt_device.h"
#include "driver_atomics.h"
#include "device/grayskull_implementation.h"
#include "src/firmware/riscv/wormhole/l1_address_map.h"
#include "src/firmware/riscv/wormhole/host_mem_address_map.h"
#include "src/firmware/riscv/wormhole/eth_interface.h"
#include "src/firmware/riscv/wormhole/eth_l1_address_map.h"

#include "fmt/format.h"

#include <chrono>

static const tt_cxy_pair PCIE_GS(0, 0, 4);
static const tt_cxy_pair PCIE_WH(0, 0, 3);
static const tt_cxy_pair PCIE = PCIE_WH;
static const size_t ONE_GIG = 1024 * 1024 * 1024;
static const size_t ONE_MEG = 1024 * 1024;
static const size_t ONE_KB = 1024;
static const size_t PCIE_BASE_WH = 0x800000000ULL;

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

void sysmem_via_pcie_write32(tt_SiliconDevice &device, uint64_t addr, uint32_t val)
{
    addr |= PCIE_BASE_WH;
    device.write_to_device(&val, sizeof(val), PCIE, addr, "REG_TLB");
}

uint32_t sysmem_via_pcie_read32(tt_SiliconDevice &device, uint64_t addr)
{
    uint32_t val;
    addr |= PCIE_BASE_WH;
    device.read_from_device(&val, PCIE, addr, sizeof(val), "REG_TLB");
    return val;
}

void dump_iatu_config(tt_SiliconDevice &device, int region)
{
    auto read_iatu_reg = [&](uint64_t addr) {
        uint32_t v;
        addr |= 0x8'0000'0000;
        device.read_from_device(&v, PCIE, addr, 4, "LARGE_READ_TLB");
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
    auto write_iatu_reg = [&](uint32_t addr, uint64_t val) {
        device.write_to_device(&val, sizeof(val), PCIE, addr, "REG_TLB");
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

void set_params_for_remote_txn(tt_SiliconDevice& device) {
    // Populate address map and NOC parameters that the driver needs for remote transactions
    device.set_driver_host_address_params({host_mem::address_map::ETH_ROUTING_BLOCK_SIZE, host_mem::address_map::ETH_ROUTING_BUFFERS_START});

    device.set_driver_eth_interface_params({NOC_ADDR_LOCAL_BITS, NOC_ADDR_NODE_ID_BITS, ETH_RACK_COORD_WIDTH, CMD_BUF_SIZE_MASK, MAX_BLOCK_SIZE,
                                            REQUEST_CMD_QUEUE_BASE, RESPONSE_CMD_QUEUE_BASE, CMD_COUNTERS_SIZE_BYTES, REMOTE_UPDATE_PTR_SIZE_BYTES,
                                            CMD_DATA_BLOCK, CMD_WR_REQ, CMD_WR_ACK, CMD_RD_REQ, CMD_RD_DATA, CMD_BUF_SIZE, CMD_DATA_BLOCK_DRAM, ETH_ROUTING_DATA_BUFFER_ADDR,
                                             REQUEST_ROUTING_CMD_QUEUE_BASE, RESPONSE_ROUTING_CMD_QUEUE_BASE, CMD_BUF_PTR_MASK, CMD_ORDERED, CMD_BROADCAST});
    
    device.set_device_l1_address_params({l1_mem::address_map::NCRISC_FIRMWARE_BASE, l1_mem::address_map::FIRMWARE_BASE,
                                  l1_mem::address_map::TRISC0_SIZE, l1_mem::address_map::TRISC1_SIZE, l1_mem::address_map::TRISC2_SIZE,
                                  l1_mem::address_map::TRISC_BASE, l1_mem::address_map::L1_BARRIER_BASE, eth_l1_mem::address_map::ERISC_BARRIER_BASE, eth_l1_mem::address_map::FW_VERSION_ADDR});

}

std::unique_ptr<tt_SiliconDevice> lol_umd(std::set<chip_id_t> target_devices, std::string arch_yaml)
{
    uint32_t num_host_mem_ch_per_mmio_device = 4;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config;
    dynamic_tlb_config["REG_TLB"] = tt::umd::grayskull::REG_TLB;
    tt_device_params default_params;
    std::string soc_desc = fmt::format("../tests/soc_descs/{}.yaml", arch_yaml);
    auto device = std::make_unique<tt_SiliconDevice>(soc_desc,
                                            "ethernet-map.yaml",
                                            target_devices,
                                            num_host_mem_ch_per_mmio_device,
                                            dynamic_tlb_config,
                                            false,
                                            true,
                                            true);
    // set_params_for_remote_txn(*device);
    // device->start_device(default_params);
    // device->deassert_risc_reset();
    return device;
}

// For the sake of retracing my steps later:
// joel@sjc-lab-t7003 ~/gh/tt-umd/build $ ../device/bin/silicon/x86/create-ethernet-map ethernet-map.yaml
// ethernet-map.yaml says,
// arch: { 0: Wormhole, 1: Wormhole, 2: Grayskull, 3: Wormhole, 4: Wormhole, }
// chips_with_mmio: [ 0: 0, 1: 1, 2: 2, ]

std::vector<std::unique_ptr<tt_SiliconDevice>> lol_umd2()
{
    std::vector<std::unique_ptr<tt_SiliconDevice>> devices;

    devices.push_back(lol_umd({0,1}, "wormhole_b0_8x10"));
    return devices;
}

// because doing it over the NOC is shitting the bed
uint32_t *map_wh_bar2_for_iatu_inspection()
{
    int fd = open("/dev/tenstorrent/0", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        throw std::runtime_error("Failed to open device");
    }

    size_t size = ONE_MEG;
    off_t offset = (2ULL) << 32;    // BAR2, UC
    void *mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);

    if (mem == MAP_FAILED) {
        throw std::runtime_error("Failed to mmap device");
    }
    return reinterpret_cast<uint32_t*>(mem);
}

void* map_buffer()
{
    int fd = open("/dev/tenstorrent/0", O_RDWR | O_CLOEXEC);

    if (fd < 0) {
        throw std::runtime_error("Failed to open device");
    }

    size_t size = 4ULL * ONE_GIG;
    off_t offset = (6ULL) << 32;
    uint8_t *mem = (uint8_t*)mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);

    if (mem == MAP_FAILED) {
        throw std::runtime_error("Failed to mmap device");
    }

    std::memset(mem, 0xff, 1ULL << 30);
    std::memset(mem + 1 * (1ULL << 30), 0x55, 1ULL << 30);
    std::memset(mem + 2 * (1ULL << 30), 0xAA, 1ULL << 30);
    std::memset(mem + 3 * (1ULL << 30), 0x22, 1ULL << 30);

    return mem;
}


uint32_t rand32()
{
    return rand() & 0xFFFFFFFF;
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

bool run_test2(tt_SiliconDevice &device, uint8_t *virt_addr)
{
    size_t FAIL_THRESHOLD = 1;
    size_t size = 4ULL * ONE_GIG;
    size_t top = 0xfffe'0000;

    // write a pattern to the section of the buffer that HW will read.
    uint32_t *base = reinterpret_cast<uint32_t*>(virt_addr);
    for (size_t i = 0; i < size / sizeof(uint32_t); ++i) {
        base[i] = 4 * (uint32_t)i;
    }
    base[0] = 0xdeadbeef;
    base[size/4 - 1] = 0xdeadbeef;

    Timestamp ts;
    size_t fails = 0;
    size_t inc = 0x1000;
    // std::vector<ssize_t> small_offsets { -16, -8, -4, 0, 4, 8, 16 };
    std::vector<ssize_t> small_offsets { 0 };
    for (ssize_t addr = 0; addr < top; addr += inc) {
        double percent = (double)addr / (double)top * 100;
        // std::cout << "\r" << std::dec << percent << "%                      ";
        for (ssize_t small_offset : small_offsets) {
            if (small_offset + addr < 0) {
                continue;
            }
            if (small_offset + addr >= top) {
                continue;
            }

            size_t test_address = addr + small_offset;
            uint32_t expected_val = *reinterpret_cast<uint32_t*>(virt_addr + test_address);
            uint32_t pcie_val = sysmem_via_pcie_read32(device, test_address);

            if (test_address == 0) {
                std::cout << "0x" << std::hex << test_address << " ->>>> 0x" << expected_val << std::endl;
                std::cout << pcie_val << std::endl;
            }

            // std::cout << "0x" << test_address << " ->>>> 0x" << expected_val << std::endl;

            if (pcie_val != expected_val) {
                std::cerr << "\tPCIe Read Mismatch @ 0x" << std::hex << test_address;
                std::cerr << "\tactual: 0x" << std::hex << pcie_val << " != expected: 0x" << expected_val << std::endl;
                ++fails;
            }
            if (fails >= FAIL_THRESHOLD) {
                break;
            }
        }
        if (fails >= FAIL_THRESHOLD) {
            break;
        }
    }
    bool ok = (fails == 0);
    auto elapsed_ms = ts.milliseconds();
    std::cout << (ok ? "PASSED" : "FAILED") << " in " << std::dec << elapsed_ms << " ms" << std::endl;
    return ok;
}

void read_iatu_via_bar2(size_t region)
{
    uint32_t *bar2 = map_wh_bar2_for_iatu_inspection();

    auto read_iatu_reg = [&](uint64_t addr) {
        uint64_t outbound_base = 0x1200 + (0x200 * region);
        return bar2[(outbound_base + addr)/4];
    };

    auto ctrl1 = read_iatu_reg(0x00);
    auto crtl2 = read_iatu_reg(0x04);
    auto lower_base = read_iatu_reg(0x08);
    auto upper_base = read_iatu_reg(0x0C);
    auto limit = read_iatu_reg(0x10);
    auto lower_target = read_iatu_reg(0x14);
    auto upper_target = read_iatu_reg(0x18);
    auto base = (uint64_t)upper_base << 32 | lower_base;
    auto target = (uint64_t)upper_target << 32 | lower_target;

    std::cout << "iATU Region " << region << " configuration:" << std::endl;
    std::cout << "\tCTRL1: 0x" << std::hex << ctrl1 << std::endl;
    std::cout << "\tCTRL2: 0x" << std::hex << crtl2 << std::endl;
    std::cout << "\tLIMIT: 0x" << std::hex << limit << std::endl;
    std::cout << "\tTRGET: 0x" << std::hex << target << std::endl;
    std::cout << "\t BASE: 0x" << std::hex << base << std::endl;

    std::cout << "\n";
}


int main(int argc, char **argv)
{
    read_iatu_via_bar2(0);
    read_iatu_via_bar2(1);
    read_iatu_via_bar2(2);
    read_iatu_via_bar2(3);
    auto devices = lol_umd2();
    uint8_t *virt_addr = (uint8_t*)map_buffer();
    run_test2(*devices[0], virt_addr);

    return 0;
}
