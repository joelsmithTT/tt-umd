/*
This program configures 1G hugepages for Grayskull and Wormhole cards.  It is a
component in a scheme to administer buffers in host memory that are accessible
via DMA to Tensix cores on Grayskull or Wormhole chips.

Context:
- Tensix and Ethernet cores can access the system bus via the PCIe core.
- Approximately 32 bits of address space available to a Tensix to do this.
- Address translation is performed by the PCIe core.
- Addresses map to physical memory, which is backed by 1G hugepages.

Goals:
- Enable support of the system IOMMU.
- Share the 32 bit address space between user and platform software.

Assumptions:
- Adequate hugepages have been allocated by the system.

Technique:
- Driver provides interface to mmap Tensix DMA buffers on a per-device basis.
- Userspace maps ~ 4GiB buffer for each device.
- Top 256MiB of each buffer is reserved for driver use.
- Underlying buffer is 4x 1G hugepages.
- One-time setup: driver maps the buffer to the Tensix DMA address space.

This program performs the one-time setup.

Helpful Commands:
cat /sys/kernel/mm/hugepages/hugepages-1048576kB/nr_hugepages
cat /sys/kernel/mm/hugepages/hugepages-1048576kB/free_hugepages
cat /sys/devices/system/node/node0/hugepages/hugepages-1048576kB/free_hugepages
cat /sys/devices/system/node/node1/hugepages/hugepages-1048576kB/free_hugepages
cat /sys/devices/system/node/node0/hugepages/hugepages-1048576kB/nr_hugepages
cat /sys/devices/system/node/node1/hugepages/hugepages-1048576kB/nr_hugepages
*/

#include <fcntl.h>
#include <numa.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <filesystem>
#include <iostream>

#include "device/ioctl.h"

static const size_t ONE_GIG = 1024 * 1024 * 1024;
static const size_t NUM_HUGEPAGES_PER_CARD = 4;

static int hugepage_setup(int fd)
{
    tenstorrent_hugepage_setup hugepage_setup{};

    // Cause the driver to relinquish any hugepages it has already mapped.
    hugepage_setup.num_hugepages = 0;
    ioctl(fd, TENSTORRENT_IOCTL_HUGEPAGE_SETUP, &hugepage_setup);


    // Retrieve information about the device; this is done to determine the NUMA
    // node associated with the device (if any).
    tenstorrent_get_device_info info{};
    info.in.output_size_bytes = sizeof(info.out);
    if (ioctl(fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &info)) {
        std::cerr << "Error getting device info" << std::endl;
        return 1;
    }

    // If the device is associated with a NUMA node, run on that node so that
    // the hugepages will be taken from that node's `free_hugepages` pool.
    // If we have a NUMA system and hugepages were configured normally, Linux 
    // should have split the hugepages equally between the nodes.
    if (info.out.numa_node >= 0) {
        numa_run_on_node(info.out.numa_node);
    }

    // Map the hugepages.
    hugepage_setup.num_hugepages = NUM_HUGEPAGES_PER_CARD;
    for (size_t i = 0; i < NUM_HUGEPAGES_PER_CARD; ++i) {
        int prot = PROT_READ | PROT_WRITE;
        int flags = MAP_SHARED | MAP_POPULATE | MAP_ANONYMOUS | MAP_HUGETLB | (30 << MAP_HUGE_SHIFT);
        int fd = -1;
        off_t offset = 0;
        void *mapping = mmap(NULL, ONE_GIG, prot, flags, fd, offset);

        if (mapping != MAP_FAILED) {
            hugepage_setup.virt_addrs[i] = reinterpret_cast<uint64_t>(mapping);
        } else {
            std::cerr << "Failed to map hugepage " << i << std::endl;
            return 1;
        }
    }

    // Tell the driver about the hugepages we've mapped.  Once we've done this,
    // the hardware (i.e. Tensix and Ethernet cores) will start using them when
    // accessing the system bus via the PCIe core.
    if (ioctl(fd, TENSTORRENT_IOCTL_HUGEPAGE_SETUP, &hugepage_setup)) {
        if (ioctl(fd, TENSTORRENT_IOCTL_HUGEPAGE_SETUP, &hugepage_setup)) {
            std::cerr << "Hugepage setup failed, check dmesg" << std::endl;
            return 1;
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    std::error_code ec;
    for (const auto &entry : std::filesystem::directory_iterator("/dev/tenstorrent", ec)) {
        int fd = open(entry.path().c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            std::cerr << "Failed to open " << entry.path() << std::endl;
            return 1;
        }

        if (hugepage_setup(fd)) {
            return 1;
        }
    }
    if (ec) {
        std::cerr << "Error reading /dev/tenstorrent: " << ec.message() << std::endl;
        return 1;
    }

    std::cout << "OK" << std::endl;
    return 0;
}
