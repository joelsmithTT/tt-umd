// This code is part of a scheme to put the kernel driver in control of the
// sysmem buffer.
//
// The intent of this code is to allocate 1G hugepages on a per-device basis and
// pass the pages to the kernel driver.  It is recommended that this be done as
// a one-time setup operation, as the driver will not relinquish the hugepages
// until the this setup operation is repeated or the driver is unloaded.
//
// The driver's responsibility is to:
// - Use these pages to back the sysmem buffer
// - Ensure that the pages are associated with the correct NUMA node
// - Configure the hardware so that the buffer can be accessed via the NOC
// - Provide userspace with a mechanism to mmap the buffer
// - Provide the illusion that the buffer is contiguous to hardware and software
// - Maintain hardware configuration across card resets
//
// The userspace setup program's responsibility is to:
// - Map hugepages on a per-device basis on the correct NUMA node
// - Tell the driver about the pages
//
// Q & A
//
// Q: What is this sysmem buffer?
// A: Large buffers used for DMA between host and device.
//
// Q: What is the difference between the old and new schemes?
// A: Old scheme:
//    - At runtime, UMD allocates/maps 1G pages
//    - At runtime, UMD tells driver & fimware about the pages
//    - KMD hangs onto the pages for the lifetime of the user process
//    - Firmware tells the hardware about the pages
//    New scheme:
//    - Onetime setup: userspace allocates/maps 1G pages, tells kernel
//    - Onetime setup: kernel tells hardware; kernel hangs onto pages
//    - At runtime, UMD mmaps a virtually contiguous buffer backed by the pages
//
// Q: What is the DMA mechanism?
// A: Hardware accesses the buffer via the NOC, with access directed at the PCIe
//    tile with address ranges:
//      Grayskull:  0x0_0000_0000 to 0x0_FFFD_FFFF
//      Wormhole:   0x8_0000_0000 to 0x8_FFFD_FFFF
//    Note that this scheme does not use the PCIe IP's DMA controller, but does
//    rely on the PCIe IP's address translation capabilities.
//
// Q: Why should the kernel control the sysmem buffer?
// A: Makes it easier to share the buffer between user and platform concerns.
//    Makes it easier to support the system IOMMU.
//
// Q: How can software access the buffer? 
// A: By using the driver's ioctl interface to query for its mmap size/offset,
//    and then calling mmap on the device file descriptor.
//
// Q: Why does the kernel need to be told about the huge pages?
// A: Allocating large memory regions that appear contiguous to the hardware is
//    difficult in older kernels.  Newer kernels have features that make this
//    easier in both the no IOMMU (use CMA) and IOMMU cases, but these features
//    are unavailable in 5.4 (Ubuntu 20.04) and 5.15 (Ubuntu 22.04).  To support
//    these kernels, it is convenient to use 1G hugepages.
//
// Q: What kernel supports allocating sysmem buffers without hugepages?
// A: Kernel 6.5 (Ubuntu 22.04 HWE) supports it with and without IOMMU enabled.

#include <fcntl.h>
#include <numa.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <filesystem>
#include <iostream>

#include "device/ioctl.h"

static const size_t ONE_GIG = 1024 * 1024 * 1024;
static const size_t NUM_HUGEPAGES_PER_CARD = 4;

static void reclaim_hugepages_from_hugetlbfs()
{
    // Remove anything with 'tenstorrent' in /dev/hugepages-1G
    for (const auto &entry : std::filesystem::directory_iterator("/dev/hugepages-1G")) {
        if (entry.path().string().find("tenstorrent") != std::string::npos) {
            std::filesystem::remove(entry.path());
        }
    }
}

int hugepage_setup_for_device(int fd)
{
    tenstorrent_hugepage_setup hugepage_setup{};

    reclaim_hugepages_from_hugetlbfs();

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
        int flags = MAP_SHARED | MAP_POPULATE | MAP_ANONYMOUS| MAP_HUGETLB | (30 << MAP_HUGE_SHIFT);
        int fd = -1;
        off_t offset = 0;
        void *mapping = mmap(NULL, ONE_GIG, prot, flags, fd, offset);

        if (mapping != MAP_FAILED) {
            hugepage_setup.virt_addrs[i] = reinterpret_cast<uint64_t>(mapping);
        } else {
            std::cerr << "Failed to map a hugepage, check /proc/cmdline" << std::endl;
            return 1;
        }
    }

    // Tell the driver about the hugepages we've mapped.
    if (ioctl(fd, TENSTORRENT_IOCTL_HUGEPAGE_SETUP, &hugepage_setup)) {
        std::cerr << "Hugepage setup failed, check dmesg" << std::endl;
        return 1;
    }

    return 0;
}

int hugepage_setup_for_all_devices()
{
    std::error_code ec;
    for (const auto &entry : std::filesystem::directory_iterator("/dev/tenstorrent", ec)) {
        int fd = open(entry.path().c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            std::cerr << "Failed to open " << entry.path() << std::endl;
            return 1;
        }

        if (hugepage_setup_for_device(fd)) {
            return 1;
        }
    }
    if (ec) {
        std::cerr << "Error reading /dev/tenstorrent: " << ec.message() << std::endl;
        return 1;
    }

    return 0;
}
