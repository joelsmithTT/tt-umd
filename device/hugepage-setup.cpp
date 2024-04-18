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

#include "hugepages.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    int r = hugepage_setup_for_all_devices();
    std::cout << (r == 0 ? "OK" : "FAIL") << std::endl;
    return r;
}
