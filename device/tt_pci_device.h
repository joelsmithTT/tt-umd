#pragma once

#include <cstdint>
#include <vector>

#include "kmdif.h"  // Kernel Mode Driver Interface (KMDIF)
#include "ioctl.h"


struct TTDeviceBase
{
    unsigned int index;

    int device_fd = -1;
    std::vector<int> device_fd_per_host_ch; // TODO(jms): wtf is this?
    void *bar0_uc = nullptr;
    std::size_t bar0_uc_size = 0;
    std::size_t bar0_uc_offset = 0;

    void *bar0_wc = nullptr;
    std::size_t bar0_wc_size = 0;

    void *system_reg_mapping = nullptr;
    std::size_t system_reg_mapping_size;

    void *system_reg_wc_mapping = nullptr;
    std::size_t system_reg_wc_mapping_size;

    std::uint32_t system_reg_start_offset;  // Registers >= this are system regs, use the mapping.
    std::uint32_t system_reg_offset_adjust; // This is the offset of the first reg in the system reg mapping.

    int sysfs_config_fd = -1;
    std::uint16_t pci_domain;
    std::uint8_t pci_bus;
    std::uint8_t pci_device;
    std::uint8_t pci_function;

    unsigned int next_dma_buf = 0;

	DMAbuffer dma_completion_flag_buffer;  // When DMA completes, it writes to this buffer
	DMAbuffer dma_transfer_buffer;         // Buffer for large DMA transfers

    std::uint32_t max_dma_buf_size_log2;

    tenstorrent_get_device_info_out device_info;

    std::vector<DMAbuffer> dma_buffer_mappings;
};

struct TTDevice : TTDeviceBase
{
    static TTDevice open(unsigned int device_id);
    void open_hugepage_per_host_mem_ch(uint32_t num_host_mem_channels);
    ~TTDevice() { reset(); }

    TTDevice(const TTDevice&) = delete;
    void operator = (const TTDevice&) = delete;

    TTDevice(TTDevice &&that) : TTDeviceBase(std::move(that)) { that.drop(); }
    TTDevice &operator = (TTDevice &&that) {
        reset();

        *static_cast<TTDeviceBase*>(this) = std::move(that);
        that.drop();

        return *this;
    }

    void suspend_before_device_reset() {
        reset();
    }

    void resume_after_device_reset() {
        do_open();
    }

private:
    TTDevice() = default;

    void reset();
    void drop();
    void do_open();
};
