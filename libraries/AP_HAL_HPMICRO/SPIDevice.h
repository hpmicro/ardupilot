/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_HAL_HPM.h"
#include <AP_HAL/utility/OwnPtr.h>

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"
#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_dma_drv.h"

namespace HPMicro
{

struct SPIDeviceDesc {
    const char *name;
    uint8_t bus;
    uint8_t device;
    uint32_t cs;
    uint32_t af;
    uint32_t baf;
    uint32_t paf;
    uint16_t mode;
    uint32_t lspeed;
    uint32_t hspeed;
};

struct SPIBusDesc {
    SPI_Type *host;
    clock_name_t host_clk;
    DMA_Type *dma;
    int dma_irqn;
    int dma_ch_tx;
    int dma_tx_src;
    int dma_ch_rx;
    int dma_rx_src;
    int mosi;
    int miso;
    int sclk;
    int mosi_af;
    int miso_af;
    int sclk_af;
    int mosi_baf;
    int miso_baf;
    int sclk_baf;
    int mosi_paf;
    int miso_paf;
    int sclk_paf;
};
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);
/* @brief Channel config */
typedef struct dma_handshake_config_fixed {
    uint32_t dst;
    uint32_t src;
    uint32_t size_in_byte;
    uint8_t data_width;            /* data width, value defined by DMA_TRANSFER_WIDTH_xxx */
    uint8_t ch_index;
    bool dst_fixed;
    bool src_fixed;
    uint16_t interrupt_mask;        /**< Interrupt mask */
} dma_handshake_config_fixed_t;

typedef struct dmaChannelDescriptor_s {
    DMA_Type*                   dma;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    uint32_t                   irqN;
    uint32_t                    userParam;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
    uint32_t                    rcc;
    uint32_t                    int_stat;
    AP_HAL::BinarySemaphore     *bin_sem;
    uint32_t                    *rx_buffer;
} dmaChannelDescriptor_t;
class SPIBus : public DeviceBus
{
public:
    SPIBus(uint8_t _bus);
    ~SPIBus();
    uint8_t bus;
    uint32_t *dma_rx_buffer;
    uint32_t *dma_tx_buffer;
};

class SPIDevice : public AP_HAL::SPIDevice
{
public:
    SPIDevice(SPIBus &_bus, SPIDeviceDesc &_device_desc);
    virtual ~SPIDevice();

    bool set_speed(AP_HAL::Device::Speed speed) override;
    bool transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len) override;
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) override;
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::BinarySemaphore *get_binsemaphore();
    AP_HAL::Device::PeriodicHandle register_periodic_callback( uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

private:
    SPIBus &bus;
    SPIDeviceDesc &device_desc;
    Speed speed;
    char *pname;
    void acquire_bus(bool accuire);
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    friend class SPIDevice;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name) override;

private:
    SPIBus *buses;
};
}

