/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include "DeviceBus.h"
#include "hpm_i2c_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_soc.h"

namespace HPMicro {

struct I2CBusDesc {
    I2C_Type *port;
    uint32_t sda;
    uint32_t scl;
    uint32_t sda_af;
    uint32_t scl_af;
    uint32_t sda_paf;
    uint32_t scl_paf;
    uint32_t sda_baf;
    uint32_t scl_baf;
    clock_name_t bus_clock;
    uint32_t speed;
    bool internal;
    bool soft;
};

class I2CBus : public  DeviceBus
{
public:
    I2CBus():DeviceBus(Scheduler::I2C_PRIORITY) {};
    I2C_Type *port;
    clock_name_t bus_clock;
    bool soft;
};
class I2CDevice : public AP_HAL::I2CDevice {
public:
    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms);

    virtual ~I2CDevice();

    /* AP_HAL::I2CDevice implementation */

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override
    {
        _address = address;
    }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override
    {
        _retries = retries;
    }


    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override
    {
        return false;
    };


    /* See AP_HAL::Device::set_speed() */
    bool set_speed(enum AP_HAL::Device::Speed speed) override
    {
        return true;
    }

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override
    {
        // if asking for invalid bus number use bus 0 semaphore
        return &bus.semaphore;
    }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

protected:
    I2CBus &bus;
    uint8_t _retries;
    uint8_t _address;
    char *pname;
    uint32_t _timeout_ms;
};

class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;
    static I2CBus businfo[];
    I2CDeviceManager();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
            uint32_t bus_clock=400000,
            bool use_smbus = false,
            uint32_t timeout_ms=4) override;
    /*
      get mask of bus numbers for all configured I2C buses
     */
    uint32_t get_bus_mask(void) const override;

    /*
      get mask of bus numbers for all configured external I2C buses
     */
    uint32_t get_bus_mask_external(void) const override;

    /*
      get mask of bus numbers for all configured internal I2C buses
     */
    uint32_t get_bus_mask_internal(void) const override;

};

} // namespace HPMicro