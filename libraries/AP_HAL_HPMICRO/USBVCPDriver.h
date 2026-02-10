/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_HPMICRO/AP_HAL_HPM.h>
#include <AP_HAL_HPMICRO/Semaphores.h>

#include "FreeRTOS.h"
#include "task.h"
#include "hpm_soc.h"
#include "usb_osal.h"
#include <stdio.h>
#include "board.h"
#include "usb_config.h"
#include "usb_osal.h"
namespace HPMicro
{

class USBVCPDriver : public AP_HAL::UARTDriver
{
public:

    USBVCPDriver(uint8_t serial_num);

    virtual ~USBVCPDriver() = default;
    static USBVCPDriver *get_singleton()
    {
        return _singleton;
    }

    void vprintf(const char *fmt, va_list ap) override;

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    void _timer_tick(void) override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 10*1024*1024; // USB has higher bandwidth than UART
    }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.
      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.
      A return value of zero means the HAL does not support this API */
     
    uint64_t receive_time_constraint_us(uint16_t nbytes) override; 

    uint32_t get_baud_rate() const override { return _baudrate; }

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    BinarySemaphore _write_buffer_output_mutex;
    Semaphore _write_mutex;
    uint8_t _buffer_avalable;
    uint8_t _buffer[64];
    volatile bool _transmitting; // 标记是否有数据正在传输
    static USBVCPDriver *_singleton;
private:
    bool _initialized;
    const size_t TX_BUF_SIZE = 4096; // Larger buffer for USB
    const size_t RX_BUF_SIZE = 4096; // Larger buffer for USB
    
    uint8_t _serial_num;

    // timestamp for receiving data on the USB VCP, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;
    uint32_t _baudrate;
    const tskTaskControlBlock* _usb_owner_thd;

    void _receive_timestamp_update(void);
    void read_data();
    void write_data();
protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override; // discard all bytes available for reading
};

}
