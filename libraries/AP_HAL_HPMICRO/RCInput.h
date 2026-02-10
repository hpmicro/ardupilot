/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include "AP_HAL_HPM.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif
class HPMicro::RCInput : public AP_HAL::RCInput {
public:
    RCInput() { _init = false;}
    void init() override;
    void teardown() override {}
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    void _timer_tick(void);
    const char *protocol() const override
    {
        return last_protocol;
    }
    int16_t get_rssi(void) override {
        return _rssi;
    }
    int16_t get_rx_link_quality(void) override {
        return _rx_link_quality;
    }

private:
    int16_t _rssi = -1;
    int16_t _rx_link_quality = -1;
    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {0};
    uint64_t _last_read;
    const char *last_protocol;
    Semaphore rcin_mutex;
    uint8_t _num_channels;
    uint32_t _rcin_timestamp_last_signal;
    bool _init;
};
