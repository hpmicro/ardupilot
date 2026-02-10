/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/RCOutput.h>
#include "HAL_HPM_Namespace.h"
#include "AP_HAL_HPM.h"
#include <AP_HAL/Util.h>
#include "hpm_gpio_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_soc.h"
#include "hpm_clock_drv.h"

struct outpin_info {
    uint32_t pin_index;
    uint32_t pwm_port_index;
};
typedef uint64_t rcout_timer_t;
#define rcout_micros() AP_HAL::micros64()
class HPMicro::RCOutput : public AP_HAL::RCOutput {
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     cork(void) override {}
    void     push(void) override {}
    void set_output_mode(uint32_t mask, const enum output_mode mode) override;
    // trigger group pulses
    void trigger_groups(void);
    /*
      timer push (for oneshot min rate)
     */
    void timer_tick(rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us);
    // last time pulse was triggererd used to prevent overlap
    rcout_timer_t last_pulse_trigger_us;
    // widest pulse for oneshot triggering
    uint16_t trigger_widest_pulse;
private:
    struct pwm_group {
        uint8_t mcpwm_group_id;
        PWM_Type *base;
        clock_name_t clock;
        // SDK objects for the group

        uint32_t rc_frequency; // frequency in Hz
        uint32_t ch_mask; // mask of channels in this group
        enum output_mode current_mode; // output mode (none, normal, brushed)
        pwm_config_t config;
        uint32_t reload;
    };

    struct pwm_chan {
        // SDK objects for the channel
        pwm_group *group; // associated group
        pwm_cmp_config_t cmp_config;
        uint32_t cmp_idx;
        uint32_t gpio_num;
        uint32_t port_num; // associated GPIO number (always defined)
        int value; // output value in microseconds
    };
    void set_group_mode(pwm_group &group);
    uint16_t value[16];
    bool _initialized;
    static pwm_group pwm_group_list[];
    static pwm_chan pwm_chan_list[];
};
