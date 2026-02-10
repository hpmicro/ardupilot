/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "board.h"
#include "hpm_ioc_regs.h"

using namespace HPMicro;
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1000ns per tick, 2x80 prescaler
#define BRUSH_TIMEBASE_RESOLUTION_HZ 40000000 // 40MHz, 25ns per tick, 2x2 prescaler
#define SERVO_DEFAULT_FREQ_HZ 50 // the rest of ArduPilot assumes this!
static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

#ifdef HAL_HPM_RCOUT

struct outpin_info outputs_pins[] = {HAL_HPM_RCOUT};

//If the RTC source is not required, then GPIO32/Pin12/32K_XP and GPIO33/Pin13/32K_XN can be used as digital GPIOs.

#else
struct outpin_info outputs_pins[] = {};

#endif
#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)
static_assert(MAX_CHANNELS < 12, "overrunning _pending and safe_pwm"); // max for current chips
static_assert(MAX_CHANNELS < 32, "overrunning bitfields");

RCOutput::pwm_group RCOutput::pwm_group_list[] = HAL_PWM_GROUPS;
RCOutput::pwm_chan RCOutput::pwm_chan_list[MAX_CHANNELS];
void RCOutput::init()
{
    for (int idx = 0; idx < ARRAY_SIZE(pwm_group_list); idx++) {
        pwm_group &group = pwm_group_list[idx];
        group.mcpwm_group_id = idx;
        pwm_stop_counter(group.base);
 
        memset(&group.config, 0, sizeof(group.config));
        pwm_get_default_pwm_config(group.base, &group.config);
        group.config.enable_output = true;
        group.config.dead_zone_in_half_cycle = 0;
        group.config.invert_output = false;
        group.rc_frequency = 50;
        uint32_t reload = clock_get_frequency(group.clock) / group.rc_frequency - 1;
        group.reload = reload;
        pwm_set_reload(group.base, 0, reload);
        pwm_set_start_count(group.base, 0, 0);
        // enable all channels in the group
        for (int ch = 0; ch < 3; ch++) {
            pwm_chan &channel = pwm_chan_list[idx * 3 + ch];
            channel.group = &group;
            channel.gpio_num = outputs_pins[idx * 3 + ch].pin_index;
            channel.port_num = outputs_pins[idx * 3 + ch].pwm_port_index;
            HPM_IOC->PAD[channel.gpio_num].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(16);
            memset(&channel.cmp_config, 0, sizeof(channel.cmp_config));
            channel.cmp_config.mode = pwm_cmp_mode_output_compare;
            channel.cmp_config.cmp = reload + 1;
            channel.cmp_config.update_trigger = pwm_shadow_register_update_on_modify;
            channel.cmp_idx = idx * 3 + ch + 1;

            /*
            * config pwm as output driven by cmp
            */
            if (status_success != pwm_setup_waveform(group.base, channel.port_num, &group.config, channel.cmp_idx, &channel.cmp_config, 1)) {
                hpmhal.console->printf("failed to setup waveform\n");
                while(1) {
                    hpmhal.scheduler->delay_microseconds(1000);
                }
            }
        }
        pwm_start_counter(group.base);
        pwm_issue_shadow_register_lock_event(group.base);
    }
    _initialized = true;
}
/*
  setup output mode for a group, using group.current_mode.
 */
void RCOutput::set_group_mode(pwm_group &group)
{
    if (!_initialized) {
        return;
    }
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
        break;

    default:
        group.current_mode = MODE_PWM_NONE; // treat as 0 output normal
    // fallthrough
    case MODE_PWM_NONE:
    case MODE_PWM_NORMAL:
        break;

    case MODE_PWM_ONESHOT:
    case MODE_PWM_ONESHOT125:
        //TODO
        break;
    }
}

void RCOutput::set_output_mode(uint32_t mask, const enum output_mode mode)
{
    while (mask) {
        uint8_t chan = __builtin_ffs(mask)-1;
        if (!_initialized || chan >= MAX_CHANNELS) {
            return;
        }

        pwm_group &group = *pwm_chan_list[chan].group;
        group.current_mode = mode;
        set_group_mode(group);

        // acknowledge the setting of any channels sharing this group
        for (chan=0; chan<MAX_CHANNELS; chan++) {
            if (pwm_chan_list[chan].group == &group) {
                mask &= ~(1U << chan);
            }
        }
    }
}

/*
  trigger output groups for oneshot or dshot modes
 */
void RCOutput::trigger_groups()
{
    rcout_timer_t now = rcout_micros();

    if (!AP_HAL::timeout_expired(last_pulse_trigger_us, now, trigger_widest_pulse)) {
        // guarantee minimum pulse separation
        hpmhal.scheduler->delay_microseconds(AP_HAL::timeout_remaining(last_pulse_trigger_us, now, trigger_widest_pulse));
    }

    for (auto &group : pwm_group_list) {

        if (group.current_mode == MODE_PWM_ONESHOT ||
            group.current_mode == MODE_PWM_ONESHOT125) {
        }
    }

    /*
      calculate time that we are allowed to trigger next pulse
      to guarantee at least a 50us gap between pulses
    */
    last_pulse_trigger_us = rcout_micros();

}
/*
  periodic timer. This is used for oneshot and dshot modes, plus for
  safety switch update. Runs every 1000us.
 */
void RCOutput::timer_tick(rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us)
{
    if (last_pulse_trigger_us == 0) {
        return;
    }

    if (AP_HAL::timeout_expired(last_pulse_trigger_us, rcout_micros(), trigger_widest_pulse + 4000U)) {
        // trigger at a minimum of 250Hz
        trigger_groups();
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    for (auto &group : pwm_group_list) {
        if ((group.ch_mask & chmask) != 0) { // group has channels to set?
            group.rc_frequency = freq_hz; // set frequency and corresponding period
            uint32_t reload = clock_get_frequency(group.clock) / freq_hz;
            group.reload = reload;
            pwm_stop_counter(group.base);
            pwm_set_reload(group.base, 0, reload);
            pwm_set_start_count(group.base, 0, 0);
            pwm_start_counter(group.base);
            pwm_issue_shadow_register_lock_event(group.base);
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return SERVO_DEFAULT_FREQ_HZ;
    }

    pwm_group &group = *pwm_chan_list[chan].group;
    return group.rc_frequency;
}

void RCOutput::enable_ch(uint8_t chan)
{}

void RCOutput::disable_ch(uint8_t chan)
{}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan < ARRAY_SIZE(value)) {
        value[chan] = period_us;
    }

    pwm_group &group = *pwm_chan_list[chan].group;
    if ((int)&group == 0)
        return;
    pwm_chan &ch = pwm_chan_list[chan];
    float duty = 0;
    switch(group.current_mode) {
    case MODE_PWM_BRUSHED: {
        if (period_us <= _esc_pwm_min) {
            duty = 0;
        } else if (period_us >= _esc_pwm_max) {
            duty = 1;
        } else {
            duty = ((float)(period_us - _esc_pwm_min))/(_esc_pwm_max - _esc_pwm_min);
        }
        break;
    }
    case MODE_PWM_NORMAL:
        duty = 100.0f * period_us / (1000000 / group.rc_frequency);
        break;
    case MODE_PWM_NONE:
        duty = 100.0f * period_us / (1000000 / group.rc_frequency);
        break;
    default:
        break;
    }
    pwm_update_duty_edge_aligned(group.base, ch.cmp_idx, duty);
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= MAX_CHANNELS || !_initialized) {
        return 0;
    }

    pwm_chan &ch = pwm_chan_list[chan];
    return ch.value;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, MAX_CHANNELS); i++) {
        period_us[i] = read(i);
    }
}

