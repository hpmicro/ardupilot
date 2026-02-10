/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Namespace.h"
#include "AP_HAL_HPM.h"

class HPMicro::Util : public AP_HAL::Util
{
public:
    static Util *from(AP_HAL::Util *util)
    {
        return static_cast<Util*>(util);
    }

    uint32_t available_memory() override;

    // Special Allocation Routines
    void *malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type) override;
    void free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type) override;

    /*
      return state of safety switch, if applicable
     */
    enum safety_state safety_switch_state(void) override;

    // get system ID as a string
    bool get_system_id(char buf[50]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;

#ifdef HAL_PWM_ALARM
    bool toneAlarm_init() override;
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override;
#endif

    // return true if the reason for the reboot was a watchdog reset
    bool was_watchdog_reset() const override;

    // request information on running threads
    void thread_info(ExpandingString &str) override;
    void malloc_type_init(uint8_t *buffer, size_t size);
private:
#ifdef HAL_PWM_ALARM
    struct ToneAlarmPwmGroup {
        pwmchannel_t chan;
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };

    static ToneAlarmPwmGroup _toneAlarm_pwm_group;
#endif

    /*
      set HW RTC in UTC microseconds
     */
    void set_hw_rtc(uint64_t time_utc_usec) override;

    /*
      get system clock in UTC microseconds
     */
    uint64_t get_hw_rtc() const override;
#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    FlashBootloader flash_bootloader() override;
#endif

    // stm32F4 and F7 have 20 total RTC backup registers. We use the first one for boot type
    // flags, so 19 available for persistent data
    static_assert(sizeof(persistent_data) <= 19*4, "watchdog persistent data too large");
};

#ifdef __cplusplus
extern "C" {
#endif

/*
  return true if a memory region is safe for a DMA operation
*/
bool mem_is_dma_safe(const void *addr, uint32_t size, bool filesystem_op);

#ifdef __cplusplus
}
#endif
