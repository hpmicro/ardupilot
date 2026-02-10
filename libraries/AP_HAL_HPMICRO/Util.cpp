/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Util.h"
#include "RCOutput.h"
#include <AP_ROMFS/AP_ROMFS.h>
#include "sdcard.h"
#include <stdlib.h>
#include <string.h>
#include <AP_Common/ExpandingString.h>
#include <FreeRTOS.h>
#include "tinyalloc.h"
#include "hpm_ppor_drv.h"
#include "hpm_rtc_drv.h"

static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

using namespace HPMicro;

extern "C" {
    extern uint32_t otp_read_from_shadow(uint32_t addr);
}
void Util::malloc_type_init(uint8_t *buffer, size_t size)
{
    bool ok  = ta_init(buffer, buffer + size, 256, 16, 8);
    if (!ok) {
        hpmhal.console->printf("tinyalloc init failed\n");
        while(1);
    }
}

/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    return configTOTAL_HEAP_SIZE;
}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    void *p = nullptr;
    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        p = ta_alloc(size);
        if (p == nullptr) {
            hpmhal.console->printf("Util: ta_alloc failed for %u bytes\n", (unsigned)size);
        }
        return p;
    }

    // FAST = normal/freeRTOS heap allocation
    if (mem_type == AP_HAL::Util::MEM_FAST) {
        p = pvPortMalloc(size);
        if (p == nullptr) {
            hpmhal.console->printf("Util: pvPortMalloc failed for %u bytes\n", (unsigned)size);
        }
        return p;
    }

    return (void *)NULL;
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr == nullptr) {
        return;
    }

    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        ta_free(ptr);
        return;
    }

    // FAST = normal/freeRTOS heap free
    if (mem_type == AP_HAL::Util::MEM_FAST) {
        vPortFree(ptr);
        return;
    }
}


/*
  get safety switch state
 */
Util::safety_state Util::safety_switch_state(void)
{
    return (AP_HAL::Util::safety_state)0;
}

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    // Convert microseconds to seconds since epoch
    time_t time_seconds = time_utc_usec / 1000000;
    // Configure RTC with the time in seconds
    rtc_config_time(HPM_RTC, time_seconds);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    // Get time in seconds since epoch
    time_t time_seconds = rtc_get_time(HPM_RTC);
    // Convert to microseconds
    return static_cast<uint64_t>(time_seconds) * 1000000;
}

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)

#if !HAL_GCS_ENABLED
#define Debug(fmt, args ...)  do { hpmhal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifier - board type and serial number
 */

bool Util::get_system_id(char buf[50])
{
    char board_name[] = "HPMicro ";
    uint8_t mac_addr[6] = {0};
    
    // Get MAC address from OTP memory
    uint32_t macl = otp_read_from_shadow((uint32_t)OTP_SOC_MAC0_IDX);
    uint32_t mach = otp_read_from_shadow((uint32_t)(OTP_SOC_MAC0_IDX + 1));
    
    mac_addr[0] = (macl >>  0) & 0xff;
    mac_addr[1] = (macl >>  8) & 0xff;
    mac_addr[2] = (macl >> 16) & 0xff;
    mac_addr[3] = (macl >> 24) & 0xff;
    mac_addr[4] = (mach >>  0) & 0xff;
    mac_addr[5] = (mach >>  8) & 0xff;
    
    // Format MAC address as string
    char mac_str[19] = {0};
    snprintf(mac_str, sizeof(mac_str), "%02x %02x %02x %02x %02x %02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    
    // Combine board name and MAC address
    snprintf(buf, 50, "%s%s", board_name, mac_str);
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    uint8_t mac_addr[6] = {0};
    
    // Get MAC address from OTP memory
    uint32_t macl = otp_read_from_shadow((uint32_t)OTP_SOC_MAC0_IDX);
    uint32_t mach = otp_read_from_shadow((uint32_t)(OTP_SOC_MAC0_IDX + 1));

    mac_addr[0] = (macl >>  0) & 0xff;
    mac_addr[1] = (macl >>  8) & 0xff;
    mac_addr[2] = (macl >> 16) & 0xff;
    mac_addr[3] = (macl >> 24) & 0xff;
    mac_addr[4] = (mach >>  0) & 0xff;
    mac_addr[5] = (mach >>  8) & 0xff;
    
    // Copy MAC address to buffer, respecting buffer size
    len = MIN(len, sizeof(mac_addr));
    memcpy(buf, mac_addr, len);
    
    return true;
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    if ((ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog0) || 
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog1) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog2) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_wdog3) ||
        (ppor_reset_get_flags(HPM_PPOR) & ppor_reset_pmic_wdog)) {
        return true;
    }
    return false;
}

/*
  display stack usage as text buffer for @SYS/threads.txt
 */
void Util::thread_info(ExpandingString &str)
{
    char buffer[1024];
    vTaskGetRunTimeStats(buffer);
    str.printf("\n\n%s\n", buffer);
}