/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_HPMICRO/HAL_HPM_Class.h>
#include <AP_HAL_HPMICRO/Scheduler.h>
#include <AP_Math/div1000.h>
#include "sdcard.h"
#include <stdint.h>
#include "hpm_mchtmr_drv.h"
#include "hpm_soc.h"

HAL_HPM hal_hpm;
const AP_HAL::HAL &hpmhal = AP_HAL::get_HAL();
namespace AP_HAL
{

void panic(const char *errormsg, ...)
{
    hal_hpm.console->printf("PANIC: %s", errormsg);
    hal_hpm.console->flush();
    // Cast the generic Scheduler pointer to HPMicro specific implementation
    HPMicro::Scheduler *hpm_scheduler = static_cast<HPMicro::Scheduler *>(hal_hpm.scheduler);
    
    // Lock the scheduler
    hpm_scheduler->osalSysLock();
    while (1) {}
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    return mchtmr_get_count(HPM_MCHTMR) / 24;
}

uint64_t millis64()
{
    return uint64_div1000(micros64());
}

} // namespace AP_HAL

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_hpm;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_hpm;
}