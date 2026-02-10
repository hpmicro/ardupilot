/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "HAL_HPM_Namespace.h"
#include <FreeRTOS.h>
#include <semphr.h>

class HPMicro::Semaphore : public AP_HAL::Semaphore
{
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
    void take_blocking() override;

    bool check_owner();
protected:
    void*  handle;
};

class HPMicro::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state=false);
    ~BinarySemaphore(void);

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;
    void signal_ISR(void) override;

protected:
    SemaphoreHandle_t _sem;
};

