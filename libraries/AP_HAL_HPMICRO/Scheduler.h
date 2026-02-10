/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Namespace.h"
#include "FreeRTOS.h"
#include "task.h"

#define HPM_SCHEDULER_MAX_TIMER_PROCS 10
#define HPM_SCHEDULER_MAX_IO_PROCS 10

/* Scheduler implementation: */
class HPMicro::Scheduler : public AP_HAL::Scheduler
{

public:
    Scheduler();
    ~Scheduler() {};
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb) { callbacks = cb; };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    // check and set the startup state
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    void     print_stats(void) ;
    void     print_main_loop_rate(void);
    uint16_t get_loop_rate_hz(void);
    AP_Int16 _active_loop_rate_hz;
    AP_Int16 _loop_rate_hz;

    static void thread_create_trampoline(void *ctx);
    static void osalSysLock(void);
    static void osalSysUnlock(void);
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    static const int SPI_PRIORITY = 24; //      if your primary imu is spi, this should be above the i2c value, spi is better.
    static const int MAIN_PRIO    = 24; //	cpu0: we want scheduler running at full tilt.
    static const int I2C_PRIORITY = 5;  //      if your primary imu is i2c, this should be above the spi value, i2c is not preferred.
    static const int TIMER_PRIO   = 23; //dont make 24. a low priority mere might cause wifi thruput to suffer
    static const int RCIN_PRIO    = 5;
    static const int RCOUT_PRIO   = 10;
    static const int WIFI_PRIO1   = 20; //cpu1:
    static const int WIFI_PRIO2   = 12; //cpu1:
    static const int UART_PRIO    = 23; //dont make 24, scheduler suffers a bit. cpu1: a low priority mere might cause wifi thruput to suffer, as wifi gets passed its data frim the uart subsustem in _writebuf/_readbuf
    static const int IO_PRIO      = 5;
    static const int STORAGE_PRIO = 4;

    static const int TIMER_SS     = 1024*8;
    static const int MAIN_SS      = 1024*8;
    static const int RCIN_SS      = 1024*8;
    static const int RCOUT_SS     = 1024*8;
    static const int WIFI_SS1     = 1024*8;
    static const int WIFI_SS2     = 1024*8;
    static const int UART_SS      = 1024*8;
    static const int DEVICE_SS    = 1024*8;     // DEVICEBUS/s
    static const int IO_SS        = 1024*8;   // APM_IO
    static const int STORAGE_SS   = 1024*8;     // APM_STORAGE

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[HPM_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[HPM_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;

    tskTaskControlBlock* _main_task_handle;
    tskTaskControlBlock* _timer_task_handle;
    tskTaskControlBlock* _rcin_task_handle;
    tskTaskControlBlock* _rcout_task_handle;
    tskTaskControlBlock* _uart_task_handle;
    tskTaskControlBlock* _io_task_handle;
    tskTaskControlBlock* test_task_handle;
    tskTaskControlBlock* _storage_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;
};
