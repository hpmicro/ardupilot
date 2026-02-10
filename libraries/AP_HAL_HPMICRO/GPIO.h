/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include "AP_HAL_HPM.h"
typedef void (*palcallback_t)(void *arg);
class HPMicro::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;
    /* Interrupt interface - fast, for RCOutput and SPI radios */
    bool    attach_interrupt(uint8_t interrupt_num,
                             AP_HAL::Proc p,
                             INTERRUPT_TRIGGER_TYPE mode) override;

    /* Interrupt interface - for AP_HAL::GPIO */
    bool    attach_interrupt(uint8_t pin,
                             irq_handler_fn_t fn,
                             INTERRUPT_TRIGGER_TYPE mode) override;
private:
    bool _attach_interrupt(uint8_t pin, palcallback_t cb, void *p, uint8_t mode);
#ifdef HPM_BIOC
    bool    has_bioc;
#endif
#ifdef HPM_BIOC
    bool    has_pioc;
#endif
    bool _is_input[512];
};

class HPMicro::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t pin);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;
private:
    uint32_t _pin;
    uint32_t _irq;
};
