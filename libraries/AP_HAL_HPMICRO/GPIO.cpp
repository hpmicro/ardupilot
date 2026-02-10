/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "AP_HAL_HPM.h"
#include "GPIO.h"
#include "hpm_gpio_drv.h"
#include "hpm_soc.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hpm_interrupt.h"
#include "hpm_soc_irq.h"
// #define HPMICRO_GPIO_CLASS_DEBUG   1
using namespace HPMicro;
extern const AP_HAL::HAL &hpmhal;
// GPIO pin table from hwdef.dat
struct gpio_entry {
    uint32_t pin_num;
    uint32_t ioc_idx;
    bool enabled;
    uint8_t pwm_num;
    uint32_t pal_line;
    AP_HAL::GPIO::irq_handler_fn_t fn; // callback for GPIO interface
    tskTaskControlBlock *thd_wait;
    bool is_input;
    uint8_t mode;
    uint16_t isr_quota;
    uint8_t isr_disabled_ticks;
    AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE isr_mode;
};

typedef enum {
    HPM_GPIOA = 0,
    HPM_GPIOB,
    HPM_GPIOC,
    HPM_GPIOD,
    HPM_GPIOE,
    HPM_GPIOF,
    HPM_GPIO_RSV0,
    HPM_GPIO_RSV1,
    HPM_GPIO_RSV2,
    HPM_GPIO_RSV3,
    HPM_GPIO_RSV4,
    HPM_GPIO_RSV5,
    HPM_GPIO_RSV6,
    HPM_GPIOX = 13,
    HPM_GPIOY,
    HPM_GPIOZ,
}gpioPortIdx_t;
static const uint8_t extiGroupIRQn[16] = {
    IRQn_GPIO0_A,  //0
    IRQn_GPIO0_B,  //1
    IRQn_GPIO0_C,  //2
    IRQn_GPIO0_D,  //3
    IRQn_GPIO0_E,  //4
    IRQn_GPIO0_F,  //5
    255,  //6
    255,  //7
    255,  //8
    255,  //9
    255,  //10
    255,  //11
    255,  //12
    IRQn_GPIO0_X, //13
    IRQn_GPIO0_Y, //14
    IRQn_GPIO0_Z, //15
};
typedef struct {
    palcallback_t handler[32];
    void *param[32];
} extiChannelRec_t;
extiChannelRec_t extiChannelRecs[16] = {0};
#ifdef HAL_GPIO_PINS
#define HAVE_GPIO_PINS 1
static struct gpio_entry _gpio_tab[] = {HAL_GPIO_PINS};
#else
#define HAVE_GPIO_PINS 0
#endif


/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        const auto &t = _gpio_tab[i];
        if (pin_num == t.pin_num) {
            if (check_enabled && t.pwm_num != 0 && !t.enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
#endif
    return NULL;
}
static void pal_interrupt_cb_functor(void *arg)
{
    const uint32_t now = AP_HAL::micros();

    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr) {
        // what?
        return;
    }
    if (!(g->fn)) {
        return;
    }

    (g->fn)(g->pin_num, 0/*TODO*/, now);
}
GPIO::GPIO()
{
#ifdef HPM_BIOC
    has_bioc = true;
#endif
#ifdef HPM_PIOC
    has_pioc = true;
#endif
}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return;
    }
    uint32_t ioc_idx = g->ioc_idx;
    switch (output) {
    case HAL_GPIO_INPUT:
        _is_input[ioc_idx] = true;
        HPM_IOC->PAD[ioc_idx].FUNC_CTL = 0; //Set pin to gpio
        if (((uint32_t)ioc_idx >= IOC_PAD_PZ00) && ((uint32_t)ioc_idx <= IOC_PAD_PZ11)) {
            HPM_BIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        if (((uint32_t)ioc_idx >= IOC_PAD_PY00) && ((uint32_t)ioc_idx <= IOC_PAD_PY11)) {
            HPM_PIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        gpio_set_pin_input(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    break;
    case HAL_GPIO_OUTPUT:
        _is_input[ioc_idx] = false;
        HPM_IOC->PAD[ioc_idx].FUNC_CTL = 0; //Set pin to gpio
        if (((uint32_t)ioc_idx >= IOC_PAD_PZ00) && ((uint32_t)ioc_idx <= IOC_PAD_PZ11)) {
            HPM_BIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        if (((uint32_t)ioc_idx >= IOC_PAD_PY00) && ((uint32_t)ioc_idx <= IOC_PAD_PY11)) {
            HPM_PIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    break;
    case HAL_GPIO_ALT:
        hpmhal.console->printf("error pinmode, please use pinMode(pin, output, alt) instead\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    default:
        hpmhal.console->printf("error pinmode\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    }
    g->enabled = true;
}

void GPIO::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return;
    }
    uint32_t ioc_idx = g->ioc_idx;
    switch (output) {
    case HAL_GPIO_INPUT:
    case HAL_GPIO_OUTPUT:
        hpmhal.console->printf("error pinmode, please use pinMode(pin, output) instead\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    case HAL_GPIO_ALT:
        HPM_IOC->PAD[ioc_idx].FUNC_CTL = alt;
    break;
    default:
        hpmhal.console->printf("error pinmode\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    }
    g->enabled = true;
}
uint8_t GPIO::read(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return 0;
    }
    uint32_t ioc_idx = g->ioc_idx;
#if HPMICRO_GPIO_CLASS_DEBUG
    static uint8_t last_level[512];
    uint8_t current = (uint8_t)gpio_read_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    if (current != last_level[ioc_idx]) {
        hpmhal.console->printf("%s:%d %d %d\n", __PRETTY_FUNCTION__, __LINE__, (int)ioc_idx, (int)current);
        last_level[ioc_idx] = current;
    }
#endif
    return gpio_read_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return;
    }
    uint32_t ioc_idx = g->ioc_idx;
    if (_is_input[ioc_idx]) {
        HPM_IOC->PAD[ioc_idx].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(value != 0 ? 1 : 0); //Set pin to gpio
    }
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx), value);
}

void GPIO::toggle(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return;
    }
    uint32_t ioc_idx = g->ioc_idx;
    gpio_toggle_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin) {
    return (AP_HAL::DigitalSource*)nullptr;
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(_pin, false);
    if (!g) {
        return;
    }
    uint32_t ioc_idx = g->ioc_idx;
    switch (output) {
    case HAL_GPIO_INPUT:
        HPM_IOC->PAD[ioc_idx].FUNC_CTL = 0; //Set pin to gpio
        if (((uint32_t)ioc_idx >= IOC_PAD_PZ00) && ((uint32_t)ioc_idx <= IOC_PAD_PZ11)) {
            HPM_BIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        if (((uint32_t)ioc_idx >= IOC_PAD_PY00) && ((uint32_t)ioc_idx <= IOC_PAD_PY11)) {
            HPM_PIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        gpio_set_pin_input(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    break;
    case HAL_GPIO_OUTPUT:
        HPM_IOC->PAD[ioc_idx].FUNC_CTL = 0; //Set pin to gpio
        if (((uint32_t)ioc_idx >= IOC_PAD_PZ00) && ((uint32_t)ioc_idx <= IOC_PAD_PZ11)) {
            HPM_BIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        if (((uint32_t)ioc_idx >= IOC_PAD_PY00) && ((uint32_t)ioc_idx <= IOC_PAD_PY11)) {
            HPM_PIOC->PAD[ioc_idx].FUNC_CTL = 3;
        }
        gpio_set_pin_output(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    break;
    case HAL_GPIO_ALT:
        hpmhal.console->printf("error pinmode, please use pinMode(pin, output, alt) instead\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    default:
        hpmhal.console->printf("error pinmode\r\n");
        while(1) {
            hpmhal.scheduler->delay(1000);
        }
    break;
    }
}

uint8_t DigitalSource::read() {
    return gpio_read_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(_pin), GPIO_GET_PIN_INDEX(_pin));
}

void DigitalSource::write(uint8_t value) {
    gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(_pin), GPIO_GET_PIN_INDEX(_pin), value);
}

void DigitalSource::toggle() {
    gpio_toggle_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(_pin), GPIO_GET_PIN_INDEX(_pin));
}

/*
   Attach an interrupt handler to a GPIO pin number. The pin number
   must be one specified with a GPIO() marker in hwdef.dat
 */
bool GPIO::attach_interrupt(uint8_t pin,
                            irq_handler_fn_t fn,
                            INTERRUPT_TRIGGER_TYPE mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    g->isr_disabled_ticks = 0;
    g->isr_quota = 0;
    g->fn = fn;
    if (!_attach_interrupt(pin,
                           palcallback_t(fn?pal_interrupt_cb_functor:nullptr),
                           g,
                           mode)) {
        return false;
    }
    return true;
}
bool GPIO::attach_interrupt(uint8_t pin,
                            AP_HAL::Proc proc,
                            INTERRUPT_TRIGGER_TYPE mode) {
    return _attach_interrupt(pin, palcallback_t(proc?pal_interrupt_cb_functor:nullptr), (void *)proc, mode);
}
bool GPIO::_attach_interrupt(uint8_t pin, palcallback_t cb, void *p, uint8_t mode)
{
    gpio_interrupt_trigger_t trigger = gpio_interrupt_trigger_edge_falling;
    switch(mode) {
        case INTERRUPT_FALLING:
            trigger = gpio_interrupt_trigger_edge_falling;
            break;
        case INTERRUPT_RISING:
            trigger = gpio_interrupt_trigger_edge_rising;
            break;
        case INTERRUPT_BOTH:
#if defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) && (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT == 1)
            trigger = gpio_interrupt_trigger_edge_both;
#else
            hpmhal.console->printf("The SOC does not support both edge interrupt!!");
#endif
            break;
        default:
            if (p) {
                return false;
            }
            break;
    }
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    uint32_t ioc_idx = g->ioc_idx;
    uint32_t irq_num = 0;
    irq_num = extiGroupIRQn[GPIO_GET_PORT_INDEX(ioc_idx)];

    extiChannelRecs[GPIO_GET_PORT_INDEX(ioc_idx)].handler[GPIO_GET_PIN_INDEX(ioc_idx)] = cb;
    extiChannelRecs[GPIO_GET_PORT_INDEX(ioc_idx)].param[GPIO_GET_PIN_INDEX(ioc_idx)] = p;

    portENTER_CRITICAL();
    gpio_config_pin_interrupt(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx), trigger);
    gpio_enable_pin_interrupt(HPM_GPIO0, GPIO_GET_PORT_INDEX(ioc_idx), GPIO_GET_PIN_INDEX(ioc_idx));
    intc_m_enable_irq(irq_num);
    portEXIT_CRITICAL();
    return true;
}


__attribute__((section(".fast"))) static void gpio_interrupt_handler(GPIO_Type *base, unsigned char port_index, unsigned char group)
{
    uint32_t flag = gpio_get_port_interrupt_flags(base, port_index);
    for (int i = 0; i < 32; i++) {
        if (flag & (1 << i)) {
            gpio_clear_pin_interrupt_flag(base, port_index, i);
            if (extiChannelRecs[group].handler[i] != NULL) {
                extiChannelRecs[group].handler[i](extiChannelRecs[group].param[i]);
            }
        }
    }
}

#ifdef IRQn_GPIO0_A
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_A, gpio_porta_isr)
void gpio_porta_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOA, HPM_GPIOA);
}
#endif

#ifdef IRQn_GPIO0_B
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B , gpio_portb_isr)
void gpio_portb_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOB, HPM_GPIOB);
}
#endif

#ifdef IRQn_GPIO0_C
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_C , gpio_portc_isr)
void gpio_portc_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOC, HPM_GPIOC);
}
#endif

#ifdef IRQn_GPIO0_D
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_D , gpio_portd_isr)
void gpio_portd_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOD, HPM_GPIOD);
}
#endif

#ifdef IRQn_GPIO0_E
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_E , gpio_porte_isr)
void gpio_porte_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOE, HPM_GPIOE);
}
#endif

#ifdef IRQn_GPIO0_F
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_F , gpio_portf_isr)
void gpio_portf_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOF, HPM_GPIOF);
}
#endif

#ifdef IRQn_GPIO0_X
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_X, gpio_portx_isr)
void gpio_portx_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOX, HPM_GPIOX);
}
#endif

#ifdef IRQn_GPIO0_Y
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Y, gpio_porty_isr)
void gpio_porty_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOY, HPM_GPIOY);
}
#endif

#ifdef IRQn_GPIO0_Z
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_Z, gpio_portz_isr)
void gpio_portz_isr(void)
{
    gpio_interrupt_handler(HPM_GPIO0, GPIO_DI_GPIOZ, HPM_GPIOZ);
}
#endif