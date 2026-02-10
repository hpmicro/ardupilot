/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once

#include "AP_HAL_HPM.h"
#include "hpm_adc12_drv.h"
#include "hpm_adc16_drv.h"
#include "hpm_soc.h"

#define ANALOG_MAX_CHANNELS 8

namespace HPMicro
{

struct ADCBusDesc {
    bool is_adc16;
    union {
        ADC16_Type *base16;
        ADC12_Type *base12;
    } host;
    clock_name_t clk;
};
class AnalogSource : public AP_HAL::AnalogSource
{
public:
    friend class AnalogIn;
    AnalogSource(int16_t ardupin, int adc_channel, float scaler, float initial_value);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}

private:
    //ADC number (1 or 2). ADC2 is unavailable when WIFI on
    union {
        ADC16_Type *_adc_unit16;
        ADC12_Type *_adc_unit12;
    } _adc_unit;
    bool _is_adc16;
    clock_name_t _host_clk;

    //ADC channel
    int _adc_channel;

    //human readable Pin number used in ardu params
    int16_t _ardupin;
    //scaling from ADC count to Volts
    float _scaler;
    int _adc_cali_handle;

    //Current computed value (average)
    float _value;
    //Latest fetched raw value from the sensor
    float _latest_value;
    //Number of fetched value since average
    uint8_t _sum_count;
    //Sum of fetched values
    float _sum_value;
    uint32_t _pin;

    bool adc_init();
    float adc_read();
    void _add_value();

    HAL_Semaphore _semaphore;
};

class AnalogIn : public AP_HAL::AnalogIn
{
public:
    friend class AnalogSource;

    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    void _timer_tick();
    float board_voltage() override
    {
        return _board_voltage;
    }
    static uint32_t find_pinconfig(uint32_t ardupin);

private:
    HPMicro::AnalogSource* _channels[ANALOG_MAX_CHANNELS]; // list of pointers to active individual AnalogSource objects or nullptr

    uint32_t _last_run;
    float _board_voltage;

    struct pin_info {
        uint8_t channel;  // adc1 pin offset
        float scaling;
        uint32_t ardupin; // eg 3 , as typed into an ardupilot parameter
        uint32_t pin;
        bool is_adc16;
        union {
            ADC16_Type *base16;
            ADC12_Type *base12;
            uint32_t base;
        } host;
        clock_name_t clk;
    };

    static const pin_info pin_config[];
};

}
