/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_HPMICRO/Semaphores.h>

#include <stdlib.h>
#include <stdio.h>


#include "AnalogIn.h"

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "hpm_ioc_regs.h"
// #define ANALOGIN_DEBUGGING 1

// ADC Configuration Parameters - defined in hwdef.dat
#ifndef HPM_ADC_SAMPLE_CYCLE
#define HPM_ADC_SAMPLE_CYCLE 20
#endif

#ifndef HPM_ADC_PRESCALE_VALUE
#define HPM_ADC_PRESCALE_VALUE 22
#endif

#ifndef HPM_ADC_PERIOD_COUNT
#define HPM_ADC_PERIOD_COUNT 5
#endif

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3300.0f/4096.0f)

#if ANALOGIN_DEBUGGING
static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();
# define Debug(fmt, args ...)  do {hpmhal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL &hpmhal;

using namespace HPMicro;

/*
   scaling table between ADC count and actual input voltage, to account
   for voltage dividers on the board.
   */

/*
   scaling table between ADC count and actual input voltage, to account
   for voltage dividers on the board.
   */
const AnalogIn::pin_info AnalogIn::pin_config[] = {HAL_HPM_ADC_PINS};
#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)
#define DEFAULT_VREF    3300         //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
bool adc_calibration_init(int unit, int channel, int atten, int *out_handle)
{
    return true;
}

void adc_calibration_deinit(int handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    Debug("AnalogIn: deregister %s calibration scheme", "Curve Fitting");
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    Debug("AnalogIn: deregister %s calibration scheme", "Line Fitting");
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

//ardupin is the ardupilot assigned number, starting from 1-8(max)
AnalogSource::AnalogSource(int16_t ardupin, int adc_channel, float scaler, float initial_value) :
    _adc_channel(adc_channel),
    _ardupin(ardupin),
    _scaler(scaler),
    _value(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0)
{
    Debug("AnalogIn: adding ardupin:%d-> which is adc_channel:%d scaler %f\n", _ardupin, _adc_channel, _scaler);

    uint32_t pinconfig_offset = AnalogIn::find_pinconfig(ardupin);
    if (pinconfig_offset == -1 ) {
        Debug("AnalogIn: sorry set_pin() can't determine ADC offset from ardupin : %d \n",ardupin);
        return;
    }
    if (AnalogIn::pin_config[(uint32_t)pinconfig_offset].is_adc16) {
        _adc_unit._adc_unit16 = AnalogIn::pin_config[(uint32_t)pinconfig_offset].host.base16;
        _is_adc16 = true;
    } else {
        _adc_unit._adc_unit12 = AnalogIn::pin_config[(uint32_t)pinconfig_offset].host.base12;
        _is_adc16 = false;
    }
    _host_clk = AnalogIn::pin_config[(uint32_t)pinconfig_offset].clk;
    _pin = AnalogIn::pin_config[(uint32_t)pinconfig_offset].pin;
    float newscaler = AnalogIn::pin_config[(uint32_t)pinconfig_offset].scaling;
    _scaler = newscaler;
    adc_init();
}


float AnalogSource::read_average()
{
    if ( _ardupin == ANALOG_INPUT_NONE ) {
        return 0.0f;
    }

    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        float adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc_read();
        }
        adc_reading /= NO_OF_SAMPLES;
        return adc_reading;
    }

    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;

    return _value;
}

float AnalogSource::read_latest()
{
    return _latest_value;
}

//_scaler scaling from ADC count to Volts

/*
   return voltage in Volts
   */
float AnalogSource::voltage_average()
{
    return _scaler * read_average();
}

/*
   return voltage in Volts
   */
float AnalogSource::voltage_latest()
{
    return _scaler * read_latest();
}

float AnalogSource::voltage_average_ratiometric()
{
    return _scaler * read_latest();
}

// ardupin
bool AnalogSource::set_pin(uint8_t ardupin)
{

    if (_ardupin == ardupin) {
        return true;
    }
    _ardupin = ardupin;

    uint32_t pinconfig_offset = AnalogIn::find_pinconfig(ardupin);
    if (pinconfig_offset == -1 ) {
        Debug("AnalogIn: sorry set_pin() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        return false;
    }

    uint8_t newChannel = (uint8_t)AnalogIn::pin_config[(uint32_t)pinconfig_offset].channel;
    float newscaler = AnalogIn::pin_config[(uint32_t)pinconfig_offset].scaling;
    _pin = AnalogIn::pin_config[(uint32_t)pinconfig_offset].pin;
    if (AnalogIn::pin_config[(uint32_t)pinconfig_offset].is_adc16) {
        _adc_unit._adc_unit16 = AnalogIn::pin_config[(uint32_t)pinconfig_offset].host.base16;
        _is_adc16 = true;
    } else {
        _adc_unit._adc_unit12 = AnalogIn::pin_config[(uint32_t)pinconfig_offset].host.base12;
        _is_adc16 = false;
    }
    _host_clk = AnalogIn::pin_config[(uint32_t)pinconfig_offset].clk;

    Debug("AnalogIn: ardupin = %d, new channel = %d\n", ardupin, newChannel);

    if (_adc_channel == newChannel) {
        return true;
    }

    WITH_SEMAPHORE(_semaphore);  

    _adc_channel = newChannel;
    _scaler = newscaler;

    adc_init();

    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;

    return true;
}

// init ADC
bool AnalogSource::adc_init()
{
    // init the pin now if possible, otherwise do it later from set_pin
    if ( _ardupin != ANALOG_INPUT_NONE ) {
        adc16_config_t cfg;
        clock_set_adc_source(_host_clk, clk_adc_src_ahb0);
        clock_add_to_group(_host_clk, 0);
        HPM_IOC->PAD[_pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
        if (_is_adc16) {
            adc16_get_default_config(&cfg);
            cfg.res            = adc16_res_16_bits;
            cfg.conv_mode      = adc16_conv_mode_period;
            cfg.adc_clk_div    = adc16_clock_divider_4;
            cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == clock_get_source(_host_clk)) ? true : false;

            /* adc16 initialization */
            if (adc16_init(_adc_unit._adc_unit16, &cfg) == status_success) {
            } else {
                hpmhal.console->printf("AnalogSource initialization failed!\n");
                while(1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            adc16_channel_config_t ch_cfg;

            /* get a default channel config */
            adc16_get_channel_default_config(&ch_cfg);

            /* initialize an ADC channel */
            ch_cfg.ch           = _adc_channel;
            ch_cfg.sample_cycle = HPM_ADC_SAMPLE_CYCLE; //TODO

            adc16_init_channel(_adc_unit._adc_unit16, &ch_cfg);

            adc16_prd_config_t prd_cfg;
            prd_cfg.ch           = _adc_channel;
            prd_cfg.prescale     = HPM_ADC_PRESCALE_VALUE;    /* Set divider: 2^22 clocks */
            prd_cfg.period_count = HPM_ADC_PERIOD_COUNT;     /* 6 periods */

            adc16_set_prd_config(_adc_unit._adc_unit16, &prd_cfg);
        } else {
            adc12_config_t cfg;

            /* initialize an ADC instance */
            adc12_get_default_config(&cfg);

            cfg.res            = adc12_res_12_bits;
            cfg.conv_mode      = adc12_conv_mode_period;
            cfg.diff_sel       = adc12_sample_signal_single_ended;
            cfg.adc_clk_div    = adc12_clock_divider_3;
            cfg.sel_sync_ahb   = (clk_adc_src_ahb0 == clock_get_source(_host_clk)) ? true : false;

            /* adc12 initialization */
            if (adc12_init(_adc_unit._adc_unit12, &cfg) == status_success) {
            } else {
                hpmhal.console->printf("AnalogSource initialization failed!\n");
                while(1) {
                    hpmhal.scheduler->delay(1000);
                }
            }
            adc12_channel_config_t ch_cfg;

            /* get a default channel config */
            adc12_get_channel_default_config(&ch_cfg);

            /* initialize an ADC channel */
            ch_cfg.ch           = _adc_channel;
            ch_cfg.diff_sel     = adc12_sample_signal_single_ended;
            ch_cfg.sample_cycle = HPM_ADC_SAMPLE_CYCLE;

            adc12_init_channel(_adc_unit._adc_unit12, &ch_cfg);

            adc12_prd_config_t prd_cfg;

            prd_cfg.ch           = _adc_channel;
            prd_cfg.prescale     = HPM_ADC_PRESCALE_VALUE;    /* Set divider: 2^22 clocks */
            prd_cfg.period_count = HPM_ADC_PERIOD_COUNT;     /* 6 periods */

            adc12_set_prd_config(_adc_unit._adc_unit12, &prd_cfg);
        }
    }
    else {
        Debug("AnalogIn: adc_init(%d) skipped.\n",  _ardupin);
    }
    return true;
}

// read value from ADC
float AnalogSource::adc_read()
{
    uint16_t result;
    if (_is_adc16) {
        if (adc16_get_prd_result(_adc_unit._adc_unit16, _adc_channel, &result) == status_success) {
        	return (float)result / ((1<<16) - 1) * 3.3f;
        }
    } else {
        if (adc12_get_prd_result(_adc_unit._adc_unit12, _adc_channel, &result) == status_success) {
        	return (float)result / ((1<<12) - 1) * 3.3f;
        }
    }
    return (float)0.0f;
}

/*
   apply a reading in ADC counts
   */
void AnalogSource::_add_value()
{
    if ( _ardupin == ANALOG_INPUT_NONE ) {
        return;
    }

    WITH_SEMAPHORE(_semaphore);

    float value = adc_read();

    _latest_value = value;
    _sum_value += value;
    _sum_count++;

    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}

/*
   setup adc peripheral to capture samples with DMA into a buffer
   */
void AnalogIn::init()
{
}

/*
   called at 1kHz
*/
void AnalogIn::_timer_tick()
{
    for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
        HPMicro::AnalogSource *c = _channels[j];
        if (c != nullptr) {
            // add a value
            c->_add_value();
        }
    }
}

//positive array index (zero is ok), or -1 on error
uint32_t AnalogIn::find_pinconfig(uint32_t ardupin)
{
    // from ardupin, lookup which adc gpio that is..
    for (uint8_t j = 0; j < ADC_GRP1_NUM_CHANNELS; j++) {
        if (pin_config[j].ardupin == ardupin) {
            return j;
        }
    }
    // can't find a match in definitions
    return -1;

}

//
AP_HAL::AnalogSource *AnalogIn::channel(int16_t ardupin)
{
    if (ardupin < 0) ardupin = ANALOG_INPUT_NONE;

    Debug("AnalogIn: configuring channel %d\n", ardupin);

    int8_t pinconfig_offset = find_pinconfig(ardupin);

    uint8_t adc_channel = (uint8_t)ANALOG_INPUT_NONE;
    float scaler = 0;

    if ((ardupin != ANALOG_INPUT_NONE) && (pinconfig_offset == -1 )) {
        Debug("AnalogIn: sorry channel() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        ardupin = ANALOG_INPUT_NONE; // default it to this not terrible value and allow to continue
    }

    // although ANALOG_INPUT_NONE=255 is not a valid pin, we let it through here as
    //  a special case, so that it can be changed with set_pin(..) later.
    if (ardupin != ANALOG_INPUT_NONE) {
        adc_channel = (uint8_t)pin_config[(uint8_t)pinconfig_offset].channel;
        scaler = pin_config[(uint8_t)pinconfig_offset].scaling;
        Debug("AnalogIn: channel(): ardupin %d mapped to adc_channel %d with scaler %f\n",\
                            ardupin, adc_channel, scaler);
    }

    for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {

            _channels[j] = NEW_NOTHROW AnalogSource(ardupin, adc_channel, scaler, 0.0f);

            if (ardupin != ANALOG_INPUT_NONE) {
                Debug("AnalogIn: channel: %d attached to ardupin:%d at adc1_offset:%d\n",\
                                    j, ardupin, adc_channel);
            }
            else {
                Debug("AnalogIn: channel: %d created but using delayed adc and gpio pin configuration\n", j);
            }

            return _channels[j];
        }
    }
    Debug("AnalogIn: out of channels\n");
    return nullptr;
}

