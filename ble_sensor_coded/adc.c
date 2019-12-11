/**
 * Copyright (c) 2019 Hendrik van Wyk
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "adc.h"
#include "nrf_gpio.h"

#define SAMPLES_IN_BUFFER 375
#define BAT_SAMPLES 1
#define CT_SAMPLES SAMPLES_IN_BUFFER

#define ADC_BAT_INTERVAL APP_TIMER_TICKS(120011)
#define ADC_CT_INTERVAL APP_TIMER_TICKS(15013)
#define ADC_CALIBRATE_INTERVAL APP_TIMER_TICKS(600000)
#define ADC_WAIT_INTERVAL APP_TIMER_TICKS(1000)

#define OPAMP_PWR_PIN NRF_GPIO_PIN_MAP(1, 10)

typedef void (*adc_callback)(const nrfx_saadc_done_evt_t* data);

APP_TIMER_DEF(m_adc_bat_timer_id);
APP_TIMER_DEF(m_adc_ct_timer_id);
APP_TIMER_DEF(m_adc_calibrate_timer_id);

int current_ct = 0;

static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];

adc_callback callback;

bool adc_lib_in_use = false;

static nrf_saadc_input_t ct_inputs[] = {
    NRF_SAADC_INPUT_AIN5,
    NRF_SAADC_INPUT_AIN0,
};

extern SensorMessage sensor_meas;

int16_t saadc_convert_to_volts(int16_t input) {
    return (5 * 1000 * (int32_t)input) / 11378;
}

void saadc_callback(nrfx_saadc_evt_t const* p_event) {
    if (p_event->type == NRFX_SAADC_EVT_DONE) {
        nrfx_saadc_uninit();
        callback(&p_event->data.done);
        adc_lib_in_use = false;
    }
    if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
        nrfx_saadc_uninit();
        adc_lib_in_use = false;
    }
}

bool saadc_init(
    nrfx_saadc_config_t*        saadc_config,
    nrf_saadc_channel_config_t* channel_config,
    uint32_t                    len,
    adc_callback                c) {

    nrfx_err_t err_code;
    if (adc_lib_in_use) {
        return true;
    }
    adc_lib_in_use = true;

    err_code = nrfx_saadc_init(saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool, len);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_continuous_mode_enable(80);

    callback = c;
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);
    return false;
}

bool saadc_sample_bat(adc_callback c) {
    nrfx_saadc_config_t saadc_config =
        NRFX_SAADC_DEFAULT_CONFIG;
    saadc_config.low_power_mode = true;
    saadc_config.resolution     = SAADC_RESOLUTION_VAL_12bit;
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over256x;

    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);
    channel_config.burst = SAADC_CH_CONFIG_BURST_Enabled;

    return saadc_init(&saadc_config, &channel_config, BAT_SAMPLES, c);
}

bool saadc_sample_ct(nrf_saadc_input_t input, adc_callback c) {
    NRF_LOG_INFO("CT start");
    nrfx_saadc_config_t saadc_config =
        NRFX_SAADC_DEFAULT_CONFIG;
    saadc_config.low_power_mode = false;
    saadc_config.resolution     = SAADC_RESOLUTION_VAL_14bit;
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over128x;

    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(input);
    channel_config.burst    = SAADC_CH_CONFIG_BURST_Disabled;
    channel_config.gain     = NRF_SAADC_GAIN1_5;
    channel_config.acq_time = NRF_SAADC_ACQTIME_5US;

    return saadc_init(&saadc_config, &channel_config, CT_SAMPLES, c);
}

bool saadc_calibrate() {
    nrfx_err_t          err_code;
    nrfx_saadc_config_t saadc_config =
        NRFX_SAADC_DEFAULT_CONFIG;
    saadc_config.low_power_mode = false;
    saadc_config.resolution     = SAADC_RESOLUTION_VAL_14bit;
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over32x;

    NRF_LOG_INFO("ADC calibrate start");

    if (adc_lib_in_use) {
        return true;
    }
    adc_lib_in_use = true;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(err_code);
    return false;
}

static void adc_bat_callback_handler(const nrfx_saadc_done_evt_t* data) {
    if (data->size == BAT_SAMPLES) {
        sensor_meas.BatteryVoltage = saadc_convert_to_volts(data->p_buffer[0]);
        NRF_LOG_INFO("%d", sensor_meas.BatteryVoltage);
    }
}

static void adc_timer_scheduler(bool busy, uint32_t interval, app_timer_id_t timer_id) {
    ret_code_t err_code;

    uint32_t timeout = interval;
    if (busy) {
        NRF_LOG_DEBUG("ADC BUSY");
        timeout = ADC_WAIT_INTERVAL;
    }

    err_code = app_timer_start(timer_id, timeout, NULL);
    APP_ERROR_CHECK(err_code);
}

static void adc_ct_callback_handler(const nrfx_saadc_done_evt_t* data) {
    nrf_gpio_pin_clear(OPAMP_PWR_PIN);
    if (data->size == CT_SAMPLES) {
        int     i;
        int32_t total = 0;
        float   avg   = 0;
        for (i = 0; i < data->size; i++) {
            total += data->p_buffer[i];
        }
        avg = (float)total / (float)(data->size);
        NRF_LOG_INFO("dc: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(avg));
        float total_squared = 0;
        float rms;
        for (i = 0; i < data->size; i++) {
            total_squared += pow((float)(data->p_buffer[i]) - avg, 2.0);
        }
        rms = sqrt(total_squared / (float)data->size);
        NRF_LOG_INFO("rms " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(rms));
        //RESULT = V * (GAIN/REFERENCE) * 2^(RESOLUTION)
        float rms_v = rms / (((1.0 / 5.0) / 0.6) * 16384.0);
        // 120 ohm load resistor. 100A to 50mA ct
        float ct_rms_a = (rms_v / 120.0) * 2000.0;
        NRF_LOG_INFO("rms " NRF_LOG_FLOAT_MARKER "mV", NRF_LOG_FLOAT(1000 * rms_v));
        NRF_LOG_INFO("ct rms A " NRF_LOG_FLOAT_MARKER "mA", NRF_LOG_FLOAT(1000 * ct_rms_a));
        NRF_LOG_INFO("watts " NRF_LOG_FLOAT_MARKER "W", NRF_LOG_FLOAT(ct_rms_a * 230));
        sensor_meas.Readings[current_ct].Current     = ct_rms_a;
        sensor_meas.Readings[current_ct].Id.bytes[0] = current_ct + 1,
        sensor_meas.Readings[current_ct].Id.size     = 1;

        current_ct++;
        current_ct = current_ct % sizeof(ct_inputs);
    }
}

static void adc_bat_timer_handler(void* p_context) {
    bool busy = saadc_sample_bat(adc_bat_callback_handler);
    adc_timer_scheduler(busy, ADC_BAT_INTERVAL, m_adc_bat_timer_id);
}

static void adc_ct_timer_handler(void* p_context) {
    nrf_gpio_pin_set(OPAMP_PWR_PIN);
    NRF_LOG_INFO("sample ct %d", current_ct);
    bool busy = saadc_sample_ct(ct_inputs[current_ct], adc_ct_callback_handler);
    adc_timer_scheduler(busy, ADC_CT_INTERVAL, m_adc_ct_timer_id);
}

static void adc_calibrate_timer_handler(void* p_context) {
    bool busy = saadc_calibrate();
    adc_timer_scheduler(busy, ADC_CALIBRATE_INTERVAL, m_adc_calibrate_timer_id);
}

void adclib_init() {
    ret_code_t err_code;

    nrf_gpio_cfg_output(OPAMP_PWR_PIN);
    nrf_gpio_pin_clear(OPAMP_PWR_PIN);
    saadc_calibrate();

    err_code = app_timer_create(&m_adc_bat_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                adc_bat_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adc_ct_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                adc_ct_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adc_calibrate_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                adc_calibrate_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_adc_bat_timer_id, ADC_BAT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_adc_ct_timer_id, ADC_CT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_adc_calibrate_timer_id, ADC_CALIBRATE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}