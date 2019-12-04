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
#include "nrf.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "adc.h"

static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];

adc_callback callback;

bool adc_lib_in_use = false;

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

    nrf_saadc_continuous_mode_enable(2000);

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
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over8x;

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
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over64x;

    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(input);
    channel_config.burst = SAADC_CH_CONFIG_BURST_Enabled;
    channel_config.gain  = NRF_SAADC_GAIN1_5;

    return saadc_init(&saadc_config, &channel_config, CT_SAMPLES, c);
}

bool saadc_calibrate() {
    nrfx_err_t          err_code;
    nrfx_saadc_config_t saadc_config =
        NRFX_SAADC_DEFAULT_CONFIG;
    saadc_config.low_power_mode = false;
    saadc_config.resolution     = SAADC_RESOLUTION_VAL_14bit;
    saadc_config.oversample     = SAADC_OVERSAMPLE_OVERSAMPLE_Over32x;

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