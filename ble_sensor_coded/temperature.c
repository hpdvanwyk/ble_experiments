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

#include "nrf_log.h"
#include "nrf_gpio.h"
#include "utility/onewire.h"
#include "utility/hires.h"
#include "ow_temp/ow_temp.h"
#include "temperature.h"

#define TEMPERATURE_MEAS_INTERVAL APP_TIMER_TICKS(31013)
#define SENSORS_READ_TIME APP_TIMER_TICKS(1000)
APP_TIMER_DEF(m_temp_timer_id);
APP_TIMER_DEF(m_temp_send_timer_id);

oo_power_t ow_power;
#define ONEWIRE_PWR_PIN NRF_GPIO_PIN_MAP(0, 22)

#define ONEWIRE_DQ_PIN 13
APP_TIMER_DEF(m_onewire_timer_id);
oo_temp_reader_t ow_temp1;

#define ONEWIRE_DQ_PIN2 15
//#define ONEWIRE_PWR_PIN2 NRF_GPIO_PIN_MAP(0, 13)
APP_TIMER_DEF(m_onewire2_timer_id);
oo_temp_reader_t ow_temp2;

#define ONEWIRE_DQ_PIN3 17
//#define ONEWIRE_PWR_PIN3 NRF_GPIO_PIN_MAP(0, 17)
APP_TIMER_DEF(m_onewire3_timer_id);
oo_temp_reader_t ow_temp3;

#define ONEWIRE_DQ_PIN4 20
//#define ONEWIRE_PWR_PIN4 NRF_GPIO_PIN_MAP(0, 22)
APP_TIMER_DEF(m_onewire4_timer_id);
oo_temp_reader_t ow_temp4;

extern SensorMessage sensor_meas;
extern int8_t        last_rssi;
extern void          readings_send(void* p_context);

static void temperature_update(ow_temp_reading_t* reading) {

    int i                               = sensor_meas.Readings_count;
    sensor_meas.Readings[i].Temperature = reading->temperature;
    memcpy(sensor_meas.Readings[i].Id.bytes,
           reading->serial.id,
           ID_SIZE); // C does not have a sane way of getting a struct members size.
    sensor_meas.Readings[i].Id.size = ID_SIZE;
    sensor_meas.Readings_count++;

    NRF_LOG_INFO("temp1 %d", reading->temperature);
    NRF_LOG_INFO("Serial %02x:%02x:%02x:%02x:%02x:%02x ",
                 reading->serial.id[0], reading->serial.id[1], reading->serial.id[2],
                 reading->serial.id[3], reading->serial.id[4], reading->serial.id[5]);
}

static void temperature_meas_timeout_handler(void* p_context) {
    ret_code_t err_code;
    UNUSED_PARAMETER(p_context);
    read_one_wire_temp(&ow_temp1, temperature_update, 11);
    read_one_wire_temp(&ow_temp2, temperature_update, 11);
    read_one_wire_temp(&ow_temp3, temperature_update, 11);
    read_one_wire_temp(&ow_temp4, temperature_update, 11);
    err_code = app_timer_start(m_temp_send_timer_id, SENSORS_READ_TIME, NULL);
    APP_ERROR_CHECK(err_code);
}

void temperaturelib_init() {
    ret_code_t err_code;
    err_code = app_timer_create(&m_temp_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                temperature_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_temp_send_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                readings_send);
    APP_ERROR_CHECK(err_code);

    one_wire_power_init(&ow_power, ONEWIRE_PWR_PIN);
    one_wire_init(&ow_temp1, m_onewire_timer_id, ONEWIRE_DQ_PIN, &ow_power);
    one_wire_init(&ow_temp2, m_onewire2_timer_id, ONEWIRE_DQ_PIN2, &ow_power);
    one_wire_init(&ow_temp3, m_onewire3_timer_id, ONEWIRE_DQ_PIN3, &ow_power);
    one_wire_init(&ow_temp4, m_onewire4_timer_id, ONEWIRE_DQ_PIN4, &ow_power);

    // Start application timers.
    err_code = app_timer_start(m_temp_timer_id, TEMPERATURE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}