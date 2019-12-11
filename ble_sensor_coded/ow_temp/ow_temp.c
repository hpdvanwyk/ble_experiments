/*
Copyright (c) 2019 Hendrik van Wyk
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "ow_temp.h"
#include "nrf_log.h"
#include "app_error.h"
#include "nrf_gpio.h"

// Form the DS12B20 datasheet
#define OO_TEMP_WAIT_9_BIT APP_TIMER_TICKS(94)
#define OO_TEMP_WAIT_10_BIT APP_TIMER_TICKS(188)
#define OO_TEMP_WAIT_11_BIT APP_TIMER_TICKS(375)
#define OO_TEMP_WAIT_12_BIT APP_TIMER_TICKS(750)

#define DS18B20 0x28
#define DS18S20 0x10

void power_on(oo_power_t* power) {
    if (power->count == 0) {
        nrf_gpio_pin_clear(power->power_pin);
    }
    power->count++;
}

void power_off(oo_power_t* power) {
    power->count--;
    if (power->count == 0) {
        nrf_gpio_pin_set(power->power_pin);
    }
}

int set_resolution(oo_temp_reader_t* reader, int resolution) {
    int rc;
    rc = iBSPACMonewireSetResolution(reader->bus, resolution);
    if (rc != 0) {
        NRF_LOG_INFO("ERR setting resolution P0.%u", reader->bus->dq_pin);
        return -1;
    }
    switch (resolution) {
    case 9:
        reader->convert_wait = OO_TEMP_WAIT_9_BIT;
        break;
    case 10:
        reader->convert_wait = OO_TEMP_WAIT_10_BIT;
        break;
    case 11:
        reader->convert_wait = OO_TEMP_WAIT_11_BIT;
        break;
    case 12:
        reader->convert_wait = OO_TEMP_WAIT_12_BIT;
        break;
    default:
        NRF_LOG_INFO("ERR invalid resolution P0.%u", resolution);
        return -1;
        break;
    }
    return 0;
}

int start_temp_conversion(oo_temp_reader_t* reader, int resolution) {
    int rc;
    if (!iBSPACMonewireReset(reader->bus)) {
        NRF_LOG_INFO("ERR conversion: No DS18B20 present on P0.%u", reader->bus->dq_pin);
        return -1;
    }

    rc = iBSPACMonewireReadSerialNumber(reader->bus, &(reader->reading.serial));
    if (rc != 0) {
        NRF_LOG_INFO("ERR serial failure on P0.%u", reader->bus->dq_pin);
        return -1;
    }

    if (reader->reading.serial.family == DS18B20) {
        rc = set_resolution(reader, resolution);
        if (rc != 0) {
            return -1;
        }
    } else {
        reader->convert_wait = OO_TEMP_WAIT_12_BIT;
    }

    rc = iBSPACMonewireRequestTemperature(reader->bus);
    if (rc != 0) {
        return -1;
    }
    vBSPACMonewireParasitePower(reader->bus, true);
    return 0;
}

void hiresOn() {
    int rc;
    /* Configure high-resolution timer at 1 MHz */
    rc = iBSPACMhiresInitialize(1000U * 1000U);
    if (0 != rc) {
        NRF_LOG_ERROR("ERR: Failed to initialize high-resolution clock");
        return;
    }
    iBSPACMhiresSetEnabled(true);
}

void hiresOff() {
    ret_code_t err_code;
    iBSPACMhiresSetEnabled(false);
    err_code = sd_clock_hfclk_release();
    APP_ERROR_CHECK(err_code);
}

static void temp_wait_timeout_handler(void* p_context) {
    oo_temp_reader_t* reader = p_context;
    int               rc;
    int16_t           t_xCel = -999;
    uint8_t           count_remain;

    hiresOn();

    vBSPACMonewireParasitePower(reader->bus, false);

    rc = iBSPACMonewireReadTemperature(reader->bus, &t_xCel, &count_remain);
    vBSPACMonewireShutdown(reader->bus);
    hiresOff();
    power_off(reader->power);
    if (rc != 0) {
        NRF_LOG_ERROR("ERROR: read temperature error");
        return;
    }

    switch (reader->reading.serial.family) {
    case DS18S20:
        if (t_xCel == 0xaa) { // power up state, something went wrong
            return;
        }
        // See the DS18S20 datasheet for more information on this calculation.
        int16_t preadj_temp         = 10 * (t_xCel >> 1); // truncate last bit and multiply by 10
        reader->reading.temperature = preadj_temp + ((10 * (16 - count_remain)) / 16);
        break;
    case DS18B20:
        if (t_xCel == 0x550) {
            return;
        }
        reader->reading.temperature = (t_xCel * 10) / 16;
        break;
    default:
        reader->reading.temperature = t_xCel;
    }

    if (reader->reading.temperature == 0) {
        // If the temperature happens to actually be 0C bump it to 0.1C so that protobuf will send something.
        reader->reading.temperature = 1;
    }

    reader->callback(&reader->reading, reader->callback_context);
    return;
}

int read_one_wire_temp(oo_temp_reader_t* reader, int resolution, temp_callback callback, void* context) {
    int ret;
    power_on(reader->power);
    hiresOn();
    ret = start_temp_conversion(reader, resolution);
    if (ret != 0) {
        vBSPACMonewireShutdown(reader->bus);
        hiresOff();
        power_off(reader->power);
        return ret;
    }
    reader->callback = callback;
    reader->callback_context = context;

    ret_code_t err_code = app_timer_start(reader->timer_id, reader->convert_wait, reader);
    APP_ERROR_CHECK(err_code);
    hiresOff();

    return 0;
}

void one_wire_power_init(oo_power_t* power, uint32_t pwr_pin) {
    power->count     = 0;
    power->power_pin = pwr_pin;
    nrf_gpio_cfg_output(pwr_pin);
    nrf_gpio_pin_set(pwr_pin);
}

void one_wire_init(oo_temp_reader_t* reader, app_timer_id_t timer_id, int dq_pin, oo_power_t* power) {
    // can't get parasitic power to work so I will do my own thing.
    reader->bus = hBSPACMonewireConfigureBus(&reader->bus_config, dq_pin, -1);

    ret_code_t err_code;
    err_code = app_timer_create(&timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                temp_wait_timeout_handler);
    APP_ERROR_CHECK(err_code);
    reader->timer_id = timer_id;
    reader->power    = power;
}
