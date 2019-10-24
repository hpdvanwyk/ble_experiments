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
#include <stdint.h>
#include "nrf_log.h"
#include "ble_gap.h"

#include "sensor.pb.h"
#include "pb_encode.h"
#include "pb.h"
#include "usb.h"

#define MJ_HUMIDITY 0x06
#define MJ_HUMIDITY_LEN 2

#define MJ_TEMPERATURE 0x04
#define MJ_TEMPERATURE_LEN 2

#define MJ_HUMIDITY_TEMPERATURE 0x0d
#define MJ_HUMIDITY_TEMPERATURE_LEN 4

#define MJ_BATTERY 0x0a
#define MJ_BATTERY_LEN 1

#define MJ_TYPE_OFFSET 11
#define MJ_LEN_OFFSET 13

#define MJ_DATA_OFFSET 14
#define MJ_DATA2_OFFSET 16

ble_uuid_t mj_uuid = {
    .uuid = 0xfe95,
    .type = BLE_UUID_TYPE_BLE,
};

bool mj_parse_to_msg(uint8_t const* p_data,
                     uint16_t       data_len,
                     SensorMessage* msg) {
    // Possibly too many sanity checks in this function. Or too few?
    if (data_len <= MJ_DATA_OFFSET) {
        return false;
    }
    if (p_data[MJ_LEN_OFFSET] + MJ_LEN_OFFSET >= data_len) {
        return false;
    }

    switch (p_data[MJ_TYPE_OFFSET]) {
    case MJ_HUMIDITY_TEMPERATURE: {
        if (p_data[MJ_LEN_OFFSET] != MJ_HUMIDITY_TEMPERATURE_LEN) {
            return false;
        }
        NRF_LOG_INFO("temperature %d humidity %d",
                     uint16_decode(&p_data[MJ_DATA_OFFSET]),
                     uint16_decode(&p_data[MJ_DATA2_OFFSET]));
        msg->MJReading.Temperature = uint16_decode(&p_data[MJ_DATA_OFFSET]);
        msg->MJReading.Humidity    = uint16_decode(&p_data[MJ_DATA2_OFFSET]);
    } break;
    case MJ_HUMIDITY: {
        if (p_data[MJ_LEN_OFFSET] != MJ_HUMIDITY_LEN) {
            return false;
        }
        NRF_LOG_INFO("humidity %d", uint16_decode(&p_data[MJ_DATA_OFFSET]));
        msg->MJReading.Humidity = uint16_decode(&p_data[MJ_DATA_OFFSET]);
    } break;
    case MJ_TEMPERATURE: {
        if (p_data[MJ_LEN_OFFSET] != MJ_TEMPERATURE_LEN) {
            return false;
        }
        NRF_LOG_INFO("temperature %d", uint16_decode(&p_data[MJ_DATA_OFFSET]));
        msg->MJReading.Temperature = uint16_decode(&p_data[MJ_DATA_OFFSET]);
    } break;
    case MJ_BATTERY: {
        if (p_data[MJ_LEN_OFFSET] != MJ_BATTERY_LEN) {
            return false;
        }

        NRF_LOG_INFO("battery %d", p_data[MJ_DATA_OFFSET]);
        msg->MJReading.Battery = p_data[MJ_DATA_OFFSET];
    } break;
    default:
        break;
        return false;
    }
    return true;
}
