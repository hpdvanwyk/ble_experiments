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
#include "ble_gap.h"
#include "nrf_log.h"
#include "usb.h"

#include "sensor.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb.h"

void raw_message_to_log(uint8_t* addr, int8_t rssi, uint8_t* data, uint16_t len) {
    SensorMessage sensor_meas = SensorMessage_init_zero;
    bool          status;
    int           remain;
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(data, len);

    /* Now we are ready to decode the message. */
    status = pb_decode(&stream, SensorMessage_fields, &sensor_meas);

    /* Check for errors... */
    if (!status) {
        NRF_LOG_ERROR("Decoding failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    NRF_LOG_INFO("from %02X:%02X:%02X:%02X:%02X:%02X",
                 addr[0], addr[1],
                 addr[2], addr[3],
                 addr[4], addr[5]);
    NRF_LOG_INFO("rssi: %d", rssi);
    int i;
    for (i = 0; i < sensor_meas.Readings_count; i++) {
        uint8_t* id = sensor_meas.Readings[i].Id.bytes;
        NRF_LOG_INFO("sensor %02X:%02X:%02X:%02X:%02X:%02X",
                     id[0], id[1],
                     id[2], id[3],
                     id[4], id[5]);
        int16_t temp = sensor_meas.Readings[i].Temperature;
        if (temp < 0) {
            remain = -temp % 10;
        } else {
            remain = temp % 10;
        }
        NRF_LOG_INFO("got temp %d.%d ", temp / 10, remain);
    }
}

bool central_message_to_usb(const uint8_t* addr, int8_t rssi, pb_ostream_t* ostream) {
    CentralMessage msg = CentralMessage_init_zero;
    msg.rssi           = rssi;
    memcpy(msg.RemoteId, addr, BLE_GAP_ADDR_LEN);
    return pb_encode_delimited(ostream, CentralMessage_fields, &msg);
}

void raw_message_to_usb(const uint8_t* addr, int8_t rssi, uint8_t* data, uint16_t len) {
    bool         status;
    ret_code_t   ret;
    pb_ostream_t ostream;

    ostream = pb_ostream_from_buffer(usb_write_buffer(), usb_write_buffer_len());
    status  = central_message_to_usb(addr, rssi, &ostream);
    if (!status) {
        NRF_LOG_ERROR("failed writing message");
        return;
    }
    status = pb_encode_varint(&ostream, len);
    if (!status) {
        NRF_LOG_ERROR("failed writing message");
        return;
    }
    status = pb_write(&ostream, data, len);
    if (!status) {
        NRF_LOG_ERROR("failed writing message");
        return;
    }

    NRF_LOG_INFO("sending %d", ostream.bytes_written);

    ret = send_data(ostream.bytes_written);

    if (ret != NRF_SUCCESS) {
        NRF_LOG_WARNING("message pending");
    }
}

void message_to_usb(uint8_t* addr, int8_t rssi, SensorMessage* msg) {

    ret_code_t   ret;
    bool         status;
    pb_ostream_t ostream;

    ostream = pb_ostream_from_buffer(usb_write_buffer(), usb_write_buffer_len());

    status = central_message_to_usb(addr, rssi, &ostream);
    if (!status) {
        NRF_LOG_ERROR("failed writing central message");
        return;
    }

    status = pb_encode_delimited(&ostream, SensorMessage_fields, msg);
    if (!status) {
        NRF_LOG_ERROR("failed writing sensor message");
        return;
    }

    NRF_LOG_INFO("sending %d", ostream.bytes_written)

    ret = send_data(ostream.bytes_written);

    if (ret != NRF_SUCCESS) {
        NRF_LOG_WARNING("message pending");
    }
}