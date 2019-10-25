/**
 * Copyright (c) 2019 Hendrik van Wyk
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#include "ble_err.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"

#include "ble_sensor.h"
#include "sensor.pb.h"
#include "pb_encode.h"

static void on_connect(ble_sensor_t* p_ts, ble_evt_t const* p_ble_evt) {
    p_ts->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}

static void on_disconnect(ble_sensor_t* p_ts, ble_evt_t const* p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_ts->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_cccd_write(ble_sensor_t* p_ts, ble_gatts_evt_write_t const* p_evt_write) {
    if (p_evt_write->len == 2) {
        // CCCD written, update indication state
        if (p_ts->evt_handler != NULL) {
            ble_sensor_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)) {
                evt.evt_type = BLE_SENSOR_EVT_NOTIFICATION_ENABLED;
            } else {
                evt.evt_type = BLE_SENSOR_EVT_NOTIFICATION_DISABLED;
            }

            p_ts->evt_handler(p_ts, &evt);
        }
    }
}

static void on_write(ble_sensor_t* p_ts, ble_evt_t const* p_ble_evt) {
    ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ts->meas_handles.cccd_handle) {
        on_cccd_write(p_ts, p_evt_write);
    }
}

static void on_hvc(ble_sensor_t* p_ts, ble_evt_t const* p_ble_evt) {
    ble_gatts_evt_hvc_t const* p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_ts->meas_handles.value_handle) {
        ble_sensor_evt_t evt;

        evt.evt_type = BLE_SENSOR_EVT_INDICATION_CONFIRMED;
        p_ts->evt_handler(p_ts, &evt);
    }
}

void ble_sensor_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context) {
    ble_sensor_t* p_ts = (ble_sensor_t*)p_context;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_ts, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_ts, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_ts, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        on_hvc(p_ts, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

uint32_t ble_sensor_init(ble_sensor_t* p_ts, ble_sensor_init_t const* p_sensor_init) {
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         base_uuid = BLE_UUID_OUR_BASE_UUID;
    ble_add_char_params_t add_char_params;

    ble_uuid.uuid = BLE_UUID_SENSOR_SERVICE;

    // Initialize service structure
    p_ts->evt_handler = p_sensor_init->evt_handler;
    p_ts->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("uuid failure, %i", err_code);
        return err_code;
    }

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ts->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_SENSOR_CHAR;
    add_char_params.uuid_type         = ble_uuid.type;
    add_char_params.max_len           = SensorMessage_size;
    add_char_params.init_len          = 0;
    add_char_params.p_init_value      = NULL;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_sensor_init->ht_meas_cccd_wr_sec;

    err_code = characteristic_add(p_ts->service_handle, &add_char_params, &p_ts->meas_handles);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_sensor_measurement_send(ble_sensor_t* p_ts, SensorMessage* sensor_meas) {
    uint32_t err_code;

    // Send value if connected
    if (p_ts->conn_handle == BLE_CONN_HANDLE_INVALID) {
        return NRF_ERROR_INVALID_STATE;
    }

    uint16_t               len;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint8_t                data[SensorMessage_size];

    bool         status;
    pb_ostream_t stream;

    stream = pb_ostream_from_buffer(data, sizeof(data));
    status = pb_encode(&stream, SensorMessage_fields, sensor_meas);
    len = stream.bytes_written;
    hvx_len = len;
    NRF_LOG_INFO("len send %d", len);

    if (!status) {
        NRF_LOG_ERROR("Encoding failed: %s", PB_GET_ERROR(&stream));
        return NRF_ERROR_INVALID_DATA;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_ts->meas_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = data;

    err_code = sd_ble_gatts_hvx(p_ts->conn_handle, &hvx_params);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len)) {
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}
