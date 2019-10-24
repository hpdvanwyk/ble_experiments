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

#ifndef BLE_SENSOR_C_H__
#define BLE_SENSOR_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "sdk_config.h"
#include "nrf_sdh_ble.h"
#include "ble_sensor_uuid.h"
#include "sensor.pb.h"
#include "app_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SENSOR_C_BLE_OBSERVER_PRIO 2

#define BLE_SENSOR_C_DEF(_name)                          \
    static ble_sensor_c_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name##_obs,                    \
                         BLE_SENSOR_C_BLE_OBSERVER_PRIO, \
                         ble_sensor_c_on_ble_evt, &_name)

#define BLE_SENSOR_C_ARRAY_DEF(_name, _cnt)               \
    static ble_sensor_c_t _name[_cnt];                    \
    NRF_SDH_BLE_OBSERVERS(_name##_obs,                    \
                          BLE_SENSOR_C_BLE_OBSERVER_PRIO, \
                          ble_sensor_c_on_ble_evt, &_name, _cnt)

#ifndef BLE_SENSOR_C_RR_INTERVALS_MAX_CNT
#define BLE_SENSOR_C_RR_INTERVALS_MAX_CNT 20
#endif

typedef enum {
    BLE_SENSOR_C_EVT_DISCOVERY_COMPLETE = 1,
    BLE_SENSOR_C_EVT_SENSOR_NOTIFICATION
} ble_sensor_c_evt_type_t;

typedef struct
{
    uint16_t hrm_cccd_handle;
    uint16_t hrm_handle;
} sensor_db_t;

typedef struct {
    uint8_t  data[SensorMessage_size];
    uint16_t len;
} ble_sensor_c_data;

typedef struct
{
    ble_sensor_c_evt_type_t evt_type;
    uint16_t                conn_handle;
    union {
        sensor_db_t       peer_db;
        ble_sensor_c_data temperatures;
    } params;
} ble_sensor_c_evt_t;

typedef struct ble_sensor_c_s ble_sensor_c_t;

typedef void (*ble_sensor_c_evt_handler_t)(ble_sensor_c_t* p_ble_sensor_c, ble_sensor_c_evt_t* p_evt);

struct ble_sensor_c_s {
    uint16_t                   conn_handle;
    uint8_t                    uuid_type;
    ble_gap_addr_t             peer_address;
    int8_t                     rssi; // last rssi reading
    sensor_db_t                peer_sensor_db;
    ble_sensor_c_evt_handler_t evt_handler;
};

typedef struct
{
    ble_sensor_c_evt_handler_t evt_handler;
} ble_sensor_c_init_t;

uint32_t ble_sensor_c_init(ble_sensor_c_t* p_ble_sensor_c, ble_sensor_c_init_t* p_ble_sensor_c_init);

void ble_sensor_c_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);

uint32_t ble_sensor_c_notification_enable(ble_sensor_c_t* p_ble_sensor_c);

void ble_sensor_on_db_disc_evt(ble_sensor_c_t* p_ble_sensor_c, const ble_db_discovery_evt_t* p_evt);

uint32_t ble_sensor_c_handles_assign(ble_sensor_c_t*      p_ble_sensor_c,
                                     const ble_gap_evt_t* connection_event,
                                     const sensor_db_t*   p_peer_sensor_handles);

uint32_t ble_sensor_c_update_rssi(ble_sensor_c_t* p_ble_sensor_c,
                                  const int8_t    rssi);

#ifdef __cplusplus
}
#endif

#endif // BLE_SENSOR_C_H__
