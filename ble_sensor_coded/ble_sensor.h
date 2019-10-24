/**
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

#ifndef BLE_SENSOR_H__
#define BLE_SENSOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_date_time.h"
#include "nrf_sdh_ble.h"
#include "ble_sensor_uuid.h"
#include "sensor.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SENSOR_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_ts instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_SENSOR_DEF(_name)                           \
    static ble_sensor_t _name;                          \
    NRF_SDH_BLE_OBSERVER(_name##_obs,               \
                         BLE_SENSOR_BLE_OBSERVER_PRIO, \
                         ble_sensor_on_ble_evt, &_name)

typedef enum {
    BLE_SENSOR_EVT_NOTIFICATION_ENABLED,
    BLE_SENSOR_EVT_NOTIFICATION_DISABLED,
    BLE_SENSOR_EVT_INDICATION_CONFIRMED
} ble_sensor_evt_type_t;

typedef struct
{
    ble_sensor_evt_type_t evt_type; /**< Type of event. */
} ble_sensor_evt_t;

// Forward declaration of the ble_sensor_t type.
typedef struct ble_sensor_s ble_sensor_t;

typedef void (*ble_sensor_evt_handler_t)(ble_sensor_t* p_ts, ble_sensor_evt_t* p_evt);

typedef struct
{
    ble_sensor_evt_handler_t evt_handler;
    security_req_t       ht_meas_cccd_wr_sec;
} ble_sensor_init_t;

struct ble_sensor_s {
    ble_sensor_evt_handler_t     evt_handler;
    uint16_t                 service_handle;
    ble_gatts_char_handles_t meas_handles;
    uint16_t                 conn_handle;
};

uint32_t ble_sensor_init(ble_sensor_t* p_ts, const ble_sensor_init_t* p_sensor_init);

void ble_sensor_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);

uint32_t ble_sensor_measurement_send(ble_sensor_t* p_sensor, SensorMessage* sensor_sensor);


#ifdef __cplusplus
}
#endif

#endif // BLE_SENSOR_H__
