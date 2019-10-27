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

#include <stdint.h>
#include <stdio.h>
#include "ble.h"
#include "sdk_errors.h"
#include "ble_advdata.h"

#define UUID128_SIZE 16 /**< Size of 128 bit UUID. */
#define UUID16_SIZE 2   /**< Size of 16 bit UUID. */
uint16_t ble_advdata_service_data_uuid_find(uint8_t const*    p_encoded_data,
                                            uint16_t          data_len,
                                            uint16_t*         p_offset,
                                            ble_uuid_t const* p_target_uuid) {

    ret_code_t     err_code;
    uint16_t       data_offset  = 0;
    uint8_t        raw_uuid_len = UUID128_SIZE;
    uint8_t const* p_parsed_uuid;
    uint8_t        raw_uuid[UUID128_SIZE];
    uint16_t       parsed_data_len;

    err_code = sd_ble_uuid_encode(p_target_uuid, &raw_uuid_len, raw_uuid);

    if ((p_encoded_data == NULL) || (err_code != NRF_SUCCESS)) {
        // Invalid p_encoded_data or p_target_uuid.
        return 0;
    }

    switch (raw_uuid_len) {
    case UUID16_SIZE:
        break;

    default:
        return 0;
    }

    parsed_data_len = ble_advdata_search(p_encoded_data, data_len, &data_offset, BLE_GAP_AD_TYPE_SERVICE_DATA);

    if (data_offset == 0) {
        // Could not find any relevant UUIDs in the encoded data.
        return 0;
    }

    p_parsed_uuid = &p_encoded_data[data_offset];

    // Verify if any UUID matches the given UUID.
    if (memcmp(p_parsed_uuid, raw_uuid, raw_uuid_len) == 0) {
        *p_offset = data_offset + raw_uuid_len;
        return parsed_data_len - raw_uuid_len;
    }
    return 0;
}