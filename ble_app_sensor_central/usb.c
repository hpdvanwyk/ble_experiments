/**
 * Copyright (c) 2019 Hendrik van Wyk
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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
#include <stdbool.h>
#include "nrf_error.h"
#include "nrf_log.h"
#include "app_error.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#define USBD_POWER_DETECTION true

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const*  p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

#define READ_SIZE 1
static char m_rx_buffer[READ_SIZE];

static bool volatile m_port_open = false;
static bool volatile m_tx_in_progress;
static bool volatile m_tx_pending;

uint8_t        m_tx_buf0[NRF_DRV_USBD_EPSIZE * 4];
uint8_t        m_tx_buf1[NRF_DRV_USBD_EPSIZE * 4];
uint8_t*       mp_tx_buf;
static uint8_t m_tx_bytes;

uint8_t* usb_write_buffer(void) {
    return mp_tx_buf;
}

size_t usb_write_buffer_len(void) {
    return sizeof(m_tx_buf0);
}

ret_code_t send_data(int len) {
    ret_code_t err;

    m_tx_bytes = len;

    if (m_tx_in_progress) {
        m_tx_pending = true;
        // No more buffers available, can't continue filling.
        return -1;
    }

    if (m_port_open) {
        m_tx_in_progress = true;

        err = app_usbd_cdc_acm_write(&m_app_cdc_acm,
                                     mp_tx_buf, m_tx_bytes);
        APP_ERROR_CHECK(err);
    }
    mp_tx_buf  = (mp_tx_buf == m_tx_buf0) ? m_tx_buf1 : m_tx_buf0;
    m_tx_bytes = 0;

    return 0;
}

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const*  p_inst,
                                    app_usbd_cdc_acm_user_event_t event) {
    app_usbd_cdc_acm_t const* p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event) {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
        NRF_LOG_INFO("USB port open.")

        /*Setup first transfer*/
        ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                               m_rx_buffer,
                                               READ_SIZE);
        UNUSED_VARIABLE(ret);
        m_port_open = true;
        break;
    }
    case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        m_tx_in_progress = false;
        m_port_open      = false;
        NRF_LOG_INFO("USB port close.")
        break;
    case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        if (m_tx_pending) {
            APP_ERROR_CHECK(app_usbd_cdc_acm_write(p_cdc_acm,
                                                   mp_tx_buf, m_tx_bytes));

            // Switch to the buffer that has just been sent completely
            // and now can be filled again.
            mp_tx_buf  = (mp_tx_buf == m_tx_buf0) ? m_tx_buf1 : m_tx_buf0;
            m_tx_bytes = 0;

            m_tx_pending = false;
        } else {
            m_tx_in_progress = false;
        }
        break;
    case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
        ret_code_t ret;
        NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
        do {
            /*Get amount of data transfered*/
            size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
            NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

            /* Fetch data until internal buffer is empty */
            ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                        m_rx_buffer,
                                        READ_SIZE);
        } while (ret == NRF_SUCCESS);

        break;
    }
    default:
        break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
    switch (event) {
    case APP_USBD_EVT_DRV_SUSPEND:
        break;
    case APP_USBD_EVT_DRV_RESUME:
        break;
    case APP_USBD_EVT_STARTED:
        break;
    case APP_USBD_EVT_STOPPED:
        app_usbd_disable();
        break;
    case APP_USBD_EVT_POWER_DETECTED:
        NRF_LOG_INFO("USB power detected");

        if (!nrf_drv_usbd_is_enabled()) {
            app_usbd_enable();
        }
        break;
    case APP_USBD_EVT_POWER_REMOVED:
        NRF_LOG_INFO("USB power removed");
        app_usbd_stop();
        break;
    case APP_USBD_EVT_POWER_READY:
        NRF_LOG_INFO("USB ready");
        app_usbd_start();
        break;
    default:
        break;
    }
}

void usb_init(void) {
    ret_code_t                     ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler};

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM example started.");

    app_usbd_class_inst_t const* class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret                                        = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION) {
    } else {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
    mp_tx_buf = m_tx_buf0;
}
