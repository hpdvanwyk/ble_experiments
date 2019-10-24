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
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "app_button.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_ble_lesc.h"
#include "ble_advdata.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "mjparser/mjparser.h"
#include "ble_sensor_c.h"
#include "usb.h"
#include "pb_usb/pb_usb.h"

#define APP_BLE_CONN_CFG_TAG 1  /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO 3 /**< BLE observer priority of the application. There is no need to modify this value. */

#define CENTRAL_SCANNING_LED BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED BSP_BOARD_LED_1
#define LEDBUTTON_LED BSP_BOARD_LED_2 /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON BSP_BUTTON_0              /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define RSSI_CHANGE_THRESHOLD 3
#define RSSI_CHANGE_SKIP 5

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 1                               /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

NRF_BLE_GATT_DEF(m_gatt);                                              /**< GATT module instance. */
BLE_SENSOR_C_ARRAY_DEF(m_sensor_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);    /**< LED button client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                              /**< Scanning Module instance. */
NRF_BLE_SCAN_DEF(m_scan_noconnect);                                    /**< Scanning Module instance. */

#define DEVICE_NAME "Can has bluetooth?"                /**< Name of device. Will be included in the advertising data. */
static char const m_target_periph_name[] = DEVICE_NAME; /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for initializing the LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void) {
    bsp_board_init(BSP_INIT_LEDS);
}

static void scan_evt_handler(scan_evt_t const* p_scan_evt) {
    ret_code_t err_code;

    switch (p_scan_evt->scan_evt_id) {
    case NRF_BLE_SCAN_EVT_CONNECTING_ERROR: {
        err_code = p_scan_evt->params.connecting_err.err_code;
        APP_ERROR_CHECK(err_code);
    } break;

    default:
        break;
    }
}

static ble_gap_scan_params_t m_scan_param = /**< Scan parameters requested for scanning and connection. */
    {
        .active        = 0,
        .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
        .window        = NRF_BLE_SCAN_SCAN_WINDOW,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
        .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
        .scan_phys     = BLE_GAP_PHY_CODED | BLE_GAP_PHY_1MBPS,
        .extended      = true,
};

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(nrf_ble_scan_t* const p_scan_ctx, bool connect) {
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = connect;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_scan_param     = &m_scan_param;

    err_code = nrf_ble_scan_init(p_scan_ctx, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(p_scan_ctx, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(p_scan_ctx, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting scanning. */
static void scan_start(nrf_ble_scan_t* const p_scan_ctx) {
    ret_code_t ret;

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = nrf_ble_scan_start(p_scan_ctx);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}
static void sensor_c_evt_handler(ble_sensor_c_t* p_sensor_c, ble_sensor_c_evt_t* p_sensor_c_evt) {
    switch (p_sensor_c_evt->evt_type) {
    case BLE_SENSOR_C_EVT_DISCOVERY_COMPLETE: {
        ret_code_t err_code;

        NRF_LOG_INFO("TEMP discovered on conn_handle 0x%x",
                     p_sensor_c_evt->conn_handle);

        // Initiate bonding.
        err_code = pm_conn_secure(p_sensor_c_evt->conn_handle, false);
        if (err_code != NRF_ERROR_BUSY) {
            APP_ERROR_CHECK(err_code);
        }

        err_code = ble_sensor_c_notification_enable(p_sensor_c);
        APP_ERROR_CHECK(err_code);
    } break; // BLE_SENSOR_C_EVT_DISCOVERY_COMPLETE
    case BLE_SENSOR_C_EVT_SENSOR_NOTIFICATION: {
        raw_message_to_log(p_sensor_c->peer_address.addr,
                           p_sensor_c->rssi,
                           p_sensor_c_evt->params.temperatures.data,
                           p_sensor_c_evt->params.temperatures.len);
        raw_message_to_usb(p_sensor_c->peer_address.addr,
                           p_sensor_c->rssi,
                           p_sensor_c_evt->params.temperatures.data,
                           p_sensor_c_evt->params.temperatures.len);
    } break;

    default:
        // No implementation needed.
        break;
    }
}

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

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const* p_ble_evt, void* p_context) {
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const* p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
    // Upon connection, check which peripheral is connected, initiate DB
    // discovery, update LEDs status, and resume scanning, if necessary.
    case BLE_GAP_EVT_CONNECTED: {
        NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                     p_gap_evt->conn_handle);

        APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

        err_code = ble_sensor_c_handles_assign(&m_sensor_c[p_gap_evt->conn_handle],
                                               p_gap_evt,
                                               NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                          p_gap_evt->conn_handle);
        if (err_code != NRF_ERROR_BUSY) {
            APP_ERROR_CHECK(err_code);
        }

        err_code = sd_ble_gap_rssi_start(p_gap_evt->conn_handle, RSSI_CHANGE_THRESHOLD, RSSI_CHANGE_SKIP);
        APP_ERROR_CHECK(err_code);

        // Update LEDs status and check whether it is needed to look for more
        // peripherals to connect to.
        bsp_board_led_on(CENTRAL_CONNECTED_LED);
        if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            scan_start(&m_scan_noconnect);
        } else {
            // Resume scanning.
            bsp_board_led_on(CENTRAL_SCANNING_LED);
            scan_start(&m_scan);
        }
    } break; // BLE_GAP_EVT_CONNECTED

    // Upon disconnection, reset the connection handle of the peer that disconnected, update
    // the LEDs status and start scanning again.
    case BLE_GAP_EVT_DISCONNECTED: {
        NRF_LOG_INFO("Sensor central link 0x%x disconnected (reason: 0x%x)",
                     p_gap_evt->conn_handle,
                     p_gap_evt->params.disconnected.reason);

        if (ble_conn_state_central_conn_count() == 0) {
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Turn off the LED that indicates the connection.
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
        }

        // Start scanning.
        scan_start(&m_scan);

        // Turn on the LED for indicating scanning.
        bsp_board_led_on(CENTRAL_SCANNING_LED);

    } break;

    case BLE_GAP_EVT_TIMEOUT: {
        // Timeout for scanning is not specified, so only the connection requests can time out.
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
            NRF_LOG_DEBUG("Connection request timed out.");
        }
    } break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
        // Accept parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT: {
        // Disconnect on GATT client timeout event.
        NRF_LOG_DEBUG("GATT client timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTS_EVT_TIMEOUT: {
        // Disconnect on GATT server timeout event.
        NRF_LOG_DEBUG("GATT server timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_RSSI_CHANGED: {
        int8_t rssi = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
        err_code    = ble_sensor_c_update_rssi(&m_sensor_c[p_gap_evt->conn_handle], rssi);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_ADV_REPORT: {
        const ble_gap_evt_adv_report_t* report           = &p_gap_evt->params.adv_report;
        uint16_t                        service_data_off = 0;
        uint16_t                        service_data_len = ble_advdata_service_data_uuid_find(report->data.p_data,
                                                                       report->data.len,
                                                                       &service_data_off,
                                                                       &mj_uuid);
        if (service_data_len != 0) {
            NRF_LOG_DEBUG("Found mijia")
            uint8_t*      data = &report->data.p_data[service_data_off];
            SensorMessage msg  = SensorMessage_init_zero;

            bool ok = mj_parse_to_msg(data, service_data_len, &msg);
            if (!ok) {
                return;
            }
            message_to_usb(report->peer_addr.addr, report->rssi, &msg);
        }
    }

    default:
        // No implementation needed.
        break;
    }
}

static void sensor_c_init(void) {
    ret_code_t          err_code;
    ble_sensor_c_init_t sensor_c_init_obj;

    sensor_c_init_obj.evt_handler = sensor_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++) {
        err_code = ble_sensor_c_init(&m_sensor_c[i], &sensor_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
    switch (event) {
    case BSP_EVENT_SLEEP:
        break;

    case BSP_EVENT_DISCONNECT:
        break;

    case BSP_EVENT_KEY_0:
        break;

    default:
        break;
    }
}

static void buttons_init(bool* p_erase_bonds) {
    ret_code_t  err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t* p_evt) {
    NRF_LOG_DEBUG("call to ble_sensor_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_sensor_on_db_disc_evt(&m_sensor_c[p_evt->conn_handle], p_evt);
}

/** @brief Database discovery initialization.
 */
static void db_discovery_init(void) {
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details This function handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void) {
    ret_code_t err_code;
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
    while (app_usbd_event_queue_process()) {
        /* Nothing to do */
    }

    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

/** @brief Function for initializing the log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/** @brief Function for initializing the timer.
 */
static void timer_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

#if defined(BOARD_PCA100593v3)
/**
 * Function for configuring UICR_REGOUT0 register
 * to set GPIO output voltage to 3.0V.
 */
static void gpio_output_voltage_setup(void) {
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
        }

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
        }

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
}
#endif

static void gpio3v3_init(void) {
#if defined(BOARD_PCA100593v3)
    // If nRF52 USB Dongle is powered from USB (high voltage mode),
    // GPIO output voltage is set to 1.8 V by default, which is not
    // enough to turn on green and blue LEDs. Therefore, GPIO voltage
    // needs to be increased to 3.0 V by configuring the UICR register.
    if (NRF_POWER->MAINREGSTATUS &
        (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos)) {
        gpio_output_voltage_setup();
    }
#endif
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const* p_evt) {
    ret_code_t err_code;
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id) {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        NRF_LOG_INFO("peers deleted");
        break;

    case PM_EVT_CONN_SEC_FAILED: {
        NRF_LOG_INFO("sec failed: %d", p_evt->params.conn_sec_failed.error);
        if (p_evt->params.conn_sec_failed.error == PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING) {
            // Rebond if one party has lost its keys.
            err_code = pm_conn_secure(p_evt->conn_handle, true);

            if (err_code != NRF_ERROR_BUSY) {
                APP_ERROR_CHECK(err_code);
            }
        }
    } break;

    default:
        break;
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void delete_bonds(void) {
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

static void tx_power_set(void) {
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0, 8);
    APP_ERROR_CHECK(err_code);
}

int main(void) {
    bool       erase_bonds;
    ret_code_t err;

    // Initialize.
    log_init();
    timer_init();
    gpio3v3_init();
    usb_init();
    leds_init();
    buttons_init(&erase_bonds);
    ble_stack_init();

    err = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    APP_ERROR_CHECK(err);
    err = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
    APP_ERROR_CHECK(err);

    gatt_init();
    db_discovery_init();
    sensor_c_init();
    ble_conn_state_init();
    peer_manager_init();
    scan_init(&m_scan, true);
    scan_init(&m_scan_noconnect, false);
    if (erase_bonds) {
        delete_bonds();
    }
    tx_power_set();

    ret_code_t ret;
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);

    // Start execution.
    NRF_LOG_INFO("Sensor central started.");
    scan_start(&m_scan);

    for (;;) {
        idle_state_handle();
    }
}
