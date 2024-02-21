/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
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
/**
 * @brief Seer tag
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "lis2ds12.h"
#include "tag.h"

#define BLE_SEER_TAG_SERVICE_UUID_TYPE  BLE_UUID_TYPE_VENDOR_BEGIN

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)      /**< The advertising interval */
#define APP_ADV_DURATION                MSEC_TO_UNITS(0, UNIT_10_MS)            /**< The advertising time-out. When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define NRF_CKAA 1

#if NRF_CKAA
#define LIS_INT1_PIN 5
#define LIS_INT2_PIN 4
#else
#define LIS_INT1_PIN 41
#define LIS_INT2_PIN 12
#endif

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); 

BLE_SEER_TAG_DEF(m_seer_tag, NRF_SDH_BLE_TOTAL_LINK_COUNT);

ble_uuid_t m_adv_uuids[] = {{BLE_SEER_TAG_SERVICE_UUID, BLE_SEER_TAG_SERVICE_UUID_TYPE}};
ble_advdata_t m_adv_data;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

#define SPI_INSTANCE 0
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static bool m_acc_stream = false;
static bool m_acc_ready = true;
static bool m_fifo_full = true;

static uint32_t m_lis_value;
static uint8_t m_lis_event;
static bool m_lis_update = false;
static uint16_t m_steps = 0;
static uint16_t m_last_steps = 0xffff;
static uint8_t m_avg = 0;

static ble_advdata_manuf_data_t manuf_data;
static uint8_t manufacturing_data_payload[4];

static uint8_t m_coded = 0;

LIS2DS12_INSTANCE_DEF(m_lis, &spi);

#undef APP_ERROR_CHECK
#define APP_ERROR_CHECK(code) if (code) { NRF_LOG_ERROR("Error %x line %d\n", code, __LINE__); }

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

bool clear_acc_flag = false;

static void clear_acc()
{
    if (!clear_acc_flag) {
      return;
    }

    clear_acc_flag = false;

    int16_t acc_xyz[3 * 256];
    uint16_t size = lis2ds12_get_fifo_level(&m_lis);
    if (size > 255)
      size = 255;
    lis2ds12_get_acc_fifo(&m_lis, acc_xyz, size, m_avg);
}

bool update_step_flag = false;

static void update_step()
{
    if (!update_step_flag) {
      return;
    }
    update_step_flag = false;

    uint16_t step = lis2ds12_get_step_counter(&m_lis);
    NRF_LOG_INFO("step %u", step);
    m_steps = step;

    ret_code_t err_code;
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        err_code = ble_seer_tag_update_step(&m_seer_tag, m_steps, m_conn_handle);
        APP_ERROR_CHECK(err_code);
    }

    if (m_last_steps != m_steps) {
        m_adv_data.p_manuf_specific_data = &manuf_data;

        manuf_data.company_identifier = 0x6666;
        manufacturing_data_payload[0] = m_steps & 0xff;
        manufacturing_data_payload[1] = m_steps >> 8;
        manuf_data.data.p_data = manufacturing_data_payload;
        manuf_data.data.size = 4;
    
        NRF_LOG_INFO("adv update");
        err_code = ble_advertising_advdata_update(&m_advertising, &m_adv_data, NULL);
        APP_ERROR_CHECK(err_code);

        m_last_steps = m_steps;
    }
}

static void compress_acc(uint8_t *data, uint8_t slen, uint16_t size, uint16_t *out_size)
{
    *out_size = 0;
    uint8_t *out = data;
    for (int i = 0; i < size; ++i) {
        uint8_t xl = *data++;
        uint8_t xh = *data++;
        uint8_t yl = *data++;
        uint8_t yh = *data++;
        uint8_t zl = *data++;
        uint8_t zh = *data++;
        if (slen == 10) {
            *out++ = xl;
            *out++ = ((xh & 0x03) << 6) | ((yl >> 2) & 0x3f);
            *out++ = ((yl & 0x03) << 6) | ((yh & 0x3) << 4) | ((zl >> 4) & 0x0f);
            *out++ = ((zl & 0x0f) << 4) | (zh & 0x3) << 2;
            *out_size += 4;
        }
        else if (slen == 12) {
            *out++ = xl;
            *out++ = ((xh & 0x0f) << 4) | ((yl >> 4) & 0x0f);
            *out++ = ((yl & 0x0f) << 4) | (yh & 0xf);
            *out++ = zl;
            *out++ = (zh & 0x0f) << 4;
        }
    }
}

#if 0
static int16_t ext(uint8_t l, uint8_t h)
{
  if (h & 2) {
    h |= 0xfc;
  }
  return (int16_t)((h << 8) | l);
}

static void decomp(int16_t x, int16_t y, int16_t z, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  uint8_t xl = a;
  uint8_t xh = b >> 6;
  uint8_t yl = ((b & 0x3f) << 2) | (c >> 6);
  uint8_t yh = (c >> 4) & 0x3;
  uint8_t zl = ((c & 0xf) << 4) | (d >> 4);
  uint8_t zh = (d >> 2) & 0x3;
  int16_t x1 = ext(xl, xh);
  int16_t y1 = ext(yl, yh);
  int16_t z1 = ext(zl, zh);
  if (x != x1 || y != y1 || z != z1) {
    NRF_LOG_INFO("!!!decomp %d=%d %d=%d %d=%d", x, x1, y, y1, z, z1);
    NRF_LOG_INFO("%x %x %x", x, y, z);
    NRF_LOG_INFO("%x%x%x%x", a, b, c, d);
  }
}
#endif

#define BUFFER_SIZE (60 * 4)
#define BUFFER_SLOTS 5
static uint8_t buffer[BUFFER_SIZE * BUFFER_SLOTS];
static int buffer_slots = 0;

static void update_acc()
{
    if (!m_fifo_full || !m_acc_stream) {
      return;
    }
    m_fifo_full = false;

    ret_code_t err_code;
    int16_t acc_xyz[3 * 256];
    uint16_t size = lis2ds12_get_fifo_level(&m_lis);
    bool skip = false;
    if (size > 60) {
        skip = true;
    }
    if (size > 255) {
        NRF_LOG_ERROR("FIFO overflow");
        size = 255;
    }
    lis2ds12_get_acc_fifo(&m_lis, acc_xyz, size, m_avg);
    if (!skip && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        uint16_t out_size;
        uint8_t *comp = (uint8_t*)acc_xyz;
        compress_acc(comp, 10, size, &out_size);
        if (!m_acc_ready) {
            NRF_LOG_INFO("BLE add buffer, slots %d", buffer_slots);
            if (buffer_slots >= BUFFER_SLOTS - 1) {
                NRF_LOG_ERROR("BLE add overflow");
                buffer_slots = 0;
                m_acc_ready = true;
            }
            memcpy(&buffer[BUFFER_SIZE * buffer_slots], comp, BUFFER_SIZE);
            ++buffer_slots;
            return;
        }
        m_acc_ready = false;
        err_code = ble_seer_tag_update(&m_seer_tag, comp, out_size, m_conn_handle);
        APP_ERROR_CHECK(err_code);
    }
}

APP_TIMER_DEF(m_timer);
#define TIMER_INTERVAL APP_TIMER_TICKS(30000L)

static void timer_handler(void * p_context)
{
    /*  static uint8_t x = 0;
    NRF_LOG_INFO("timer %d", x++);
  
    static uint8_t odr = 8;
    ++odr;
    if (odr == 12) {
        odr = 8;
    }
    m_lis_value = odr;
    m_lis_event = BLE_SEER_TAG_EVT_ODR;
    m_lis_update = true;
    */
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer,
                               APP_TIMER_MODE_REPEATED,
                               timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer,
                               TIMER_INTERVAL,
                               NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    static char name_buffer[32];
    memset(name_buffer, 0, 32);
    sprintf(name_buffer, "PN_%08X", NRF_FICR->DEVICEID[0]);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          name_buffer,
                                          strlen(name_buffer));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    NRF_LOG_INFO("SLEEP");

    //nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);

    //uint32_t err_code = sd_power_system_off();
    //APP_ERROR_CHECK(err_code);
}

static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;


    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertisement");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("IDLE");
            sleep_mode_enter();
            break;
        default:
            NRF_LOG_INFO("adv event %x", ble_adv_evt);
            break;
    }
}
static void on_adv_error(uint32_t nrf_error)
{
    NRF_LOG_INFO("error %x", nrf_error);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    // srdata
    //init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memcpy(&m_adv_data, &init.advdata, sizeof(m_adv_data));

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.config.ble_adv_on_disconnect_disabled = false;
    init.evt_handler = on_adv_evt;
    init.error_handler = on_adv_error;

    if (m_coded) {
        init.config.ble_adv_primary_phy = BLE_GAP_PHY_1MBPS;
        init.config.ble_adv_secondary_phy = BLE_GAP_PHY_1MBPS;
        init.config.ble_adv_extended_enabled = true;
    }
    else {
        init.config.ble_adv_primary_phy = BLE_GAP_PHY_1MBPS;
        init.config.ble_adv_secondary_phy = BLE_GAP_PHY_1MBPS;
    }

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void update_lis()
{
    if (!m_lis_update) {
        return;
    }

    lis2ds12_odr_t odr;

    switch (m_lis_event) {
    case BLE_SEER_TAG_EVT_ODR:
        NRF_LOG_INFO("ODR %x", m_lis_value);
        lis2ds12_set_odr(&m_lis, m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_STEP_MINTHS:
        NRF_LOG_INFO("MINTH %x", m_lis_value);
        lis2ds12_set_step_minths(&m_lis, m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_HP_FILTER:
        NRF_LOG_INFO("HP %x", m_lis_value);
        lis2ds12_set_hp_filter(&m_lis, m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_STEP_DEBOUNCE:
        NRF_LOG_INFO("STEP %x", m_lis_value);
        lis2ds12_set_step_debounce(&m_lis, m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_STEP_DELTA:
        NRF_LOG_INFO("STEPD %x", m_lis_value);
        lis2ds12_set_step_delta(&m_lis, m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_FS:
        NRF_LOG_INFO("FS %x", m_lis_value);
        lis2ds12_set_fs(&m_lis, m_lis_value);
        break;
    }

    m_lis_update = false;
}

static void advertising_start(void);

static void seer_tag_data_handler(ble_seer_tag_evt_t * p_evt)
{
    if (p_evt->params.data.length == 1) {
        m_lis_value = p_evt->params.data.p_data[0];
    }
    if (p_evt->params.data.length == 4) {
        m_lis_value = *(uint32_t*)p_evt->params.data.p_data;
    }

    switch (p_evt->type) {
    case BLE_SEER_TAG_EVT_ODR_AVG:
        m_avg = m_lis_value;
        break;
    case BLE_SEER_TAG_EVT_ACC_STREAM_READY:
        m_acc_ready = true;
        if (buffer_slots > 0 && m_acc_stream && m_conn_handle != BLE_CONN_HANDLE_INVALID) {
            NRF_LOG_INFO("SEND slots: %d", buffer_slots);
            --buffer_slots;
            m_acc_ready = false;
            ble_seer_tag_update(&m_seer_tag, buffer, BUFFER_SIZE, m_conn_handle);
            memmove(buffer, buffer + BUFFER_SIZE, BUFFER_SIZE * buffer_slots);
        }
        break;
    case BLE_SEER_TAG_EVT_START_ACC_STREAM:
        m_acc_stream = true;
        NRF_LOG_INFO("START");
        clear_acc_flag = true;
        nrf_drv_gpiote_in_event_enable(LIS_INT1_PIN, true);
        break;
    case BLE_SEER_TAG_EVT_STOP_ACC_STREAM:
        NRF_LOG_INFO("STOP");
        m_acc_stream = false;
        buffer_slots = 0;
        nrf_drv_gpiote_in_event_disable(LIS_INT1_PIN);
        break;
    case BLE_SEER_TAG_EVT_TX_POWER:
        NRF_LOG_INFO("Set power %d", (int8_t)m_lis_value);
        sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle,
                                (int8_t)m_lis_value);
        break;
    case BLE_SEER_TAG_EVT_CODED:
        NRF_LOG_INFO("Set coded %d", m_lis_value);
        /*
        m_coded = m_lis_value;
        if (m_coded == 1) {
            m_advertising.adv_modes_config.ble_adv_primary_phy = BLE_GAP_PHY_CODED;
            m_advertising.adv_modes_config.ble_adv_secondary_phy = BLE_GAP_PHY_CODED;
            m_advertising.adv_modes_config.ble_adv_extended_enabled = true;
        }
        else {
            m_advertising.adv_modes_config.ble_adv_primary_phy = BLE_GAP_PHY_1MBPS;
            m_advertising.adv_modes_config.ble_adv_secondary_phy = BLE_GAP_PHY_1MBPS;
            m_advertising.adv_modes_config.ble_adv_extended_enabled = false;
        }
        */
        m_lis_event = BLE_SEER_TAG_EVT_ODR;
        m_lis_update = true;
        break;
    default:
        m_lis_event = p_evt->type;
        m_lis_update = true;
        break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t          err_code;
    ble_seer_tag_init_t seer_tag_init = {0};
    nrf_ble_qwr_init_t  qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    seer_tag_init.data_handler = seer_tag_data_handler;

    err_code = ble_seer_tag_init(&m_seer_tag, &seer_tag_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        NRF_LOG_INFO("CONN fail disconnect");
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            //advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            NRF_LOG_DEBUG("Event: %d", p_ble_evt->header.evt_id);
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    ble_cfg_t ble_cfg;
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ble_cfg.gap_cfg.device_name_cfg.write_perm);
    ble_cfg.gap_cfg.device_name_cfg.vloc        = BLE_GATTS_VLOC_STACK;
    ble_cfg.gap_cfg.device_name_cfg.p_value     = NULL;
    ble_cfg.gap_cfg.device_name_cfg.current_len = 0;
    ble_cfg.gap_cfg.device_name_cfg.max_len     = 32;

    err_code = sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    nrf_power_dcdcen_vddh_set(true);
    nrf_power_dcdcen_set(true);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static int spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
#if NRF_CKAA
    spi_config.ss_pin   = 32+9;  //P1.09
    spi_config.sck_pin  = 8;     //P0.08
    spi_config.miso_pin = 12;    //P0.12
    spi_config.mosi_pin = 32+8;  //P1.08
#else
    spi_config.ss_pin   = 6;
    spi_config.sck_pin  = 8;
    spi_config.miso_pin = 4;
    spi_config.mosi_pin = 26;
#endif
    spi_config.mode = NRF_DRV_SPI_MODE_3;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;

    // NOTE 200ms delay on SS in nrfx_spim.c must be removed
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

static int lis_init(void)
{
    APP_ERROR_CHECK(lis2ds12_init(&m_lis));
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == LIS_INT1_PIN) {
        m_fifo_full = true;
    }
    else if (pin == LIS_INT2_PIN) {
        update_step_flag = true;
    }
}

static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    err_code = nrf_drv_gpiote_in_init(LIS_INT1_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_in_init(LIS_INT2_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(LIS_INT2_PIN, true);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    lfclk_config();
    gpio_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    spi_init();
    lis_init();
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
        clear_acc();
        update_acc();
        update_step();
        update_lis();
    }
}


/**
 * @}
 */
