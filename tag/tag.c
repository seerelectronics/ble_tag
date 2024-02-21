#include "nrf_log.h"
#include "tag.h"
#include <stdio.h>

#define RETURN_IF_ERR(err)  \
    if (err != NRF_SUCCESS) \
    {                       \
        return err;         \
    }

#define BLE_UUID_SENSOR_READ_CHARACTERISTIC 0x0002
#define BLE_UUID_STEP_COUNTER_CHARACTERISTIC 0x0003
#define BLE_UUID_ODR_CHARACTERISTIC 0x0004
#define BLE_UUID_ODR_AVG_CHARACTERISTIC 0x0005
#define BLE_UUID_FS_CHARACTERISTIC 0x0006
#define BLE_UUID_HP_FILTER_CHARACTERISTIC 0x0007
#define BLE_UUID_STEP_MINTHS_CHARACTERISTIC 0x0008
#define BLE_UUID_STEP_DEBOUNCE_CHARACTERISTIC 0x0009
#define BLE_UUID_STEP_DELTA_CHARACTERISTIC 0x000A
#define BLE_UUID_TX_POWER_CHARACTERISTIC 0x000B
#define BLE_UUID_CODED_CHARACTERISTIC 0x000C

#define BLE_SEER_BASE_UUID  {{ 0x67, 0x0c, 0xd7, 0x78, 0xca, 0xbf, 0x7e, 0xaf, 0x97, 0x43, 0x34, 0x83, 0x00, 0x00, 0x00, 0x00   }}

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_seer_tag_t * p_seer_tag, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_seer_tag_evt_t         evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_seer_tag_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_seer_tag->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the sensor data characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_seer_tag->sensor_read_char.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->sensor_data_notify = true;
        }

        if (p_seer_tag->data_handler != NULL) {
            memset(&evt, 0, sizeof(ble_seer_tag_evt_t));
            evt.type        = BLE_SEER_TAG_EVT_START_ACC_STREAM;
            evt.p_seer_tag  = p_seer_tag;
            evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            evt.p_link_ctx  = p_client;

            p_seer_tag->data_handler(&evt);
        }
    }

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_seer_tag->step_counter_char.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->step_counter_notify = true;
            p_client->last_step = 0xffff;
        }
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 */
static void on_write(ble_seer_tag_t * p_seer_tag, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_seer_tag_evt_t            evt;
    ble_seer_tag_client_context_t * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_seer_tag->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    if (p_seer_tag->data_handler == NULL) {
        return;
    }

    memset(&evt, 0, sizeof(ble_seer_tag_evt_t));
    evt.p_seer_tag  = p_seer_tag;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;
    evt.params.data.p_data = p_evt_write->data;
    evt.params.data.length = p_evt_write->len;

    if ((p_evt_write->handle == p_seer_tag->sensor_read_char.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->sensor_data_notify = true;
                evt.type = BLE_SEER_TAG_EVT_START_ACC_STREAM;
            }
            else
            {
                p_client->sensor_data_notify = false;
                evt.type = BLE_SEER_TAG_EVT_STOP_ACC_STREAM;
            }

            p_seer_tag->data_handler(&evt);
        }
    }
    else if ((p_evt_write->handle == p_seer_tag->step_counter_char.cccd_handle) &&
             (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->step_counter_notify = true;
                p_client->last_step = 0xffff;
            }
            else
            {
                p_client->step_counter_notify = false;
            }
        }
    }
    else if (p_evt_write->handle == p_seer_tag->odr_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_ODR;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->step_minths_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_STEP_MINTHS;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->hp_filter_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_HP_FILTER;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->odr_avg_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_ODR_AVG;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->fs_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_FS;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->step_debounce_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_STEP_DEBOUNCE;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->step_delta_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_STEP_DELTA;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->tx_power_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_TX_POWER;
        p_seer_tag->data_handler(&evt);
    }
    else if (p_evt_write->handle == p_seer_tag->coded_char.value_handle)
    {
        evt.type = BLE_SEER_TAG_EVT_CODED;
        p_seer_tag->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 */
static void on_hvx_tx_complete(ble_seer_tag_t * p_seer_tag, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_seer_tag_evt_t         evt;
    ble_seer_tag_client_context_t * p_client;

    err_code = blcm_link_ctx_get(p_seer_tag->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
        return;
    }

    if (p_client->sensor_data_notify && (p_seer_tag->data_handler != NULL))
    {
        memset(&evt, 0, sizeof(ble_seer_tag_evt_t));
        evt.type        = BLE_SEER_TAG_EVT_ACC_STREAM_READY;
        evt.p_seer_tag  = p_seer_tag;
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_seer_tag->data_handler(&evt);
    }
}


void ble_seer_tag_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_seer_tag_t * p_seer_tag = (ble_seer_tag_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_seer_tag, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_seer_tag, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_seer_tag, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_seer_tag_init(ble_seer_tag_t * p_seer_tag, ble_seer_tag_init_t const * p_seer_tag_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         base_uuid = BLE_SEER_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_seer_tag);
    VERIFY_PARAM_NOT_NULL(p_seer_tag_init);

    // Initialize the service structure.
    p_seer_tag->data_handler = p_seer_tag_init->data_handler;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_seer_tag->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_seer_tag->uuid_type;
    ble_uuid.uuid = BLE_SEER_TAG_SERVICE_UUID;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_seer_tag->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the LIS2DS12 setup characteristics.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BLE_UUID_STEP_MINTHS_CHARACTERISTIC;
    add_char_params.uuid_type                = p_seer_tag->uuid_type;
    add_char_params.max_len                  = 1;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = false;
    //add_char_params.char_props.read          = 1;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    add_char_params.uuid = BLE_UUID_STEP_MINTHS_CHARACTERISTIC;
    p_seer_tag->value = 0x10;
    add_char_params.p_init_value = &p_seer_tag->value;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->step_minths_char));

    add_char_params.uuid = BLE_UUID_HP_FILTER_CHARACTERISTIC;
    p_seer_tag->value = 0x00;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->hp_filter_char));

    add_char_params.uuid = BLE_UUID_STEP_DEBOUNCE_CHARACTERISTIC;
    p_seer_tag->value = 0x6e;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->step_debounce_char));

    add_char_params.uuid = BLE_UUID_STEP_DELTA_CHARACTERISTIC;
    p_seer_tag->value = 0x00;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->step_delta_char));

    add_char_params.uuid = BLE_UUID_FS_CHARACTERISTIC;
    p_seer_tag->value = 0;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->fs_char));

    add_char_params.uuid = BLE_UUID_TX_POWER_CHARACTERISTIC;
    p_seer_tag->value = 8;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->tx_power_char));

    add_char_params.uuid = BLE_UUID_CODED_CHARACTERISTIC;
    p_seer_tag->value = 0;
    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->coded_char));

    // Add the sensor read characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_SENSOR_READ_CHARACTERISTIC;
    add_char_params.uuid_type         = p_seer_tag->uuid_type;
    add_char_params.max_len           = BLE_SEER_TAG_MAX_DATA_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    RETURN_IF_ERR(characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->sensor_read_char));

    add_char_params.uuid  = BLE_UUID_STEP_COUNTER_CHARACTERISTIC;
    return characteristic_add(p_seer_tag->service_handle, &add_char_params, &p_seer_tag->step_counter_char);
}

static uint32_t data_send(ble_seer_tag_t * p_seer_tag,
                          uint8_t   * p_data,
                          uint16_t  * p_length,
                          uint16_t    conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_seer_tag_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_seer_tag);

    err_code = blcm_link_ctx_get(p_seer_tag->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->sensor_data_notify)
    {
        return NRF_SUCCESS;
    }

    if (*p_length > BLE_SEER_TAG_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_seer_tag->sensor_read_char.value_handle;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}

uint32_t ble_seer_tag_update(ble_seer_tag_t * p_seer_tag, uint8_t *xyz, uint16_t size, uint16_t conn_handle)
{
    return data_send(p_seer_tag, xyz, &size, conn_handle);
}


uint32_t ble_seer_tag_update_step(ble_seer_tag_t * p_seer_tag, uint16_t step, uint16_t conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_seer_tag_client_context_t * p_client;
    uint16_t length = 2;
    uint8_t data[2];

    err_code = blcm_link_ctx_get(p_seer_tag->p_link_ctx_storage, conn_handle, (void *) &p_client);
    VERIFY_SUCCESS(err_code);

    if (!p_client->step_counter_notify)
    {
        return NRF_SUCCESS;
    }

    if (p_client->last_step == step)
    {
        return NRF_SUCCESS;
    }
    p_client->last_step = step;

    data[0] = step & 0xff;
    data[1] = (step >> 8) & 0xff;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_seer_tag->step_counter_char.value_handle;
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}
