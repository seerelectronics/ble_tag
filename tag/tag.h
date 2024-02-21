#ifndef BLE_SEER_TAG_H__
#define BLE_SEER_TAG_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif


#define BLE_SEER_TAG_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_seer_tag instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _seer_tag_max_clients Maximum number of SEER_TAG clients connected at a time.
 * @hideinitializer
 */
#define BLE_SEER_TAG_DEF(_name, _seer_tag_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_seer_tag_max_clients),                  \
                             sizeof(ble_seer_tag_client_context_t));   \
    static ble_seer_tag_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_SEER_TAG_BLE_OBSERVER_PRIO,               \
                         ble_seer_tag_on_ble_evt,                      \
                         &_name)

#define BLE_SEER_TAG_SERVICE_UUID 0x0001

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_SEER_TAG_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_SEER_TAG_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   Service event types. */
typedef enum
{
    BLE_SEER_TAG_EVT_ODR,
    BLE_SEER_TAG_EVT_FS,
    BLE_SEER_TAG_EVT_HP_FILTER,
    BLE_SEER_TAG_EVT_STEP_MINTHS,
    BLE_SEER_TAG_EVT_STEP_DEBOUNCE,
    BLE_SEER_TAG_EVT_STEP_DELTA,
    BLE_SEER_TAG_EVT_ODR_AVG,
    BLE_SEER_TAG_EVT_START_ACC_STREAM,
    BLE_SEER_TAG_EVT_STOP_ACC_STREAM,
    BLE_SEER_TAG_EVT_ACC_STREAM_READY,
    BLE_SEER_TAG_EVT_TX_POWER,
    BLE_SEER_TAG_EVT_CODED
} ble_seer_tag_evt_type_t;


/* Forward declaration of the ble_seer_tag_t type. */
typedef struct ble_seer_tag_s ble_seer_tag_t;


typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_seer_tag_evt_data_t;

/**@brief Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool sensor_data_notify;
    bool step_counter_notify;
    uint16_t last_step;
} ble_seer_tag_client_context_t;


/**@brief   Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_seer_tag_evt_type_t         type;        /**< Event type. */
    ble_seer_tag_t                * p_seer_tag;       /**< A pointer to the instance. */
    uint16_t                        conn_handle; /**< Connection handle. */
    ble_seer_tag_client_context_t * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_seer_tag_evt_data_t     data;
    } params;
} ble_seer_tag_evt_t;


/**@brief Service event handler type. */
typedef void (* ble_seer_tag_data_handler_t) (ble_seer_tag_evt_t * p_evt);


/**@brief   Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_seer_tag_init
 *          function.
 */
typedef struct
{
    ble_seer_tag_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_seer_tag_init_t;


/**@brief   Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_seer_tag_s
{
    uint8_t                         uuid_type;          /**< UUID type for Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        odr_char;
    ble_gatts_char_handles_t        sensor_read_char;
    ble_gatts_char_handles_t        step_minths_char;
    ble_gatts_char_handles_t        odr_avg_char;
    ble_gatts_char_handles_t        hp_filter_char;
    ble_gatts_char_handles_t        step_debounce_char;
    ble_gatts_char_handles_t        step_delta_char;
    ble_gatts_char_handles_t        step_counter_char;
    ble_gatts_char_handles_t        fs_char;
    ble_gatts_char_handles_t        tx_power_char;
    ble_gatts_char_handles_t        coded_char;
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    ble_seer_tag_data_handler_t     data_handler;  /**< Event handler to be called for handling received data. */
    int value;
};


/**@brief   Function for initializing the Service.
 *
 * @param[out] p_seer_tag      Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_seer_tag_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_seer_tag or p_seer_tag_init is NULL.
 */
uint32_t ble_seer_tag_init(ble_seer_tag_t * p_seer_tag, ble_seer_tag_init_t const * p_seer_tag_init);

uint32_t ble_seer_tag_update(ble_seer_tag_t * p_seer_tag, uint8_t *xyz, uint16_t size, uint16_t conn_handle);
uint32_t ble_seer_tag_update_step(ble_seer_tag_t * p_seer_tag, uint16_t steps, uint16_t conn_handle);

/**@brief   Function for handling the Service's BLE events.
 *
 * @details The Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Service structure.
 */
void ble_seer_tag_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_seer_tag       Pointer to the Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_seer_tag_data_send(ble_seer_tag_t * p_seer_tag,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_SEER_TAG_H__