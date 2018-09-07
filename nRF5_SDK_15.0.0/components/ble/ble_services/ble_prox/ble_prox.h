#ifndef BLE_PROX_H__
#define BLE_PROX_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_PROX_DEF(_name)                                                                          \
static ble_prox_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_prox_on_ble_evt, &_name)



// Proximo UUID BF05DEBC-2FE6-406E-9828-E1BD1A61054D
#define PROX_UUID_BASE         {0xBF, 0x05, 0xDE, 0xBC, 0x2F, 0xE6, 0x40, 0x6E, 0x98, 0x28, 0xE1, 0xBD, 0x1A, 0x61, 0x05, 0x4D}

#define PROX_SERVICE_UUID		0xC722
#define LED_CONFIG_UUID			0xC723
#define BUZZER_CONFIG_UUID		0xC724
#define ALARM_CONFIG_UUID		0xC725

/**@brief Custom Service event type. */
typedef enum
{
    BLE_PROX_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_PROX_EVT_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
    BLE_PROX_EVT_DISCONNECTED,
    BLE_PROX_EVT_CONNECTED
} ble_prox_evt_type_t;



/**@brief Proximo Service event. */
typedef struct
{
    ble_prox_evt_type_t evt_type;                                  /**< Type of event. */
} ble_prox_evt_t;

// Forward declaration of the ble_prox_t type.
typedef struct ble_prox_s ble_prox_t;


/**@brief Proximo Service event handler type. */
typedef void (*ble_prox_evt_handler_t) (ble_prox_t * p_prox, ble_prox_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_prox_evt_handler_t         evt_handler;                   /**< Event handler to be called for handling events in the Proximo Service. */
    uint8_t                       initial_custom_value;          /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Proximo characteristics attribute */
} ble_prox_init_t;

/**@brief Proximo Service structure. This contains various status information for the service. */
struct ble_prox_s
{
    ble_prox_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in the Proximo Service. */
    uint16_t                      service_handle;                 /**< Handle of Proximo Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      led_charr;			  /**< Handles related to the Master Slave Value characteristic. */
    ble_gatts_char_handles_t      buzzer_charr;			  /**< Handles related to the Serial Number Value characteristic. */
    ble_gatts_char_handles_t      alarm_charr;		  /**< Handles related to the Serial Number Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

/**@brief Function for initializing the Proximo Service.
 *
 * @param[out]  p_cus       Proximo Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_prox_init(ble_prox_t * p_prox, const ble_prox_init_t * p_prox_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_prox      Proximo Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_prox_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);



#endif // BLE_prox_H__
