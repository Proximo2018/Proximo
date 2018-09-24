#include <stdio.h>
#include "sdk_common.h"
#include "ble_prox.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "event.h"
#include "app_util.h"




/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_prox       Proximo Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_prox_t * p_prox, ble_evt_t const * p_ble_evt)
{
    p_prox->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_prox_evt_t evt;

    evt.evt_type = BLE_PROX_EVT_CONNECTED;

    p_prox->evt_handler(p_prox, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_prox       Proximo Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_prox_t * p_prox, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_prox->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_prox_evt_t evt;

    evt.evt_type = BLE_PROX_EVT_DISCONNECTED;

    p_prox->evt_handler(p_prox, &evt);
}



static void on_write_authorize_request(ble_prox_t * p_prox, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_attr_t			  attr_char_value;
    ble_gatts_value_t			  gatts_value;
    uint32_t                      err_code = NRF_ERROR_INVALID_DATA;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_gatts_evt->params.authorize_request;
    ble_gatts_evt_write_t const * p_evt_write = &p_auth_req->request.write;

    /* Only handle Authorize Write type events further */
    if (p_auth_req->type != BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
      return;
    }
    
    /* Check the opcode */
    if( (p_evt_write->op == BLE_GATTS_OP_PREP_WRITE_REQ) && (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) && (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL) )
    { 
      return;
    }

    /* Create a default GATT reply message */
    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
    auth_reply.params.write.update      = 1;

    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
    switch(p_evt_write->uuid.uuid)
    {
      case LED_CONFIG_UUID:
      {
        if(p_evt_write->len == LED_PARAM_LENGHT && p_evt_write->handle == p_prox->led_charr.value_handle)
        {
          uint8_t   Green       = p_evt_write->data[0];
          uint8_t   Red         = p_evt_write->data[1];
          uint8_t   Blue        = p_evt_write->data[2];
          uint16_t  on_time     = (uint16_t)p_evt_write->data[3] * 100;
          uint16_t  off_time    = (uint16_t)p_evt_write->data[4] * 100;
          uint16_t  blink_count = p_evt_write->data[5];

          NRF_LOG_INFO("Led Colour: G:0x%02X R:0x%02X B:0x%02X, On:%u, Off:%u, Repeat: %u",
              Green, Red, Blue,
              on_time,
              off_time,
              blink_count);

	  /* BLE test commands
	  * Green FF0000006400640A
	  * Red	  00FF00006400640A
	  * Blue  0000FF006400640A
	  */
          if(sk6812_blink_event(Green, Red, Blue, on_time, off_time, blink_count))
          {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
          }
        }
      }
        break;
      case BUZZER_CONFIG_UUID:
      {
        if(p_evt_write->len == BUZZ_PARAM_LENGHT && p_evt_write->handle == p_prox->buzzer_charr.value_handle)
        {
          uint16_t  frequency	  = ((uint16_t) p_evt_write->data[0]) * 100;
          uint8_t   dutycycle	  = p_evt_write->data[1];
          uint16_t  on_time       = ((uint16_t) p_evt_write->data[2]) * 100;
          uint16_t  off_time	  = ((uint16_t) p_evt_write->data[3]) * 100;
          uint8_t   repeat        = p_evt_write->data[4];

          NRF_LOG_INFO("Buzz Freq: %u, D:%u, On:%u, Off:%u, Repeat: %u",
              frequency,
              dutycycle,
              on_time,
              off_time,
              repeat);

          if(buzz_event(frequency, dutycycle, on_time, off_time, repeat))
          {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
          }
        }
      }
        break;

      case ALARM_CONFIG_UUID:
      {
        if(p_evt_write->len == ALARM_PARAM_LENGHT && p_evt_write->handle == p_prox->alarm_charr.value_handle)
        {
          uint16_t on_time  = ((uint16_t) p_evt_write->data[0]) * 100;
          uint16_t off_time = ((uint16_t) p_evt_write->data[1]) * 100;
          uint8_t repeat    = p_evt_write->data[2];

          NRF_LOG_INFO("Alarm On:%u, Off:%u, Repeat: %u",
              on_time,
              off_time,
              repeat);

          if(alarm_blink(on_time, off_time, repeat))
          {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
          }
        }
      }
        break;
      case PROX_SERVICE_UUID:
      default:
        return;
        break;
    }

    #ifndef DEBUG_PROX

      err_code = sd_ble_gatts_rw_authorize_reply(p_prox->conn_handle, &auth_reply);

    #else

      // Print unsuccessfull writes
      if(auth_reply.params.write.gatt_status == BLE_GATT_STATUS_SUCCESS)
      {
        NRF_LOG_INFO("Write authorized");
      }
      else
      {
        NRF_LOG_INFO("Write not authorized, status: %u", auth_reply.params.write.gatt_status);
      }

      err_code = sd_ble_gatts_rw_authorize_reply(p_prox->conn_handle, &auth_reply);
      if (err_code != NRF_SUCCESS)
      {
	    NRF_LOG_INFO("Authrorize reply error: %x", err_code);
      }
    #endif

}





void ble_prox_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_prox_t * p_prox = (ble_prox_t *) p_context;

    #if 0
      NRF_LOG_INFO("Prox BLE Event %u", (uint32_t) p_ble_evt->header.evt_id);
    #endif
    
    if (p_prox == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_prox, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_prox, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_write_authorize_request(p_prox, &p_ble_evt->evt.gatts_evt);
            break;

        default:
            // No implementation needed.
            break;
    }

    
}

/**@brief Function for adding the Proximo Value characteristic.
 *
 * @param[in]   p_prox        Battery Service structure.
 * @param[in]   p_prox_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t led_charr_add(ble_prox_t * p_prox, const ble_prox_init_t * p_prox_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Proximo Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_prox_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_prox->uuid_type;
    ble_uuid.uuid = LED_CONFIG_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_prox_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_prox_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 1;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = LED_PARAM_LENGHT;

    err_code = sd_ble_gatts_characteristic_add(p_prox->service_handle, &char_md, &attr_char_value, &p_prox->led_charr);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Proximo Value characteristic.
 *
 * @param[in]   p_prox        Battery Service structure.
 * @param[in]   p_prox_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t alarm_charr_add(ble_prox_t * p_prox, const ble_prox_init_t * p_prox_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Proximo Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_prox_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_prox->uuid_type;
    ble_uuid.uuid = ALARM_CONFIG_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_prox_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_prox_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 1;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = ALARM_PARAM_LENGHT;

    err_code = sd_ble_gatts_characteristic_add(p_prox->service_handle, &char_md, &attr_char_value, &p_prox->alarm_charr);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}




/**@brief Function for adding the Proximo Value characteristic.
 *
 * @param[in]   p_prox        Proximo Service structure.
 * @param[in]   p_prox_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t buzzer_charr_add(ble_prox_t * p_prox, const ble_prox_init_t * p_prox_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Add Proximo Value characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.write_perm = p_prox_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    ble_uuid.type = p_prox->uuid_type;
    ble_uuid.uuid = BUZZER_CONFIG_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_prox_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_prox_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 1;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BUZZ_PARAM_LENGHT;

    err_code = sd_ble_gatts_characteristic_add(p_prox->service_handle, &char_md, &attr_char_value, &p_prox->buzzer_charr);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_prox_init(ble_prox_t * p_prox, const ble_prox_init_t * p_prox_init)
{
    if (p_prox == NULL || p_prox_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_prox->evt_handler               = p_prox_init->evt_handler;
    p_prox->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Proximo Service UUID
    ble_uuid128_t base_uuid = {PROX_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_prox->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_prox->uuid_type;
    ble_uuid.uuid = PROX_SERVICE_UUID;

    // Add the Proximo Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_prox->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* Add the LED characteristics */
    err_code = led_charr_add(p_prox, p_prox_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* Add the buzzer event configuration */
    err_code = buzzer_charr_add(p_prox, p_prox_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* Add the pedal offset field */
    err_code = alarm_charr_add(p_prox, p_prox_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}


