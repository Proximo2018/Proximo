#include "ble_setup.h"
#include "ble_dfu.h"
#include "nrf_bootloader_info.h"
#include "io.h"
#include "ble_prox.h"
#include "ble_conn_state.h" 
#include "event.h"


BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
BLE_PROX_DEF(m_prox);                                               /**< Structure used to identify the Proximo Configuration service. */      
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static ble_gap_adv_params_t m_adv_params; 
static pm_peer_id_t	  m_peer_id;                                            /**< Device reference handle to the current bonded central. */
static pm_peer_id_t	  m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];  /**< List of peers currently in the whitelist. */
static uint32_t		  m_whitelist_peer_cnt;                                 /**< Number of peers currently in the whitelist. */
static sensorsim_cfg_t	  m_battery_sim_cfg;					/**< Battery Level sensor simulator configuration. */
static sensorsim_state_t  m_battery_sim_state;					/**< Battery Level sensor simulator state. */


static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static volatile bool advertising_on = false;


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

void button_adv_start(void)
{
//  advertising_stop();
  advertising_start();
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    if(advertising_on)
    {
      err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
      if(err_code == NRF_SUCCESS)
      {
        advertising_on = false;
      }
    }

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    if(err_code == NRF_SUCCESS)
    {
      advertising_on = true;
    }
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Advertising started");
}

void advertising_stop(void)
{
    ret_code_t err_code;

    if(advertising_on)
    {
      err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
      if(err_code == NRF_SUCCESS)
      {
        advertising_on = false;
      }
      APP_ERROR_CHECK(err_code);
      NRF_LOG_INFO("Advertising stopped");
    }
    else
    {
      NRF_LOG_INFO("Device isn't advertising");
    }
}

static void advertising_restart (void)
{
  ret_code_t err_code;
  uint32_t per_count = ble_conn_state_peripheral_conn_count();
  

  if(per_count < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
  {
      advertising_start();
      NRF_LOG_INFO("Advertising Restarted, current count %u", per_count);
  }
  else
  {
      NRF_LOG_INFO("Max link count reached %u, advertising not restarted", per_count);
  }
}

static void advertising_check_peer_count (void)
{
    /* When no smartphone has connected and been whitelisted, stop advertising until the user presses the activation button. */
    if(m_whitelist_peer_cnt == 0)
    {
	advertising_stop();
    }
}


void enter_bootloader (void)
{
  ret_code_t err_code;

  err_code = sd_power_gpregret_clr(0, 0xffffffff);
  APP_ERROR_CHECK(err_code);

  err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Entering bootloader");
  NRF_LOG_FLUSH();

  NVIC_SystemReset();
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ? *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    NRF_LOG_INFO("Peer count: %u", peers_to_copy);

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
	NRF_LOG_INFO("Peer:%u, uuid: %u", peers_to_copy, peer_id);
    } 
}

static void peer_clear_list (pm_peer_id_t * p_peers, uint32_t * p_size)
{
  if(p_peers == NULL || p_size == NULL)
  {
    return;
  }

  for(uint8_t i = 0; i < BLE_GAP_WHITELIST_ADDR_MAX_COUNT; i++)
  {
      p_peers[i] = PM_PEER_ID_INVALID;
  }

  *p_size = 0;
}

void peer_list_load(void)
{
    ret_code_t err_code;

    memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
    m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

    peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
    APP_ERROR_CHECK(err_code);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
    if (err_code != NRF_ERROR_NOT_SUPPORTED)
    {
	APP_ERROR_CHECK(err_code);
    }
}



/**@brief Clear bond information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;

    advertising_stop();

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    NRF_LOG_INFO("Delete bonds Error_code: %u/%04X", err_code, err_code);
    APP_ERROR_CHECK(err_code);

    // Retrieve the cleared whitelist
    m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));
    peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

    advertising_start();
}

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_prox_evt(ble_prox_t * p_prox_service, ble_prox_evt_t * p_evt)
{
    ret_code_t err_code;

    NRF_LOG_INFO("on_prox_event");
    
    switch(p_evt->evt_type)
    {
        case BLE_PROX_EVT_NOTIFICATION_ENABLED:
          break;

        case BLE_PROX_EVT_NOTIFICATION_DISABLED:
            break;

        case BLE_PROX_EVT_CONNECTED:
            break;

        case BLE_PROX_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}








/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed\n");
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    #if 0
      NRF_LOG_INFO("PM event: %u", (uint32_t) p_evt->evt_id);
    #endif

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
	  #if 1
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
	    
	  #endif
	  m_peer_id = p_evt->peer_id;
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            // Note: You should check on what kind of white list policy your application should use.
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
		advertising_stop();

                #ifdef APP_DEVELOPMENT
		  sk6812_blink_event(SK6812_WHITE, 200, 200, 10);
		#endif

                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d", m_whitelist_peer_cnt, BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt] = m_peer_id;

                    m_whitelist_peer_cnt += 1;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }

                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);

                    advertising_restart();
                }
		else
		{
		    NRF_LOG_INFO("Adding the peer was not possible, maximum peers reached");
		}
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_SUCCEEDED");
            peer_clear_list (m_whitelist_peers, &m_whitelist_peer_cnt);
            advertising_start();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
void battery_level_update(uint8_t battery_level)
{
    ret_code_t err_code;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}




/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }
}


/**@brief Function for initializing the GATT module.
 */
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
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

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
void services_init(void)
{
    ret_code_t         err_code;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_prox_init_t     prox_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);


    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    /* Initializing Proximo Service */
    memset(&prox_init, 0, sizeof(prox_init));
    prox_init.evt_handler = on_prox_evt;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&prox_init.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&prox_init.custom_value_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&prox_init.custom_value_char_attr_md.read_perm);

    err_code = ble_prox_init(&m_prox, &prox_init);
    APP_ERROR_CHECK(err_code);  
}




/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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
void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}





/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
	  NRF_LOG_INFO("Fast advertising.");
	  break;
	case BLE_ADV_EVT_SLOW:
	  NRF_LOG_INFO("Slow advertising.");
	  advertising_check_peer_count();
	  break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        default:
            break;
    }
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    #if 0
      NRF_LOG_INFO("ble event: %u", (uint32_t)p_ble_evt->header.evt_id);
    #endif

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");

	    #ifdef APP_DEVELOPMENT
	      sk6812_blink_event(SK6812_YELLOW, 200, 200, 10);
	    #endif
            
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            advertising_restart();
            break;

        case BLE_GAP_EVT_DISCONNECTED:

	    #ifdef APP_DEVELOPMENT
	      sk6812_blink_event(SK6812_PURPLE, 200, 200, 10);
	    #endif

            NRF_LOG_INFO("Disconnected, reason %d.", p_ble_evt->evt.gap_evt.params.disconnected.reason);
            advertising_restart();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

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
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
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

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}



void advertising_beacon_init(void)
{
    uint32_t                err_code;
    ble_advdata_t           advdata;
    ble_advertising_init_t  init;
    uint8_t                 flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t manuf_specific_data;


  #if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
      // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
      // UICR instead of using the default values. The major and minor values obtained from the UICR
      // are encoded into advertising data in big endian order (MSB First).
      // To set the UICR used by this example to a desired value, write to the address 0x10001080
      // using the nrfjprog tool. The command to be used is as follows.
      // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
      // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
      // the following command should be used.
      // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
      uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
      uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

      uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

      m_beacon_info[index++] = MSB_16(major_value);
      m_beacon_info[index++] = LSB_16(major_value);

      m_beacon_info[index++] = MSB_16(minor_value);
      m_beacon_info[index++] = LSB_16(minor_value);

      NRF_LOG_INFO("Beacon Major: %u/0x%04X, Minor: %u/0x%04X", major_value, major_value, minor_value, minor_value);
  #endif

    memset(&init, 0, sizeof(init));

    manuf_specific_data.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data	    = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size	    = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(ble_advdata_t));

    init.evt_handler = on_adv_evt;
    init.advdata.p_manuf_specific_data	  = &manuf_specific_data;
    init.advdata.name_type		  = BLE_ADVDATA_NO_NAME;
    init.advdata.flags			  = flags;    
    init.advdata.flags			  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.config.ble_adv_whitelist_enabled = true;
    init.config.ble_adv_fast_enabled	  = true;
    init.config.ble_adv_fast_interval	  = APP_FAST_ADV_INTERVAL;

    #ifdef APP_DEVELOPMENT
      init.config.ble_adv_fast_timeout	  = 0;
    #else
      init.config.ble_adv_fast_timeout	  = APP_ADV_DURATION;
    #endif
    init.config.ble_adv_slow_enabled	  = true;
    init.config.ble_adv_slow_interval	  = APP_SLOW_ADV_INTERVAL;
    init.config.ble_adv_slow_timeout	  = 0; // 0=disable timeout


    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);  
}



