#ifndef BLE_SETUP_H
#define BLE_SETUP_H

  #include "app_error.h"
  #include "ble.h"
  #include "ble_err.h"
  #include "ble_hci.h"
  #include "ble_srv_common.h"
  #include "ble_advdata.h"
  #include "ble_advertising.h"
  #include "ble_bas.h"
  #include "ble_hrs.h"
  #include "ble_dis.h"
  #include "ble_conn_params.h"
  #include "sensorsim.h"
  #include "nrf_sdh.h"
  #include "nrf_sdh_ble.h"
  #include "nrf_sdh_soc.h"
  #include "bsp_btn_ble.h"
  #include "peer_manager.h"
  #include "fds.h"
  #include "nrf_ble_gatt.h"
  #include "nrf_ble_qwr.h"
  #include "ble_conn_state.h"
  #include "nrf_log.h"
  #include "nrf_log_ctrl.h"
  #include "nrf_log_default_backends.h"

  #include "app_timer.h"
  #include "system.h"

  /* Beacon */
  #define USE_UICR_FOR_MAJ_MIN_VALUES
  #define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
  #define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

  #if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    #define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
    #define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
  #endif

  #define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
  #define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
  #define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
  #define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */

  #if 1	
    #define APP_COMPANY_IDENTIFIER	    0x004C			      /**< Company identifier for Apple Inc. as per www.bluetooth.org. */  
  #else
    #define APP_COMPANY_IDENTIFIER          0x0059                           /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
  #endif

  #define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
  #define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
  #define APP_BEACON_UUID                 0x6b, 0xb0, 0xb6, 0x7a, 0x94, \
					  0x99, 0x4f, 0xa1, 0xab, 0xbc, \
					  0x08, 0x6e, 0x0a, 0x06, 0x41, 0x66 /**< Proprietary UUID for Beacon. */


  /* BLE HRS */
  #define DEVICE_NAME                         "Proximo   "                            /**< Name of device. Will be included in the advertising data. */
  #define MANUFACTURER_NAME                   "FHI"                                   /**< Manufacturer. Will be passed to Device Information Service. */
  #define APP_FAST_ADV_INTERVAL               MSEC_TO_UNITS(100, UNIT_0_625_MS)	      /**< The advertising interval (in units of 0.625 ms.). */
  #define APP_SLOW_ADV_INTERVAL               MSEC_TO_UNITS(300, UNIT_0_625_MS)	      /**< The advertising interval (in units of 0.625 ms.). */
  #define APP_ADV_DURATION                    3000				      /**< The advertising duration (30 seconds) in 10 milliseconde resolutie. */

  #define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
  #define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

  #define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
  #define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
  #define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
  #define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

  #define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

  #define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
  #define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
  #define SLAVE_LATENCY                       0                                       /**< Slave latency. */
  #define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

  #define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
  #define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
  #define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

  #define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
  #define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
  #define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
  #define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
  #define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
  #define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
  #define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
  #define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

  #define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

  /* Bootloader functions */
  void bootloader_enter_check   (void);
  void bootloader_enter_timeout (void);
  void enter_bootloader         (void);

  void delete_bonds		(void);
  void whitelist_load		(void);
  void peer_list_get		(pm_peer_id_t * p_peers, uint32_t * p_size);

  /* BLE */
  void advertising_stop		(void);
  void advertising_start        (void);
  void peer_list_load		(void);

  void sensorsim_app_timers_init(void);
  void gap_params_init          (void);
  void gatt_init                (void);
  void services_init            (void);
  void sensor_simulator_init    (void);
  void conn_params_init         (void);
  void on_adv_evt               (ble_adv_evt_t ble_adv_evt);
  void ble_evt_handler          (ble_evt_t const * p_ble_evt, void * p_context);
  void ble_stack_init           (void);
  void peer_manager_init        (void);
  void advertising_init         (void);
  void bsp_ble_gap_disconnect   (void);
  void bsp_ble_whitelist_off    (void);

  void advertising_beacon_init  (void);
  void battery_level_update     (uint8_t battery_level);

#endif