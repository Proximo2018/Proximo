#include "system.h"
//#include "bsp.h"

#define COMPARE_COUNTERTIME  (4UL)                      /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);      /**< Declaring an instance of nrf_drv_rtc for RTC1. Note that RTC0 is used by the soft device */

/** @brief Function initialization and configuration of RTC driver instance.
 */
void rtc_config(nrfx_rtc_handler_t rtc_handler)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME, true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}


void rtc_reload_compare (void)
{
    uint32_t err_code;  
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_counter_clear(&rtc);
}


/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
void lfclk_config (void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


/**@brief Function for initializing power management.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    ret_code_t err_code;

    // Sleep until the next event wakes the device up
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}