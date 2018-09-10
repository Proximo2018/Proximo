/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "sensorsim.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"

#include "ble_setup.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "proximo_board.h"
#include "io.h"
#include "sq_min_200.h"
#include "nrf_delay.h"
#include "sk6812.h"
#include "buzzer.h"
#include "th06.h"
#include "adc.h" 
#include "system.h"
#include "event.h"


APP_TIMER_DEF(m_LED_id);                                            /**< Debug LED timer. */


//#define PRINT_MEASUREMENT_RESULTS                       // Definition to enable priting all the measurement results on the debug itnerface RTT or UART
static volatile bool    measureTemperature = false;     //  Flag used to sample the th06 in the main loop

static volatile int16_t          vcc, vldr;                       // ADC result




double temperature  = 0.0;
double humidity	    = 0.0;
uint32_t movement_count = 0;
static volatile uint32_t manual_LED_timeout = 0;



/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        // Reset the compare counter of the RTC
        rtc_reload_compare();

        measureTemperature = true;
        

        // print and the clear the number of movement pulses counted
        #ifdef PRINT_MEASUREMENT_RESULTS
            NRF_LOG_INFO("Movement count: %u", movementCount);
        #endif
    }
    else if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        bootloader_enter_timeout();

        if(manual_LED_timeout != 0){
          manual_LED_timeout -= 1;
        }
    }
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}




/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void app_timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_LED_id, APP_TIMER_MODE_REPEATED, timer_led_blink_handler);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

//    err_code = app_timer_start(m_LED_id, APP_TIMER_TICKS(200), NULL);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    static uint8_t index = 0;
    uint8_t palet[7][3] = 
    {
        {SK6812_OFF},     //0
        {SK6812_GREEN},   //1
        {SK6812_RED},     //2
        {SK6812_BLUE},    //3  
        {SK6812_YELLOW},  //4
        {SK6812_PURPLE},  //5
        {SK6812_WHITE}    //6
    }; 

    switch (event)
    {
        //  Button 1 - RED
        case BSP_EVENT_KEY_1:
            NRF_LOG_INFO("Button 1: Right");

            if(index < 6){
              index += 1;
            }
            break;

        //  Button 2 - Blue.
        case BSP_EVENT_KEY_2:

            NRF_LOG_INFO("Button 2: Left");
            if(index > 0){
              index -= 1;
            }
            break;

        //  Button 3 - Green
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button 3: Down");
            bootloader_enter_check();
            break;

        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            bsp_ble_gap_disconnect();
            break;

        case BSP_EVENT_WHITELIST_OFF:
            bsp_ble_whitelist_off();
            break;

        default:
            break;
    }

//  alarm_blink(100, 100, 10);
  sk6812_blink_event(palet[index][0], palet[index][1], palet[index][2], 500, 500, 3);
//  buzz_event(1000, 50, 100, 1000, 4);
}


static void buttons_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  #if 0
    bsp_board_leds_on();
    app_error_save_and_stop(id, pc, info);
  #endif
}



/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    
    // Initialize.
    log_init();
    lfclk_config();
    app_timers_init();
    buttons_init(&erase_bonds);
    proximo_io_init();
    start_saadc_timer();
    rtc_config(&rtc_handler);
    movement_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    whitelist_load();
    advertising_beacon_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    event_init();

    // Start execution.
    NRF_LOG_INFO("Proximo Application started.");
    NRF_LOG_FLUSH();

    twi_init();
    th06_init();
    
    application_timers_start();
    advertising_start();


    // Enter main loop.
    for (;;)
    {
        if(measureTemperature == true)
        {
        }


        if (NRF_LOG_PROCESS() == false)
        {
          nrf_pwr_mgmt_run();
        }
    }
}


