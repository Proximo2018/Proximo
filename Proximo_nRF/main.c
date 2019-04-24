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

static volatile bool bootloader_en = false;
static void enable_enter_bootloader (void);
APP_TIMER_DEF(m_BUTTONS_id);

/*
    uint32_t	pin;
    uint32_t	pin_last_input;
    uint32_t	count;
    uint32_t	hysteresis;
    uint8_t	G;
    uint8_t	R;
    uint8_t	B;
    callback_fp	callback;
*/
PIN_EVENT button1_event = {BUTTON_1, 1, 0, 0, SK6812_RED,   &enable_enter_bootloader};
PIN_EVENT button2_event = {BUTTON_3, 1, 0, 0, SK6812_BLUE,  &delete_bonds};
PIN_EVENT button3_event = {BUTTON_2, 1, 0, 0, SK6812_GREEN, &button_adv_start};



/** @brief: Function for handling the button animations.
 */
void button_timer_event(void * p_context)
{
    check_button_press_animation(&button1_event);
    check_button_press_animation(&button2_event);
    check_button_press_animation(&button3_event);
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
    #ifdef DEBUG
      NRF_LOG_ERROR("Fatal Error on line %u, file: %s", line_num, p_file_name);
      NRF_LOG_INFO("Fatal Error on line %u, file: %s", line_num, p_file_name);
      NRF_LOG_FLUSH();
      app_error_handler(DEAD_BEEF, line_num, p_file_name);
    #endif
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

    err_code = app_timer_create(&m_BUTTONS_id, APP_TIMER_MODE_REPEATED, button_timer_event);
    APP_ERROR_CHECK(err_code);    
}

static void app_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_BUTTONS_id, APP_TIMER_TICKS(100), NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        //  Button 1 - RED
        case BSP_EVENT_KEY_1:
            NRF_LOG_INFO("Button 1: Right");
            break;

        //  Button 2 - Blue.
        case BSP_EVENT_KEY_2:
            NRF_LOG_INFO("Button 2: Left");
            break;

        //  Button 3 - Green
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Button 3: Down");
            break;

        case BSP_EVENT_SLEEP:
            break;

        case BSP_EVENT_DISCONNECT:
            break;

        case BSP_EVENT_WHITELIST_OFF:
            break;

        default:
            break;
    }
}




static void buttons_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
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
  #ifdef DEBUG
    NRF_LOG_ERROR("Fatal Error id: %u on pc %u, info: %u", id, pc, info);
    NRF_LOG_INFO("Fatal Error id: %u on pc %u, info: %u", id, pc, info);
    NRF_LOG_FLUSH();
    app_error_save_and_stop(id, pc, info);
  #endif
}

static void enable_enter_bootloader (void)
{
  bootloader_en = true;
}

static void check_enter_bootloader (void)
{
    if(!bootloader_en)
    {
      return;
    }

    if(lED_event_complete())
    {
        enter_bootloader();
    }
}



/**@brief Function for application main entry.
 */
int main(void)
{  
    // Initialize.
    log_init();
    lfclk_config();
    app_timers_init();
    buttons_init();
    proximo_io_init();
    start_saadc_timer();
    movement_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_beacon_init();
    services_init();
    conn_params_init();
    peer_manager_init();
    events_init();

    twi_init();
    th06_init();
    peer_list_load();
    advertising_start();
    app_timers_start();

    // Start execution.
    NRF_LOG_INFO("Proximo Application started.");
    NRF_LOG_FLUSH();
    measure_vcc(NULL);

    // Enter main loop.
    for (;;)
    {
        // Reset the SK6812
        check_enter_bootloader();

        if (NRF_LOG_PROCESS() == false)
        {
          nrf_pwr_mgmt_run();
        }
    }
}


