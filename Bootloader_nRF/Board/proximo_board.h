/*
 *	Proximo board definitions
 *	The Proximo board is selected by the CUSTOM_BOARD_INC=proximo_board.h definition in the Preprocessor Definitions 
 *	Project options under "Common".
 */
#ifndef PROXIMO_BOARD_H
#define PROXIMO_BOARD_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "bsp.h"
#include "nrf_gpio.h"
#include "nrf_drv_lpcomp.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_lpcomp.h"
#include "nrf_log.h"


#define HWFC           true


#define VIBRATION_PIN     2 //  Input pin for the vibration sensor
#define LDR_VCC_PIN       3 //  LDR Supply Pin
#define LDR_OUT_PIN       4 //  LDR input pin
#define RTS_PIN_NUMBER    5 //  Request to Send
#define RX_PIN_NUMBER     6 //  RX pin
#define CTS_PIN_NUMBER    7 //  Clear to Send
#define TX_PIN_NUMBER     8 //  TX pin
#define I2C_SCL_PIN      11 //  I2C Serial Clock Line
#define I2C_SDA_PIN      12 //  I2C Serial Data
#define SK6812_DIN_PIN   13 //  DATA IN pin for the SK6812 mini RGB LED
#define BUZZER_PIN       14 //  Buzzer pin
#define ALARM_OUT_PIN    15 //  Alarm output pin
#define BUTTON_1         16 //  Pushbutton 1
#define BUTTON_2         17 //  Pushbutton 2
#define BUTTON_3         19 //  Pushbutton 3
#define TPS_EN_PIN       24 //  Boost converter enable pin
  
// Button definitions for the Proximo hardware
#define BUTTONS_NUMBER      3
#define BUTTON_START        13
#define BUTTON_STOP         19
#define BUTTON_PULL         NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0
#define BUTTONS_LIST        { BUTTON_1, BUTTON_2, BUTTON_3 }

#define BSP_BUTTON_0        BUTTON_1
#define BSP_BUTTON_1        BUTTON_2
#define BSP_BUTTON_2        BUTTON_3

// LEDs definitions for the Proximo hardware
#define LEDS_NUMBER         1
#define LED_START           ALARM_OUT_PIN
#define LED_1               ALARM_OUT_PIN
#define LED_STOP            ALARM_OUT_PIN
#define LEDS_ACTIVE_STATE   1
#define LEDS_INV_MASK       LEDS_MASK
#define LEDS_LIST           { ALARM_OUT_PIN }
#define BSP_LED_0           LED_1


void movement_init    (void (*movement_event_handler)(nrf_lpcomp_event_t));
void proximo_io_init  (void);
void movement_deinit  (void);


void proximo_tps_on   (void);
void proximo_tps_off  (void);

void proximo_ldr_on   (void);
void proximo_ldr_off  (void);

#ifdef __cplusplus
}
#endif


#endif
