#include "io.h"


void proximo_io_init(void)
{
    //  Input pin for the vibration sensor SQ-MIN-200
    nrf_gpio_cfg_input(VIBRATION_PIN, NRF_GPIO_PIN_NOPULL);

    //  LDR Supply Pin
    nrf_gpio_cfg_output(LDR_VCC_PIN);
    nrf_gpio_pin_write(LDR_VCC_PIN, 0);

    //  LDR input pin
    nrf_gpio_cfg_input(LDR_OUT_PIN, NRF_GPIO_PIN_NOPULL);

    //  DATA IN pin for the SK6812 mini RGB LED
    nrf_gpio_cfg_output(SK6812_DIN_PIN);
    nrf_gpio_pin_write(SK6812_DIN_PIN, 0);

    //  Buzzer pin
    nrf_gpio_cfg_output(BUZZER_PIN);
    nrf_gpio_pin_write(BUZZER_PIN, 0);

    //  Alarm output pin
    nrf_gpio_cfg_output(ALARM_OUT_PIN);
    nrf_gpio_pin_write(ALARM_OUT_PIN, 0);

    //  Boost converter enable pin
    nrf_gpio_cfg_output(TPS_EN_PIN);
    nrf_gpio_pin_write(TPS_EN_PIN, 0);
}



__inline void proximo_tps_on(void)
{
  nrf_gpio_pin_write(TPS_EN_PIN, 1);
}

__inline void proximo_tps_off(void)
{
  nrf_gpio_pin_write(TPS_EN_PIN, 0);
}

__inline void proximo_ldr_on(void)
{
  nrf_gpio_pin_write(LDR_VCC_PIN, 1);
}

__inline void proximo_ldr_off(void)
{
  nrf_gpio_pin_write(LDR_VCC_PIN, 0);
}

__inline void proximo_alarm_low(void)
{
  nrf_gpio_pin_write(ALARM_OUT_PIN, 1);
}

__inline void proximo_alarm_high(void)
{
  nrf_gpio_pin_write(ALARM_OUT_PIN, 0);
}





/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
void buttons_init(bool * p_erase_bonds, void (*handler)(bsp_event_t))
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, *handler);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

// Timeout handler for the repeated timer
void timer_led_blink_handler(void * p_context)
{
    nrf_gpio_pin_toggle(ALARM_OUT_PIN);
}