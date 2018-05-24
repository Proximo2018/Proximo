/*
*/
#include "nrf_gpio.h"
#include "sdk_config.h"
#include "proximo_board.h"
#include "nrf_delay.h"
#include "nrf_drv_lpcomp.h"


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





/*
 * @brief Initialize LPCOMP driver.
 */
void movement_init(void (*movement_event_handler)(nrf_lpcomp_event_t))
{
    uint32_t err_code;
	
    /*
    * Configures the LPCOMP to VCC4/8, UP with hysteresis enabled
    */

    nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;
	
    // Initialize LPCOMP driver, from this point LPCOMP will be active and provided event handler will be executed when defined action is detected
    err_code = nrf_drv_lpcomp_init(&config, *(movement_event_handler));
    APP_ERROR_CHECK(err_code);
    nrf_drv_lpcomp_enable();
}


/*
 * @brief De-initialize LPCOMP driver for shutdown.
 */
void movement_deinit(void)
{
  //  Disable the Low Power Comperator.
  nrf_drv_lpcomp_disable();
  nrf_drv_lpcomp_uninit();
}