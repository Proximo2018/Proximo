#include "sq_min_200.h"



static volatile uint32_t movementCount = 0;


void movement_event_handler(nrf_lpcomp_event_t event)
{
  if (event == NRF_LPCOMP_EVENT_UP && movementCount != UINT32_MAX)
  {
      // Check if the next increment will cause an overflow if not, increment the pulse count value.
      movementCount++;
  }
}

/*
 * @brief Initialize LPCOMP driver.
 */
void movement_init(void)
{
  uint32_t err_code;
  
  /*
  * Configures the LPCOMP to VCC4/8, UP with hysteresis enabled as set in the SDK_config file
  */
  nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;
  
  // Initialize LPCOMP driver, from this point LPCOMP will be active and provided event handler will be executed when defined action is detected
  err_code = nrf_drv_lpcomp_init(&config, movement_event_handler);
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

uint32_t get_movement_count(void)
{
  return movementCount;
}

void clear_movement_count(void)
{
  movementCount = 0;
}