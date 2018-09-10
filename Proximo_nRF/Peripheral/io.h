#ifndef IO_H
#define IO_H

  #include <stdint.h>
  #include <stdbool.h>
  #include "sdk_config.h"
  #include "bsp.h"
  #include "nrf_gpio.h"
  #include "nrf_log.h"
  #include "proximo_board.h"

  void proximo_io_init  (void);

  void proximo_tps_on   (void);
  void proximo_tps_off  (void);
  uint32_t proximo_tps_read_output(void);

  void proximo_ldr_on   (void);
  void proximo_ldr_off  (void);

  void proximo_alarm_high(void);
  void proximo_alarm_low (void);

  void timer_led_blink_handler(void * p_context);

#endif