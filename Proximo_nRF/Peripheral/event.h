#ifndef	EVENT_H
#define EVENT_H

  #include <stdint.h>
  #include <stdbool.h>
  #include "sk6812.h"
  #include "buzzer.h"
  #include "io.h"
  #include "app_timer.h"
  #include "nrf_delay.h"

  typedef struct
  {
    uint32_t  time_on;
    uint32_t  time_off;
    uint32_t  count;
    uint32_t  count_threshold;  
    bool      flag;
    bool      on;
  }EVENT_t;

  #define ALARM_PARAM_LENGHT  5 // 2 * uint16_t + 1 * uint8_t = 5 bytes
  #define LED_PARAM_LENGHT    8	// 2 * uint16_t + 4 * uint8_t = 8 bytes
  #define BUZZ_PARAM_LENGHT   8	// 3 * uint16_t + 2 * uint8_t = 8 bytes


  void sk6812_timer_event	    (void * p_context);
  bool alarm_blink		    (uint16_t on_time, uint16_t off_time, uint8_t repeat);
  bool sk6812_blink_event	    (uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count);
  bool buzz_event		    (uint16_t frequency, uint8_t dutycycle, uint16_t on_time, uint16_t off_time, uint8_t repeat);
  void event_init		    (void);

#endif