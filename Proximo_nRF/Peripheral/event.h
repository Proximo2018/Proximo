#ifndef	EVENT_H
#define EVENT_H

  #include <stdint.h>
  #include <stdbool.h>
  #include "sk6812.h"
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


  void sk6812_timer_event	    (void * p_context);
  void sk6812_single_colour_blink   (uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count);
  void event_init		    (void);

#endif