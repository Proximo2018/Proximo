#ifndef	EVENT_H
#define EVENT_H

  #include <stdint.h>
  #include <stdbool.h>
  #include "sk6812.h"
  #include "buzzer.h"
  #include "io.h"
  #include "app_timer.h"
  #include "nrf_delay.h"
  #include "proximo_board.h" 

  #define ALARM_PARAM_LENGHT  5 // 2 * uint16_t + 1 * uint8_t = 5 bytes
  #define LED_PARAM_LENGHT    8	// 2 * uint16_t + 4 * uint8_t = 8 bytes
  #define BUZZ_PARAM_LENGHT   8	// 3 * uint16_t + 2 * uint8_t = 8 bytes
  #define PIN_HYSTERESIS      5
  #define BUTTON_EVENT_BLINK  100, 100, 10  // Upon reaching the maximum pushbutton count the LEDs wil flash 10 times 10ms on, 10 ms off 
  #define POWER_ON_DELAY      10

  typedef struct
  {
    uint32_t  time_on;
    uint32_t  time_off;
    uint32_t  count;
    uint32_t  count_threshold;  
    bool      flag;
    bool      on;
  }EVENT_t;

  typedef  void (*callback_fp) (void);	 // Void callback function pointer typedefinition

  typedef struct
  {
    uint32_t	pin;
    uint32_t	pin_last_input;
    uint32_t	count;
    uint32_t	hysteresis;
    uint8_t	G;
    uint8_t	R;
    uint8_t	B;
    callback_fp	callback;
  }PIN_EVENT;



  void disable_tps_on_event_done    (void);
  void sk6812_timer_event	    (void * p_context);
  bool alarm_blink		    (uint16_t on_time, uint16_t off_time, uint8_t repeat);
  bool sk6812_blink_event	    (uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count);
  bool buzz_event		    (uint16_t frequency, uint8_t dutycycle, uint16_t on_time, uint16_t off_time, uint8_t repeat);
  void events_init		    (void);
  void check_button_press_animation (PIN_EVENT * config_p);


#endif