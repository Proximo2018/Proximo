#include "event.h"

struct LED_EVENT
{
  EVENT_t event;
  RGB_s	  colour;
}LED;

APP_TIMER_DEF(m_LED_id);
APP_TIMER_DEF(m_ALARM_id);
APP_TIMER_DEF(m_BUZZER_id);

void sk6812_timer_event(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;

    if(LED.event.flag)
    {
	LED.event.count += 1;

	if(LED.event.count >= LED.event.count_threshold)
	{
	  proximo_tps_off();
	  LED.event.on = false;
	}
	else
	{
	  sk6812_single_colour(SK6812_OFF);
	  err_code = app_timer_start(m_LED_id, APP_TIMER_TICKS(LED.event.time_off), NULL);
	  APP_ERROR_CHECK(err_code);
	}

	LED.event.flag = false;
    }
    else
    {
	sk6812_single_colour(LED.colour.G, LED.colour.R, LED.colour.B);

	err_code = app_timer_start(m_LED_id, APP_TIMER_TICKS(LED.event.time_on), NULL);
	APP_ERROR_CHECK(err_code);

	LED.event.flag = true;
    }
}

void sk6812_single_colour_blink(uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count)
{
    uint32_t err_code;

    if(LED.event.on){
      return;
    }

    LED.event.on = true;

    proximo_tps_on();
    nrf_delay_ms(100);

    LED.event.count_threshold = blink_count;
    LED.event.count	= 0;
    LED.event.time_on	= on_time;
    LED.event.time_off	= off_time;
    LED.colour.R	= Red;
    LED.colour.G	= Green;
    LED.colour.B	= Blue;
    LED.event.flag	= true;

    sk6812_single_colour(LED.colour.G, LED.colour.R, LED.colour.B);

    err_code = app_timer_start(m_LED_id, APP_TIMER_TICKS(LED.event.time_on), NULL);
    APP_ERROR_CHECK(err_code);
}



void event_init(void)
{
    ret_code_t	err_code;

    // Create application timer.
    err_code = app_timer_create(&m_LED_id, APP_TIMER_MODE_SINGLE_SHOT, sk6812_timer_event);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_create(&m_LED_id, APP_TIMER_MODE_SINGLE_SHOT, sk6812_timer_event);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = app_timer_create(&m_LED_id, APP_TIMER_MODE_SINGLE_SHOT, sk6812_timer_event);
//    APP_ERROR_CHECK(err_code);
}