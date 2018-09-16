#include "event.h"
#include "adc.h" 

APP_TIMER_DEF(m_LED_id);
APP_TIMER_DEF(m_ALARM_id);
APP_TIMER_DEF(m_BUZZER_id);

struct LED_EVENT
{
    EVENT_t event;
    uint8_t   brightness;
    RGB_s	  colour;
}LED;

struct ALARM_EVENT
{
    EVENT_t event;
}ALARM;

struct BUZZ_EVENT
{
    uint16_t  frequency;
    uint8_t   dutycycle;
    EVENT_t   event;
}BUZZ;


static volatile uint8_t button_press_animiation_pin_active = UINT8_MAX;
static SK6812_WR_BUFFERs GRB;

bool lED_event_complete (void)
{
  return !LED.event.on;
}


void disable_tps_on_event_done (void)
{
  if(proximo_tps_read_output() && (!BUZZ.event.on) && (!LED.event.on))
  {
    proximo_tps_off();  
  }
}


static void start_timer(app_timer_id_t timer_id, uint32_t time_ms)
{
    uint32_t err_code;
    err_code = app_timer_start(timer_id, APP_TIMER_TICKS(time_ms), NULL);
    APP_ERROR_CHECK(err_code);
}


void alarm_timer_event(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;

    #if 0
      NRF_LOG_INFO("Alarm %u, %u",(uint32_t) ALARM.event.flag, ALARM.event.count);
    #endif

    if(ALARM.event.flag)
    {
      ALARM.event.flag = false;
      proximo_alarm_low();

      start_timer(m_ALARM_id, ALARM.event.time_on);
    }
    else
    {
        ALARM.event.flag = true;
        proximo_alarm_high();

        ALARM.event.count += 1;

        if(ALARM.event.count >= ALARM.event.count_threshold)
        {
            ALARM.event.on = false;
        }
        else
        {
            start_timer(m_ALARM_id, ALARM.event.time_off);
        }
    }
}

void sk6812_timer_event(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;

    if(LED.event.flag)
    {
      LED.event.count += 1;

      if(LED.event.count >= LED.event.count_threshold)
      {
        LED.event.on = false;
        disable_tps_on_event_done();
      }
      else
      {
        sk6812_single_colour(SK6812_OFF, 0);
        start_timer(m_LED_id, LED.event.time_off);
      }

      LED.event.flag = false;
    }
    else
    {
      sk6812_single_colour(LED.colour.G, LED.colour.R, LED.colour.B, LED.brightness);
      start_timer(m_LED_id, LED.event.time_on);
      LED.event.flag = true;
    }
}


bool alarm_blink(uint16_t on_time, uint16_t off_time, uint8_t repeat)
{
    if(ALARM.event.on)
    {
      return false;
    }

    ALARM.event.count_threshold = repeat;
    ALARM.event.on        = true;
    ALARM.event.count	  = 0;
    ALARM.event.flag	  = false;
    ALARM.event.time_on	  = on_time;
    ALARM.event.time_off  = off_time;

    proximo_alarm_low();
    start_timer(m_ALARM_id, ALARM.event.time_on);
    return true;
}


bool sk6812_blink_event(uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count)
{
    uint32_t err_code;

    // Do not restart a LED event or an button animation is already in progress. 
    if(LED.event.on /*|| button_press_animiation_pin_active != UINT8_MAX*/){
      return false;
    }

    LED.event.on   = true;
    
    enable_tps(POWER_ON_DELAY);

    LED.brightness      = determine_brightness_reduction();
    LED.event.count_threshold = blink_count;
    LED.event.count     = 0;
    LED.event.time_on	= on_time;
    LED.event.time_off	= off_time;
    LED.colour.R        = Red;
    LED.colour.G        = Green;
    LED.colour.B        = Blue;
    LED.event.flag      = true;

    sk6812_single_colour(LED.colour.G, LED.colour.R, LED.colour.B, LED.brightness);
    start_timer(m_LED_id, LED.event.time_on);
    return true;
}


void buzz_timer_event(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;

    if(BUZZ.event.flag)
    {
      BUZZ.event.count += 1;

      if(BUZZ.event.count >= BUZZ.event.count_threshold)
      {
        BUZZ.event.on = false;
        disable_tps_on_event_done();
      }
      else
      {
        //sk6812_single_colour(SK6812_OFF, 0);
        start_timer(m_BUZZER_id, BUZZ.event.time_off);
      }
      BUZZ.event.flag = false;
    }
    else
    {
      Buzz(BUZZ.dutycycle, BUZZ.frequency);
      start_timer(m_BUZZER_id, BUZZ.event.time_on);
      BUZZ.event.flag = true;
    }
}


bool buzz_event(uint16_t frequency, uint8_t dutycycle, uint16_t on_time, uint16_t off_time, uint8_t repeat)
{
    if(BUZZ.event.on)
    {
      return false;
    }

    BUZZ.event.count_threshold = repeat;
    BUZZ.frequency        = frequency;
    BUZZ.dutycycle        = dutycycle;
    BUZZ.event.on         = true;
    BUZZ.event.count	  = 0;
    BUZZ.event.flag       = false;
    BUZZ.event.time_on	  = on_time;
    BUZZ.event.time_off	  = off_time;

    enable_tps(POWER_ON_DELAY);

    start_timer(m_BUZZER_id, BUZZ.event.time_on);
    Buzz(BUZZ.dutycycle, BUZZ.frequency);

    // Set the LED off, as they randomly lightup when the boost converter is started
    if(!LED.event.on)
    {
      sk6812_single_colour(SK6812_OFF, 0);
    }
    return true;
}

static void clear_and_write_rgb_buffer_n(uint8_t n, uint8_t G, uint8_t R, uint8_t B)
{
    // Clear the buffer
    memset(&GRB, 0, sizeof(GRB));

    for(uint8_t i = 0 ; i < n ; i++)
    {
        sk6812_write_buffer(&GRB, i, G, R, B);
    }
}



void check_button_press_animation (PIN_EVENT * config_p)
{
    uint32_t current_pin_input;
    

    if(config_p == NULL)
    {
      return;
    }

    if(button_press_animiation_pin_active != UINT8_MAX && button_press_animiation_pin_active != config_p->pin)
    {
      return;
    }

    // Read the current state of the GPIO pin input.
    current_pin_input = nrf_gpio_pin_read(config_p->pin);

    if(config_p->hysteresis != 0)
    {
      if(current_pin_input != BUTTONS_ACTIVE_STATE)
      {
          config_p->hysteresis -= 1;
      }
      
      config_p->pin_last_input = current_pin_input;
      disable_tps_on_event_done();
      return;
    }

    // Check whether the current and last pin input do not match, then return
    if(config_p->pin_last_input != current_pin_input)
    {
      config_p->pin_last_input = current_pin_input;
      return;
    }

    if(current_pin_input == BUTTONS_ACTIVE_STATE)
    {
      if(config_p->count < NUMBER_OF_SK6812)
      {
          button_press_animiation_pin_active = config_p->pin;
          config_p->count += 1;
          clear_and_write_rgb_buffer_n(config_p->count, config_p->G, config_p->R, config_p->B);

          // Enable the boost converter when not already on.    
          enable_tps(POWER_ON_DELAY); //POWER_ON_DELAY
          sk6812_colour_string(&GRB, determine_brightness_reduction());
      }
      else
      {
          // The maximum count is reached, set the hysteresis counter and call the function referenced by the function pointer if set.
          config_p->hysteresis  = PIN_HYSTERESIS;
          config_p->count	  = 0;

          button_press_animiation_pin_active = UINT8_MAX;
          sk6812_blink_event(config_p->G, config_p->R, config_p->B, BUTTON_EVENT_BLINK);

          if(config_p->callback != NULL)
          {
            config_p->callback();
          }
      }
    }
    else
    {
      if(config_p->count != 0)
      {
          config_p->count -= 1;

          // Clear the SK6812 buffer and then place the event's colour in the buffer for count times
          clear_and_write_rgb_buffer_n(config_p->count, config_p->G, config_p->R, config_p->B);

          // Enable the boost converter when not already on.
          enable_tps(POWER_ON_DELAY);
          sk6812_colour_string(&GRB, determine_brightness_reduction());
      }
      else
      {
          disable_tps_on_event_done();
          button_press_animiation_pin_active = UINT8_MAX;
      }
    }

    config_p->pin_last_input = current_pin_input;
}


void events_init(void)
{
    ret_code_t	err_code;

    // Create application timer.
    err_code = app_timer_create(&m_LED_id, APP_TIMER_MODE_SINGLE_SHOT, sk6812_timer_event);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ALARM_id, APP_TIMER_MODE_SINGLE_SHOT, alarm_timer_event);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_BUZZER_id, APP_TIMER_MODE_SINGLE_SHOT, buzz_timer_event);
    APP_ERROR_CHECK(err_code);
}