#include "stdint.h"
#include "sk6812.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrfx_pwm.h"
#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#include "proximo_board.h"
#include "io.h"

static volatile bool init = false;
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
                                          /**< Event LED timer. */

// This array cannot be allocated on stack (hence "static") and it must
// be in RAM (hence no "const", though its content is not changed).
static uint16_t pwm_bit_buffer[SK6812_PWM_BUFFER_LENGHT];




static void sk6812_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRFX_PWM_EVT_STOPPED)
    {
        nrf_drv_pwm_uninit(&m_pwm1);
        init = false;
    }
}

static void sk6812_pwm_init(void)
{
  uint32_t err;
  nrf_drv_pwm_config_t const config1 =
  {
      .output_pins =
      {
	  SK6812_DIN_PIN,             // channel 0
	  NRF_DRV_PWM_PIN_NOT_USED,   // channel 1
	  NRF_DRV_PWM_PIN_NOT_USED,   // channel 2
	  NRF_DRV_PWM_PIN_NOT_USED,   // channel 3
      },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock   = PWM_PRESCALER_PRESCALER_DIV_1,
      .count_mode   = NRF_PWM_MODE_UP,
      .top_value    = FDIN_PERIOD_COUNT,
      .load_mode    = NRF_PWM_LOAD_COMMON,
      .step_mode    = NRF_PWM_STEP_AUTO
  };

  err = nrf_drv_pwm_init(&m_pwm1, &config1, sk6812_handler);
  APP_ERROR_CHECK(err);
  init = true;
}


static void sk6812_write_byte_to_pwm_buffer(uint8_t colour, uint16_t * offset)
{
  uint8_t i;

  if(offset == NULL){
    return;
  }

  //  Set the bits for the colour
  for(i = 0 ; i < 8 ; i++)
  {
      if(colour & (1U << (7-i)))
      {
	  pwm_bit_buffer[(*offset) + i] = PWM_COUNT_BIT1;
      }
      else
      {
	  pwm_bit_buffer[(*offset) + i] = PWM_COUNT_BIT0;
      }
  }

  (*offset) += 8;
}






void sk6812_single_colour(uint8_t Green, uint8_t Red, uint8_t Blue)
{
    uint32_t err;
    uint16_t offset, i;

    if(!init){
      sk6812_pwm_init();
    }

    offset = 0;

    //  Set the bits for the GRB bytes
    sk6812_write_byte_to_pwm_buffer(Green >> BRIGHTNESS_REDUCTION, &offset);
    sk6812_write_byte_to_pwm_buffer(Red   >> BRIGHTNESS_REDUCTION, &offset);
    sk6812_write_byte_to_pwm_buffer(Blue  >> BRIGHTNESS_REDUCTION, &offset);

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = pwm_bit_buffer,
        .length          = offset,
        .repeats         = 0,
        .end_delay       = 0
    };

    // Repeat the pwm buffer created for a single colour for the number of LEDs
    nrf_drv_pwm_simple_playback(&m_pwm1, &seq, NUMBER_OF_SK6812, NRF_DRV_PWM_FLAG_STOP);
}






void sk6812_colour_string(SK6812_WR_BUFFERs * GRB)
{
    uint16_t  offset;
    uint8_t   n, G, R, B, LED_index, row, column;
    uint32_t  err;

    if(GRB == NULL){
        return;
    }

    if(!init){
      sk6812_pwm_init();
    }

    offset = 0;

    // Set the given colours on the SK6812 LED matrix. Loop through the LEDs for each row and column, to compensate for the fact that the colour array and the displayed LEDs are not in the same order. 
    for(row = 0 ; row < NUMBER_OF_ROWS ; row++)
    {
	for(column = 0 ; column < NUMBER_OF_COLUMNS ; column++)
	{
	    LED_index = (column * NUMBER_OF_COLUMNS) + ((NUMBER_OF_ROWS - row - 1) * NUMBER_OF_COLUMNS * NUMBER_OF_ROWS);
	    G = GRB->data[LED_index + 0U] >> BRIGHTNESS_REDUCTION;
	    R = GRB->data[LED_index + 1U] >> BRIGHTNESS_REDUCTION;
	    B = GRB->data[LED_index + 2U] >> BRIGHTNESS_REDUCTION;

	    sk6812_write_byte_to_pwm_buffer(G, &offset);
	    sk6812_write_byte_to_pwm_buffer(R, &offset);
	    sk6812_write_byte_to_pwm_buffer(B, &offset);
	}
    }

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = pwm_bit_buffer,
        .length          = offset,
        .repeats         = 0,
        .end_delay       = 0
    };

    nrf_drv_pwm_simple_playback(&m_pwm1, &seq, 1, NRF_DRV_PWM_FLAG_STOP);
}


void sk6812_write_buffer(SK6812_WR_BUFFERs * GRB, uint8_t index, uint8_t Green, uint8_t Red, uint8_t Blue)
{
  uint8_t offset;

  if(GRB == NULL || index > NUMBER_OF_SK6812)
  {
      return;
  }

  offset = index * 3;
  GRB->data[offset + 0] = Green;
  GRB->data[offset + 1] = Red;
  GRB->data[offset + 2] = Blue;
}



