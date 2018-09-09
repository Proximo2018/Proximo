#include "stdint.h"
#include "buzzer.h"
#include "nrf_log.h"
#include "nrfx_pwm.h"
#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#include "proximo_board.h"

static bool init = false;

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static void buzzer_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
    }
}


void Buzz(uint8_t dutycycle, uint16_t frequency)
{
    uint32_t err; 
    static uint32_t regVal, topvalue;

    if(frequency > 10000){
       frequency = 10000;
    }

    if(frequency < 1000){
       frequency = 1000;
    }

    topvalue = 1250000 / frequency;

    if(dutycycle > 100){
      dutycycle = 50;
    }

    // Calculate the PWM register value from the maximum top value and the given duty Cycle.
    regVal = (topvalue * dutycycle) / 1000;
    topvalue /= 10;

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BUZZER_PIN ,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = topvalue,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    if(init == true)
    {
        init = false;
        nrf_drv_pwm_uninit(&m_pwm0);
    }

    err = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err);
    init = true;

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t /*const*/ seq_values[6];
    seq_values[0] = regVal;
    seq_values[1] = regVal;
    seq_values[2] = regVal;
    seq_values[3] = regVal;
    seq_values[4] = regVal;
    seq_values[5] = regVal;

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 10, NRF_DRV_PWM_FLAG_STOP);
}