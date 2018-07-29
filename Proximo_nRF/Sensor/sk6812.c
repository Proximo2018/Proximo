#include "stdint.h"
#include "sk6812.h"
#include "nrf_log.h"
#include "nrfx_pwm.h"
#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#include "proximo_board.h"

static bool init = false;

static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);

static void sk6812_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        nrf_drv_pwm_uninit(&m_pwm1);
    }
}


void sk6812_single_colour(uint8_t Green, uint8_t Red, uint8_t Blue, uint8_t brightnessReduction)
{
    uint32_t err;
    uint8_t offset, i;


    nrf_drv_pwm_config_t const config1 =
    {
        .output_pins =
        {
            SK6812_DIN_PIN,             // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,   // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,   // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,   // channel 3
        },
        .irq_priority = _PRIO_APP_MID, //APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = PWM_PRESCALER_PRESCALER_DIV_1,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = FDIN_PERIOD_COUNT,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    if(init == true)
    {
        nrf_drv_pwm_uninit(&m_pwm1);
        init = false;
    }

    err = nrf_drv_pwm_init(&m_pwm1, &config1, sk6812_handler);
    APP_ERROR_CHECK(err);
    init = true;

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t seq_values[25];

    offset = 0;

    //  Set the bits for the Green byte
    for(i = 0 ; i < 8 ; i++)
    {
        if((Green >> brightnessReduction) & (1<<(7-i)))
        {
            seq_values[offset+i] = PWM_COUNT_BIT1;
        }
        else
        {
            seq_values[offset+i] = PWM_COUNT_BIT0;
        }
    }
    offset += 8;

    //  Set the bits for the Red byte
    for(i = 0 ; i < 8 ; i++)
    {
        if((Red >> brightnessReduction) & (1<<(7-i)))
        {
            seq_values[offset+i] = PWM_COUNT_BIT1;
        }
        else
        {
            seq_values[offset+i] = PWM_COUNT_BIT0;
        }
    }
    offset += 8;

    //  Set the bits for the Blue byte 
    for(i = 0 ; i < 8 ; i++)
    {
        if((Blue >> brightnessReduction) & (1<<(7-i)))
        {
            seq_values[offset+i] = PWM_COUNT_BIT1;
        }
        else
        {
            seq_values[offset+i] = PWM_COUNT_BIT0;
        }
    }
    offset += 8;

    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = offset,
        .repeats         = 0,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm1, &seq, NUMBER_OF_SK6812, NRF_DRV_PWM_FLAG_STOP);
}


void sk6812_colour_string(SK6812_WR_BUFFERs * GRB)
{
    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t seq_values[SK6812_PWM_BUFFER_LENGHT];

    uint16_t  i, offset;
    uint8_t   n, G, R, B;
    uint32_t err;

    if(GRB == NULL){
        return;
    }

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

    if(init == true)
    {
        nrf_drv_pwm_uninit(&m_pwm1);
        init = false;
    }

    err = nrf_drv_pwm_init(&m_pwm1, &config1, sk6812_handler);
    APP_ERROR_CHECK(err);
    init = true;

    offset = 0;
    for(n = 0 ; n < NUMBER_OF_SK6812 ; n++)
    {
        G = GRB->data[(n * 3U) + 0U];
        R = GRB->data[(n * 3U) + 1U];
        B = GRB->data[(n * 3U) + 2U];

        //  Set the bits for the Green byte
        for(i = 0 ; i < 8 ; i++)
        {
            if(G & (1U<<(7-i)))
            {
                seq_values[offset+i] = PWM_COUNT_BIT1;
            }
            else
            {
                seq_values[offset+i] = PWM_COUNT_BIT0;
            }
        }
        offset += 8;

        //  Set the bits for the Red byte
        for(i = 0 ; i < 8 ; i++)
        {
            if(R & (1U<<(7-i)))
            {
                seq_values[offset+i] = PWM_COUNT_BIT1;
            }
            else
            {
                seq_values[offset+i] = PWM_COUNT_BIT0;
            }
        }
        offset += 8;

        //  Set the bits for the Blue byte 
        for(i = 0 ; i < 8 ; i++)
        {
            if(B & (1U<<(7-i)))
            {
                seq_values[offset+i] = PWM_COUNT_BIT1;
            }
            else
            {
                seq_values[offset+i] = PWM_COUNT_BIT0;
            }
        }
        offset += 8;
    }


    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = offset,
        .repeats         = 0,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm1, &seq, 1, NRF_DRV_PWM_FLAG_STOP);
}



