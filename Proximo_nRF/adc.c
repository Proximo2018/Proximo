#include "adc.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_saadc.h"

#define UART_PRINTING_ENABLED                     //Enable to see SAADC output on UART. Comment out for low power operation.
#define SAADC_CALIBRATION_INTERVAL  5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER     2                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
#ifdef UART_PRINTING_ENABLED
//  static uint32_t                m_adc_evt_counter = 0;
#endif //UART_PRINTING_ENABLED

static volatile bool m_saadc_initialized = false; 


void measure_vcc(void)
{
    uint32_t err_code;

    //Set SAADC as initialized
    if(m_saadc_initialized == true)
    {
        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
#ifdef UART_PRINTING_ENABLED
    int16_t  vcc, ldr;
#endif

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;
			     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);

    #ifdef UART_PRINTING_ENABLED
//        NRF_LOG_INFO("ADC event number: %d",(int)m_adc_evt_counter);                                      //Print the event number on UART


        vcc = (p_event->data.done.p_buffer[0] * 3600) / 4096;
        ldr = (p_event->data.done.p_buffer[1] * vcc) / 4096;
        NRF_LOG_INFO("VCC: %d, %d mV LDR: %04X, %d mV", p_event->data.done.p_buffer[0], vcc, (uint16_t) p_event->data.done.p_buffer[1], ldr);    //Print the SAADC result on UART

//        m_adc_evt_counter++;
    #endif //UART_PRINTING_ENABLED				
				                                                                 //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    nrf_saadc_channel_config_t channel_config1 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	
    //Configure SAADC
    saadc_config.resolution         = NRF_SAADC_RESOLUTION_12BIT;                         //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample         = NRF_SAADC_OVERSAMPLE_16X;                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    saadc_config.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;
    saadc_config.low_power_mode     = true;
	
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config0.reference   = NRF_SAADC_REFERENCE_INTERNAL;                             //Set internal reference of fixed 0.6 volts
    channel_config0.gain        = NRF_SAADC_GAIN1_6;                                        //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config0.acq_time    = NRF_SAADC_ACQTIME_40US;                                   //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config0.mode        = NRF_SAADC_MODE_SINGLE_ENDED;                              //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config0.pin_p       = NRF_SAADC_INPUT_VDD;                                      //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config0.pin_n       = NRF_SAADC_INPUT_DISABLED;                                 //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config0.resistor_p  = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config0.resistor_n  = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    channel_config0.burst       = NRF_SAADC_BURST_ENABLED;

    channel_config1.reference   = NRF_SAADC_REFERENCE_VDD4;                                 //VDD/4 as reference.
    channel_config1.gain        = NRF_SAADC_GAIN1_4;                                        //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config1.acq_time    = NRF_SAADC_ACQTIME_40US;                                   //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config1.mode        = NRF_SAADC_MODE_SINGLE_ENDED;                              //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config1.pin_p       = NRF_SAADC_INPUT_AIN2;                                     //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config1.pin_n       = NRF_SAADC_INPUT_DISABLED;                                 //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config1.resistor_p  = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config1.resistor_n  = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    channel_config1.burst       = NRF_SAADC_BURST_ENABLED;

    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);

        //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
	
		  
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);

    m_saadc_initialized = true;  
}