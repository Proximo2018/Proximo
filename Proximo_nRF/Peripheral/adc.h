#ifndef ADC_H
#define ADC_H

    #include <stdbool.h>
    #include <stdint.h>
    #include "nrf.h"
    #include "nrf_log.h"
    #include "nrf_drv_saadc.h"
    #include "nrf_drv_saadc.h"

    #define ADC_INTERVAL                APP_TIMER_TICKS(30000)
    #define SAADC_CALIBRATION_INTERVAL  5		  //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
    #define SAADC_SAMPLES_IN_BUFFER     1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
    #define CUTT_OF_VOLTAGE             2000
    #define OPEN_CIRCUIT_VOLTAGE        2800		  //The actual measured supply voltage is lower due to diode losses

    void measure_vcc            (void * p_context);
    void saadc_callback         (nrf_drv_saadc_evt_t const * p_event);
    void start_saadc_timer      (void);
    void saadc_init             (void);
    int16_t get_vcc             (void);
    uint8_t get_bat_percentage	(void);

#endif
