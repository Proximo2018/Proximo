#ifndef ADC_H
#define ADC_H

    #include <stdbool.h>
    #include <stdint.h>
    #include "nrf.h"
    #include "nrf_log.h"
    #include "nrf_drv_saadc.h"
    #include "nrf_drv_saadc.h"

    #define SAADC_CALIBRATION_INTERVAL  5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
    #define SAADC_SAMPLES_IN_BUFFER     2                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.


    void measure_vcc    (void);
    void measure_vcc_ldr(void);
    void saadc_callback (nrf_drv_saadc_evt_t const * p_event);
    void saadc_init     (void);
    int16_t get_vcc     (void);
    int16_t get_vldr    (void);

#endif
