#ifndef TH06_H
#define TH06_H

#include <stdint.h>
#include "nrf_twi_mngr.h"

//    extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND th06_write_user_reg1;


   /******************************************************************************************
                                            DEFINES
    ******************************************************************************************/

    #define THO6_I2C_ADDRESS (0x80U>>1)
    #define TH06_RSVD_MASK   0b00111010 // bit mask for the temperature user register 1

//    extern uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND th06_write_user_reg1;

//    #define TH06_READ(p_reg_addr, p_buffer, byte_cnt)                                   \
//    NRF_TWI_MNGR_WRITE(THO6_I2C_ADDRESS, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
//    NRF_TWI_MNGR_READ (THO6_I2C_ADDRESS, p_buffer,   byte_cnt, 0)

    /******************************************************************************************
                                        TYPE DEFINITIONs
    ******************************************************************************************/

    /* TH06 Registers */
    typedef enum
    {
        TH06_MEASURE_HUMIDITY_HOLD_MASTER           = 0xE5,
        TH06_MEASURE_HUMIDITY_NO_HOLD_MASTER        = 0xF5,
        TH06_MEASURE_TEMPERATURE_HOLD_MASTER        = 0xE3,
        TH06_MEASURE_TEMPERATURE_NO_HOLD_MASTER     = 0xF3,
        TH06_MEASURE_TEMPERATURE_PREVIOUS           = 0xE0,
        TH06_RESET                                  = 0xFE,
        TH06_WRITE_USER_REG1                        = 0xE6,
        TH06_READ_USER_REG1                         = 0xE7,
        TH06_FIRMWARE_REV                           = 0x84
    }TH06_I2C_COMMANDS;

    typedef enum 
    {
        RES_MASK        = 0b10000001U,   //  mask for bits 7 and 0
        RES_RH12_Temp14 = 0b00000000U,   //  22.8ms MAX conversion time
        RES_RH8_Temp12  = 0b00000001U,   //  6.9ms
        RES_RH10_Temp13 = 0b10000000U,   //  23.0ms
        RES_RH11_Temp11 = 0b10000001U    //  9.4ms
    }TEMP_RESOLUTION;


    /* Heater enumeration to enable or disable the heater on initialisation. */
    typedef enum 
    {
        HEATER_ON  = 0x04U,
        HEATER_OFF = 0x00U
    }HEATERe;

    typedef struct
    {
        float temperature;
        float humidity;
    }TH06_s;



    /*
     *  FUNCTION PROTOTYPES
     */
     void   twi_init          (void);
     void   th06_init         (void);
     void   read_temperature  (TH06_s * result);
     void   th06_sample       (void);
     double th06_get_last_measured_temperature  (void);
     double th06_get_last_measured_humidity     (void);
    
#endif