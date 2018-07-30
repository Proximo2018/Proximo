#include "nrf_drv_twi.h"
#include "th06.h"
#include "sdk_config.h"
#include "proximo_board.h"

#include "nrf_log.h"


static TH06_s th06;                           //  Temperature and humidity result


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);



// TWI (with transaction manager) initialization.
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_th06_config = {
       .scl                = I2C_SCL_PIN,
       .sda                = I2C_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_th06_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


void th06_init(void)
{
    ret_code_t err_code;
    static uint8_t reg[3] = {0};
    uint8_t statusRegVal;

    /* Read the User register 1 with a repeated start. */
    reg[0] = TH06_READ_USER_REG1;
    err_code = nrf_drv_twi_tx(&m_twi, THO6_I2C_ADDRESS, reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, THO6_I2C_ADDRESS, &reg[1], 1);
    APP_ERROR_CHECK(err_code);

    //  Mask the reserved bits of the TH06 and configure the TH06 settings.
    statusRegVal = reg[1];
    reg[1] &= TH06_RSVD_MASK;
    reg[1] = RES_RH12_Temp14 | HEATER_OFF;

    /* Write User register 1 with the new setting configuration. */
    reg[0] = TH06_WRITE_USER_REG1;
    err_code = nrf_drv_twi_tx(&m_twi, THO6_I2C_ADDRESS, reg, 2, false);

    #if 0
      NRF_LOG_INFO("Init error code: %X, %d, User Register1: %02X", err_code, err_code, statusRegVal);
    #endif
    APP_ERROR_CHECK(err_code);
}


void th06_sample_temperature_and_humidity(TH06_s * result)
{
    ret_code_t err_code;
    uint16_t regVal;
    static float temperature, humidity;

    /* Read the User register 1 with a repeated start. */
    uint8_t reg[3];

    if(result == NULL){
        return;
    }

    reg[0] = TH06_MEASURE_HUMIDITY_HOLD_MASTER;
    err_code = nrf_drv_twi_tx(&m_twi, THO6_I2C_ADDRESS, reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, THO6_I2C_ADDRESS, reg, 2);
    APP_ERROR_CHECK(err_code);

    regVal = (((uint16_t) reg[0]) << 8) | ((uint16_t)reg[1]);

    // Calculate the humidity
    if(regVal == 0)
    {
        result->humidity = 0.0;
    }
    else
    {
        result->humidity = ((125.0 * ((float) regVal)) / 65536.0) - 6.0;
    }

    //  Read the temperature result
    reg[0] = TH06_MEASURE_TEMPERATURE_PREVIOUS;
    err_code = nrf_drv_twi_tx(&m_twi, THO6_I2C_ADDRESS, reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, THO6_I2C_ADDRESS, &reg[0], 2);
    APP_ERROR_CHECK(err_code);

    regVal = (((uint16_t) reg[0]) << 8) | ((uint16_t)reg[1]);

    // Calculate the temperature
    if(regVal == 0)
    {
        result->temperature = 0.0;
    }
    else
    {
        result->temperature = ((175.72 * ((double) regVal)) / 65536.0) - 46.85;
    }
}

void th06_sample (void)
{
  th06_sample_temperature_and_humidity(&th06);
}

double th06_get_last_measured_temperature(void)
{
  return th06.temperature;
}

double th06_get_last_measured_humidity(void)
{
  return th06.humidity;
}
