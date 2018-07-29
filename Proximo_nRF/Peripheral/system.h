#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
  
void rtc_config             (void (*rtc_handler)(nrf_drv_rtc_int_type_t));
void rtc_reload_compare     (void);

void lfclk_config           (void);
void power_management_init  (void);

void sleep_mode_enter       (void);
void idle_state_handle      (void);
void power_manage           (void);

#endif