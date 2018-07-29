#ifndef SQ_MIN_200
#define SQ_MIN_200

  #include <stdint.h>
  #include "nrf_drv_lpcomp.h"
  #include "nrf_gpio.h"
  #include "proximo_board.h"

  void movement_init    (void);
  void movement_deinit  (void);

  uint32_t  get_movement_count    (void);
  void      clear_movement_count  (void);

#endif