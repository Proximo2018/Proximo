/*
 */
#ifndef SK6812_H
#define SK6812_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define BRIGHTNESS_REDUCTION 4

// Buffer definitions
#define NUMBER_OF_SK6812  9U
#define NUMBER_OF_COLUMNS 3U
#define NUMBER_OF_ROWS	  3U
#define SK6812_NUMBER_OF_BITS_DATA    (3* 8)                      //  The number of bits in a data message for the SK6812: three bytes with the following format: Green, Red, Blue
#define SK6812_WR_BUFFER_LENGHT       (3 * NUMBER_OF_SK6812)      // Lenght of the data array for writing the SK6812
#define SK6812_PWM_BUFFER_LENGHT      (3 * 8 * NUMBER_OF_SK6812)  // 3 * 8 * 9 = 216  The buffer size for the PWM peripheral

// Frequency definitions
#define FDIN_SK8612     (833333UL)                        //  The communication frequency of the SK6812 DIN interface: 1/1.2us = 833.33 kHz
#define FDIN_PERIOD_COUNT    (16000000UL / FDIN_SK8612)        //  The number of time ticks for a single period
#define PWM_COUNT_BIT0  ((FDIN_PERIOD_COUNT * 70UL) / 100UL)   //  The compare value for a bit with value 0. High time 380ns, Low time: 800ns, Period: 1.18us    
#define PWM_COUNT_BIT1  ((FDIN_PERIOD_COUNT * 50UL) / 100UL)   //  The compare value for a bit with value 1. High time 640ns, Low time: 540ns, Period: 1.18us 

typedef struct
{
    uint8_t data[SK6812_WR_BUFFER_LENGHT];
}SK6812_WR_BUFFERs;

typedef struct
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
}RGB_s;

typedef struct
{
  uint32_t  time_on;
  uint32_t  time_off;
  uint32_t  count;
  uint32_t  threshold;  
  RGB_s	    event_colour;
  bool	    flag;
}RGB_event;

#define SK6812_OFF      0x00, 0x00, 0x00
#define SK6812_GREEN    0xFF, 0x00, 0x00
#define SK6812_RED      0x00, 0xFF, 0x00
#define SK6812_BLUE     0x00, 0x00, 0x7F
#define SK6812_YELLOW   0xFF, 0xFF, 0x00
#define SK6812_PURPLE   0x00, 0xFF, 0xFF
#define SK6812_WHITE    0xFF, 0xFF, 0xFF



void sk6812_init		(void);
void sk6812_single_colour       (uint8_t Green, uint8_t Red, uint8_t Blue);
void sk6812_colour_string       (SK6812_WR_BUFFERs * GRB);
void sk6812_write_buffer        (SK6812_WR_BUFFERs * GRB, uint8_t index, uint8_t Green, uint8_t Red, uint8_t Blue);
void sk6812_single_colour_blink	(uint8_t Green, uint8_t Red, uint8_t Blue, uint16_t on_time, uint16_t off_time, uint8_t blink_count);




#ifdef __cplusplus
}
#endif


#endif
