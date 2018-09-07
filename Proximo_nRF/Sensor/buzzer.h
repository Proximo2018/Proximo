/*
 */
#ifndef BUZZER_H
#define BUZZER_H


#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdint.h>

#define BUZZ_RESONANT_FREQ 3100 // On 3100 Hz the Buzzer resonates

void Buzz(uint8_t dutycycle, uint16_t frequency);


#ifdef __cplusplus
}
#endif


#endif
