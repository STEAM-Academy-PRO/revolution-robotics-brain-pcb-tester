/*
 * test.h
 *
 * Common include file for the business logic and drivers
 *
 * Created: 2019. 08. 06. 11:30:13
 *  Author: Dániel Buga
 */ 


#ifndef TEST_H_
#define TEST_H_

#include "atmel_start_pins.h"
#include <peripheral_clk_config.h>

#define COLOR_OFF     ((uint8_t) 0u)
#define COLOR_RED     ((uint8_t) 1u)
#define COLOR_GREEN   ((uint8_t) 2u)

void WS2812_Init(void);
void WS2812_Write(uint8_t colors[16]);
void WS2812_SetLed(uint8_t led, uint8_t color);

#endif /* TEST_H_ */
