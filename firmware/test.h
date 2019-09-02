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
#define COLOR_BLUE    ((uint8_t) 3u)

void WS2812_Init(void);
void WS2812_Write(uint8_t colors[16]);
void WS2812_SetLed(uint8_t led, uint8_t color);

void test_init(void);
void test_leds(void);
void test_pullups(void);
void test_charger(void);
void test_enable_connections(void);
void test_sensor_ports(void);
void test_motor_ports(void);
void test_supply_adc(void);
void test_imu(void);
void test_sound(void);
void test_end(void);

void imu_init(void);
uint8_t imu_read_whoami(void);
bool imu_run_selftest(void);

#endif /* TEST_H_ */
