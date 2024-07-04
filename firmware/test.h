#ifndef TEST_H_
#define TEST_H_

#include <hal_delay.h>

#include "atmel_start_pins.h"
#include <peripheral_clk_config.h>

#include <math.h>
#include "utils.h"
#include "SEGGER_RTT.h"

#define COLOR_OFF     ((uint8_t) 0u)
#define COLOR_RED     ((uint8_t) 1u)
#define COLOR_GREEN   ((uint8_t) 2u)
#define COLOR_BLUE    ((uint8_t) 3u)

/* Visual outputs */
#define TEST_PULLUP_LED         ((uint8_t) 0u)
#define TEST_CHARGER_LED        ((uint8_t) 1u)
#define TEST_S0_LED             ((uint8_t) 2u)
#define TEST_S1_LED             ((uint8_t) 3u)
#define TEST_S2_LED             ((uint8_t) 4u)
#define TEST_S3_LED             ((uint8_t) 5u)
#define TEST_M0_LED             ((uint8_t) 6u)
#define TEST_M1_LED             ((uint8_t) 7u)
#define TEST_M2_LED             ((uint8_t) 8u)
#define TEST_M3_LED             ((uint8_t) 9u)
#define TEST_M4_LED             ((uint8_t) 10u)
#define TEST_M5_LED             ((uint8_t) 11u)
#define TEST_IMU_LED            ((uint8_t) 12u)
#define TEST_BATTERY_ADC_LED    ((uint8_t) 13u)
#define TEST_MOTOR_ADC_LED      ((uint8_t) 14u)
#define TEST_TEST_DONE_LED      ((uint8_t) 15u)

void test_init(void);
void test_leds(void);
void test_pullups(void);
void test_charger(void);
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
