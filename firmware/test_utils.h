#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

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

typedef struct {
    uint32_t pin;
    const char* name;
} gpio_t;


void reset_result(void);
bool get_result(void);

void adc_init(void);

void WS2812_Init(void);
void WS2812_Write(uint8_t colors[16]);
void WS2812_SetLed(uint8_t led, uint8_t color);

void _indicate(uint8_t led, bool success);
bool _test_gpio(gpio_t* driver, gpio_t* sense);

float map(float in, float min_in, float max_in, float min_out, float max_out);
float _read_analog(uint32_t adc, uint32_t ch);
bool _sysmon_analog_expect(uint32_t adc, uint32_t ch, float lower, float upper, float divider);

#endif /* TEST_UTILS_H_ */
