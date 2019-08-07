/*
 * test.c
 *
 * Created: 2019. 08. 06. 14:53:49
 *  Author: bugad
 */ 
#include "test.h"

#include <hal_delay.h>

#define TEST_PULLUP_LED   ((uint8_t) 0u)
#define TEST_CHARGER_LED  ((uint8_t) 1u)
#define TEST_S0_LED       ((uint8_t) 2u)
#define TEST_S1_LED       ((uint8_t) 3u)
#define TEST_S2_LED       ((uint8_t) 4u)
#define TEST_S3_LED       ((uint8_t) 5u)

static void _indicate(uint8_t led, bool success)
{
    if (success)
    {
        WS2812_SetLed(led, COLOR_GREEN);
    }
    else
    {
        WS2812_SetLed(led, COLOR_RED);
    }
}

static bool _test_pullup(uint32_t gpio)
{
    /* by default they should be pulled up */
    gpio_set_pin_direction(gpio, GPIO_DIRECTION_IN);
    bool success = gpio_get_pin_level(gpio) == 1u;

    /* pull them down and release them */
    gpio_set_pin_direction(gpio, GPIO_DIRECTION_INOUT);
    gpio_set_pin_level(TEST_ENABLE, false);
    success &= gpio_get_pin_level(gpio) == 0u;
    delay_ms(1);
    
    gpio_set_pin_direction(gpio, GPIO_DIRECTION_IN);
    delay_ms(1u);
    success &= gpio_get_pin_level(gpio) == 1u;

    return success;
}

static float _read_analog(uint32_t adc, uint32_t ch)
{
    delay_ms(1u);
    return 0.0f;
}

static bool _analog_expect_0v(uint32_t adc, uint32_t ch)
{
    float voltage = _read_analog(adc, ch);
    
    return -0.1f < voltage && voltage < 0.1f;
}

static bool _analog_expect_3v3(uint32_t adc, uint32_t ch)
{
    float voltage = _read_analog(adc, ch);
    
    return 3.2f < voltage && voltage < 3.4f;
}

static bool _analog_expect_5v(uint32_t adc, uint32_t ch)
{
    float voltage = _read_analog(adc, ch);
    
    return 4.9f < voltage && voltage < 5.1f;
}

static bool _test_gpio(uint32_t driver, uint32_t sense)
{
    bool success = true;

    gpio_set_pin_direction(driver, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(sense, GPIO_DIRECTION_IN);
    
    gpio_set_pin_level(driver, true);
    delay_ms(1u);
    success &= gpio_get_pin_level(sense) == 1u;

    gpio_set_pin_level(driver, false);
    delay_ms(1u);
    success &= gpio_get_pin_level(sense) == 0u;

    gpio_set_pin_direction(driver, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(sense, GPIO_DIRECTION_OFF);

    return success;
}

static bool _test_analog(uint32_t driver, uint32_t iovcc, uint32_t adc, uint32_t adc_ch)
{
    bool success = true;

    gpio_set_pin_direction(driver, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(iovcc, GPIO_DIRECTION_OUT);

    /* test 3V3 */
    gpio_set_pin_level(iovcc, false);

    gpio_set_pin_level(driver, true);
    success &= _analog_expect_3v3(adc, adc_ch);
    
    gpio_set_pin_level(driver, false);
    success &= _analog_expect_0v(adc, adc_ch);

    /* test 5V */
    gpio_set_pin_level(iovcc, true);
    
    gpio_set_pin_level(driver, true);
    success &= _analog_expect_5v(adc, adc_ch);
    
    gpio_set_pin_level(driver, false);
    success &= _analog_expect_0v(adc, adc_ch);

    /* reset to idle */
    gpio_set_pin_level(iovcc, false);
    gpio_set_pin_direction(driver, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(iovcc, GPIO_DIRECTION_OFF);

    return success;
}

void test_init(void)
{
    WS2812_Init();
    
    gpio_set_pin_direction(TEST_ENABLE, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(TEST_CHARGER_EN, GPIO_DIRECTION_OUT);

    gpio_set_pin_level(TEST_ENABLE, false);
    gpio_set_pin_level(TEST_CHARGER_EN, false);
    
    gpio_set_pin_direction(S0_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S1_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S2_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S3_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(S0_IOVCC, false);
    gpio_set_pin_level(S1_IOVCC, false);
    gpio_set_pin_level(S2_IOVCC, false);
    gpio_set_pin_level(S3_IOVCC, false);
}

void test_pullups(void)
{
    bool success = true;
    
    success &= _test_pullup(AMP_EN_sense);

    success &= _test_pullup(S0_GPIO_IN);
    success &= _test_pullup(S0_GPIO_OUT);
    success &= _test_pullup(S1_GPIO_IN);
    success &= _test_pullup(S1_GPIO_OUT);
    success &= _test_pullup(S2_GPIO_IN);
    success &= _test_pullup(S2_GPIO_OUT);
    success &= _test_pullup(S3_GPIO_IN);
    success &= _test_pullup(S3_GPIO_OUT);
    
    success &= _test_pullup(I2C0_SDApin);
    success &= _test_pullup(I2C0_SCLpin);
    success &= _test_pullup(I2C1_SDApin);
    success &= _test_pullup(I2C1_SCLpin);
    success &= _test_pullup(I2C2_SDApin);
    success &= _test_pullup(I2C2_SCLpin);
    success &= _test_pullup(I2C3_SDApin);
    success &= _test_pullup(I2C3_SCLpin);
    
    success &= _test_pullup(M0_ENC_A);
    success &= _test_pullup(M0_ENC_B);
    success &= _test_pullup(M1_ENC_A);
    success &= _test_pullup(M1_ENC_B);
    success &= _test_pullup(M2_ENC_A);
    success &= _test_pullup(M2_ENC_B);
    success &= _test_pullup(M3_ENC_A);
    success &= _test_pullup(M3_ENC_B);
    success &= _test_pullup(M4_ENC_A);
    success &= _test_pullup(M4_ENC_B);
    success &= _test_pullup(M5_ENC_A);
    success &= _test_pullup(M5_ENC_B);
    
    success &= _test_pullup(M0_GREEN_LED);
    success &= _test_pullup(M1_GREEN_LED);
    success &= _test_pullup(M2_GREEN_LED);
    success &= _test_pullup(M3_GREEN_LED);
    success &= _test_pullup(M4_GREEN_LED);
    success &= _test_pullup(M5_GREEN_LED);
    success &= _test_pullup(MOTOR_DRIVER_0_YELLOW);
    success &= _test_pullup(MOTOR_DRIVER_1_YELLOW);
    success &= _test_pullup(MOTOR_DRIVER_2_YELLOW);
    success &= _test_pullup(S0_LED_GREEN);
    success &= _test_pullup(S1_LED_GREEN);
    success &= _test_pullup(S2_LED_GREEN);
    success &= _test_pullup(S3_LED_GREEN);
    success &= _test_pullup(S0_LED_YELLOW);
    success &= _test_pullup(S1_LED_YELLOW);
    success &= _test_pullup(S2_LED_YELLOW);
    success &= _test_pullup(S3_LED_YELLOW);

    /* internal pullups, skip */
    // success &= _test_pullup(CHARGER_STAT);
    // success &= _test_pullup(CHARGER_STBY);
    
    _indicate(TEST_PULLUP_LED, success);
}

void test_charger(void)
{
    /* configure pins */
    gpio_set_pin_direction(CHARGER_STAT, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(CHARGER_STAT, GPIO_PULL_UP);
    gpio_set_pin_function(CHARGER_STAT, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_direction(CHARGER_STBY, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(CHARGER_STBY, GPIO_PULL_UP);
    gpio_set_pin_function(CHARGER_STBY, GPIO_PIN_FUNCTION_OFF);

    /* execute test */
    gpio_set_pin_level(TEST_CHARGER_EN, false);

    bool success = true;
    success &= gpio_get_pin_level(CHARGER_STAT) == 1u;
    success &= gpio_get_pin_level(CHARGER_STBY) == 1u;
    
    gpio_set_pin_level(TEST_CHARGER_EN, true);

    delay_ms(1u);

    bool pin_changed;
    pin_changed = gpio_get_pin_level(CHARGER_STAT) == 0u;
    pin_changed |= gpio_get_pin_level(CHARGER_STBY) == 0u;

    success &= pin_changed;

    /* reset pins */
    gpio_set_pin_level(TEST_CHARGER_EN, false);

    gpio_set_pin_direction(CHARGER_STBY, GPIO_DIRECTION_OFF);
    gpio_set_pin_pull_mode(CHARGER_STBY, GPIO_PULL_OFF);
    gpio_set_pin_direction(CHARGER_STBY, GPIO_DIRECTION_OFF);
    gpio_set_pin_pull_mode(CHARGER_STBY, GPIO_PULL_OFF);

    _indicate(TEST_CHARGER_LED, success);
}

void test_enable_connections(void)
{
    gpio_set_pin_level(TEST_ENABLE, true);
    delay_ms(1u);
}

void test_sensor_ports(void)
{
    /* AIN pin is always connected to analog functions, no need to manually enable */
    bool s0_result = true;
    s0_result &= _test_gpio(S0_GPIO_IN, I2C0_SDApin);
    s0_result &= _test_gpio(I2C0_SDApin, S0_GPIO_IN);
    s0_result &= _test_gpio(S0_GPIO_OUT, I2C0_SCLpin);
    s0_result &= _test_gpio(I2C0_SCLpin, S0_GPIO_OUT);
    s0_result &= _test_analog(S0_GPIO_OUT, S0_IOVCC, S0_ADC_PER, S0_ADC_CH);
    _indicate(TEST_S0_LED, s0_result);

    bool s1_result = true;
    s1_result &= _test_gpio(S1_GPIO_IN, I2C1_SDApin);
    s1_result &= _test_gpio(I2C1_SDApin, S1_GPIO_IN);
    s1_result &= _test_gpio(S1_GPIO_OUT, I2C1_SCLpin);
    s1_result &= _test_gpio(I2C1_SCLpin, S1_GPIO_OUT);
    s1_result &= _test_analog(S1_GPIO_OUT, S1_IOVCC, S1_ADC_PER, S1_ADC_CH);
    _indicate(TEST_S1_LED, s1_result);
    
    bool s2_result = true;
    s2_result &= _test_gpio(S2_GPIO_IN, I2C2_SDApin);
    s2_result &= _test_gpio(I2C2_SDApin, S2_GPIO_IN);
    s2_result &= _test_gpio(S2_GPIO_OUT, I2C2_SCLpin);
    s2_result &= _test_gpio(I2C2_SCLpin, S2_GPIO_OUT);
    s2_result &= _test_analog(S2_GPIO_OUT, S2_IOVCC, S2_ADC_PER, S2_ADC_CH);
    _indicate(TEST_S2_LED, s2_result);
    
    bool s3_result = true;
    s3_result &= _test_gpio(S3_GPIO_IN, I2C3_SDApin);
    s3_result &= _test_gpio(I2C3_SDApin, S3_GPIO_IN);
    s3_result &= _test_gpio(S3_GPIO_OUT, I2C3_SCLpin);
    s3_result &= _test_gpio(I2C3_SCLpin, S3_GPIO_OUT);
    s3_result &= _test_analog(S3_GPIO_OUT, S3_IOVCC, S3_ADC_PER, S3_ADC_CH);
    _indicate(TEST_S3_LED, s3_result);
}

void test_motor_ports(void)
{

}

void test_imu(void)
{

}
