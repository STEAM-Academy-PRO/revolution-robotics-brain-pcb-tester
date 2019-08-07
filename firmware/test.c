/*
 * test.c
 *
 * Created: 2019. 08. 06. 14:53:49
 *  Author: bugad
 */ 
#include "test.h"

#include <hal_delay.h>

void test_init(void)
{
    WS2812_Init();
    
    gpio_set_pin_direction(TEST_ENABLE, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(TEST_CHARGER_EN, GPIO_DIRECTION_OUT);

    gpio_set_pin_level(TEST_ENABLE, false);
    gpio_set_pin_level(TEST_CHARGER_EN, false);
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
    delay_ms(1);
    success &= gpio_get_pin_level(gpio) == 1u;

    return success;
}

static bool _test_gpio(uint32_t driver, uint32_t sense)
{
    return false;
}

bool test_pullups(void)
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

    return success;
}

bool test_charger(void)
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

    return success;
}

void test_sensor_ports(void)
{

}

void test_motor_ports(void)
{

}

void test_imu(void)
{

}
