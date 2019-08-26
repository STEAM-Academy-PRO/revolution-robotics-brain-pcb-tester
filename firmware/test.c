/*
 * test.c
 *
 * Created: 2019. 08. 06. 14:53:49
 *  Author: bugad
 */ 
#include "test.h"

#include <hal_delay.h>
#include <hal_adc_sync.h>

#include "utils.h"

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

static struct adc_sync_descriptor adcs[2];

static float map(float in, float min_in, float max_in, float min_out, float max_out)
{
    float in_fs = max_in - min_in;
    float out_fs = max_out - min_out;

    return (in - min_in) * (out_fs / in_fs) + min_out;
}

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
    uint16_t buffer[8];
    delay_ms(1u);
    adc_sync_set_inputs(&adcs[adc], ch, ADC_CHN_INT_GND, 0u);
    adc_sync_read_channel(&adcs[adc], 0u, (uint8_t*) buffer, sizeof(buffer));
    uint16_t sum = 0u;
    for (uint8_t i = 0u; i < ARRAY_SIZE(buffer); i++)
    {
        sum += buffer[i];
    }
    sum = sum / ARRAY_SIZE(buffer);

    return map(sum, 0, 4095, 0, 3300); /* Vref = VDDANA */
}

static bool _analog_expect(uint32_t adc, uint32_t ch, float lower, float upper)
{
    float voltage = _read_analog(adc, ch) * (250.0f / 150.0f); /* compensate for voltage divider */
    
    return lower < voltage && voltage < upper;
}

static bool _sysmon_analog_expect(uint32_t adc, uint32_t ch, float lower, float upper, float divider)
{
    float voltage = _read_analog(adc, ch) / divider;
    
    return lower < voltage && voltage < upper;
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
    success &= _analog_expect(adc, adc_ch, 3.1f, 3.5f);
    
    gpio_set_pin_level(driver, false);
    success &= _analog_expect(adc, adc_ch, 0.0f, 0.2f);

    /* test 5V */
    gpio_set_pin_level(iovcc, true);
    
    gpio_set_pin_level(driver, true);
    success &= _analog_expect(adc, adc_ch, 4.2f, 5.2f);
    
    gpio_set_pin_level(driver, false);
    success &= _analog_expect(adc, adc_ch, 0.0f, 0.2f);

    /* reset to idle */
    gpio_set_pin_level(iovcc, false);
    gpio_set_pin_direction(driver, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(iovcc, GPIO_DIRECTION_OFF);

    return success;
}

static bool _test_motor_driver(uint32_t dr_en, uint32_t pwm_0, uint32_t pwm_1, uint32_t isen_adc, uint32_t isen_ch)
{
    struct test_case {
        bool dr_en;
        bool pwm_0;
        bool pwm_1;
        float current_min;
        float current_max;
    };
    
    static const struct test_case cases[] = {
        { false, false, false, 0.0f, 0.1f},
        { false, false, true,  0.0f, 0.1f},
        { false, true,  false, 0.0f, 0.1f},
        { false, true,  true,  0.0f, 0.1f},
        
        { true, false, true,  0.9f, 1.1f},
        { true, false, false, 0.0f, 0.1f},
        { true, true,  false, 0.9f, 1.1f},
        { true, true,  true,  0.0f, 0.1f}
    };

    bool success = true;
    
    gpio_set_pin_direction(dr_en, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(pwm_0, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(pwm_1, GPIO_DIRECTION_OUT);
    
    for (uint8_t i = 0u; i < ARRAY_SIZE(cases); i++)
    {
        gpio_set_pin_level(dr_en, cases[i].dr_en);
        gpio_set_pin_level(pwm_0, cases[i].pwm_0);
        gpio_set_pin_level(pwm_1, cases[i].pwm_1);
    
        delay_ms(1u);

        success &= _analog_expect(isen_adc, isen_ch, cases[i].current_min * 0.120f, cases[i].current_max * 0.120f); /* measured on 120mOhm resistor */
    }

    gpio_set_pin_level(dr_en, false);
    gpio_set_pin_level(pwm_0, false);
    gpio_set_pin_level(pwm_1, false);
    
    gpio_set_pin_direction(dr_en, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(pwm_0, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(pwm_1, GPIO_DIRECTION_OFF);

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
    
    hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
    hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    adc_sync_init(&adcs[0], ADC0);
    adc_sync_init(&adcs[1], ADC1);
    
    adc_sync_enable_channel(&adcs[0], 0u);
    adc_sync_enable_channel(&adcs[1], 0u);

    imu_init();
}

void test_leds(void)
{
    for (uint8_t led = 0u; led < 16u; led++)
    {
        WS2812_SetLed(led, COLOR_GREEN);
    }

    delay_ms(1000u);

    for (uint8_t led = 0u; led < 16u; led++)
    {
        WS2812_SetLed(led, COLOR_RED);
    }
    
    delay_ms(1000u);

    for (uint8_t led = 0u; led < 16u; led++)
    {
        WS2812_SetLed(led, COLOR_BLUE);
    }

    delay_ms(1000u);
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
    //#define M0_DRIVER_IDX       1
    //#define M0_DRIVER_CHANNEL   DRV8833_Channel_A
    bool m0_result = true;
    m0_result &= _test_gpio(ENCODER_DRIVER, M0_ENC_A);
    m0_result &= _test_gpio(ENCODER_DRIVER, M0_ENC_B);
    m0_result &= _test_motor_driver(MOTOR_DRIVER_1_EN, MOTOR_DRIVER_1_CH_A_PWM0_PIN, MOTOR_DRIVER_1_CH_A_PWM0_PIN, M0_ISEN_ADC, M0_ISEN_CH);
    _indicate(TEST_M0_LED, m0_result);
    
    //#define M1_DRIVER_IDX       0
    //#define M1_DRIVER_CHANNEL   DRV8833_Channel_A
    bool m1_result = true;
    m1_result &= _test_gpio(ENCODER_DRIVER, M1_ENC_A);
    m1_result &= _test_gpio(ENCODER_DRIVER, M1_ENC_B);
    m1_result &= _test_motor_driver(MOTOR_DRIVER_0_EN, MOTOR_DRIVER_0_CH_A_PWM0_PIN, MOTOR_DRIVER_0_CH_A_PWM0_PIN, M1_ISEN_ADC, M1_ISEN_CH);
    _indicate(TEST_M1_LED, m1_result);
    
    //#define M2_DRIVER_IDX       0
    //#define M2_DRIVER_CHANNEL   DRV8833_Channel_B
    bool m2_result = true;
    m2_result &= _test_gpio(ENCODER_DRIVER, M2_ENC_A);
    m2_result &= _test_gpio(ENCODER_DRIVER, M2_ENC_B);
    m2_result &= _test_motor_driver(MOTOR_DRIVER_0_EN, MOTOR_DRIVER_0_CH_B_PWM0_PIN, MOTOR_DRIVER_0_CH_B_PWM0_PIN, M2_ISEN_ADC, M2_ISEN_CH);
    _indicate(TEST_M2_LED, m2_result);
    
    //#define M3_DRIVER_IDX       2
    //#define M3_DRIVER_CHANNEL   DRV8833_Channel_A
    bool m3_result = true;
    m3_result &= _test_gpio(ENCODER_DRIVER, M3_ENC_A);
    m3_result &= _test_gpio(ENCODER_DRIVER, M3_ENC_B);
    m3_result &= _test_motor_driver(MOTOR_DRIVER_2_EN, MOTOR_DRIVER_2_CH_A_PWM0_PIN, MOTOR_DRIVER_2_CH_A_PWM0_PIN, M3_ISEN_ADC, M3_ISEN_CH);
    _indicate(TEST_M3_LED, m3_result);
    
    //#define M4_DRIVER_IDX       2
    //#define M4_DRIVER_CHANNEL   DRV8833_Channel_B
    bool m4_result = true;
    m4_result &= _test_gpio(ENCODER_DRIVER, M4_ENC_A);
    m4_result &= _test_gpio(ENCODER_DRIVER, M4_ENC_B);
    m4_result &= _test_motor_driver(MOTOR_DRIVER_2_EN, MOTOR_DRIVER_2_CH_B_PWM0_PIN, MOTOR_DRIVER_2_CH_B_PWM0_PIN, M4_ISEN_ADC, M4_ISEN_CH);
    _indicate(TEST_M4_LED, m4_result);
    
    //#define M5_DRIVER_IDX       1
    //#define M5_DRIVER_CHANNEL   DRV8833_Channel_B
    bool m5_result = true;
    m5_result &= _test_gpio(ENCODER_DRIVER, M5_ENC_A);
    m5_result &= _test_gpio(ENCODER_DRIVER, M5_ENC_B);
    m5_result &= _test_motor_driver(MOTOR_DRIVER_1_EN, MOTOR_DRIVER_1_CH_B_PWM0_PIN, MOTOR_DRIVER_1_CH_B_PWM0_PIN, M5_ISEN_ADC, M5_ISEN_CH);
    _indicate(TEST_M5_LED, m5_result);
}

void test_imu(void)
{
    bool success = true;
    
    success &= imu_read_whoami() == IMU_EXPECTED_WHOAMI;
    success &= imu_run_selftest();
    
    _indicate(TEST_IMU_LED, success);
}

void test_supply_adc(void)
{
    bool success;

    /* TODO: these may be more strict */
    success = _sysmon_analog_expect(1u, ADC_CH_BAT_VOLTAGE, 3000.0f, 4300.0f, 240.0f/340.0f );
    _indicate(TEST_BATTERY_ADC_LED, success);
    
    success = _sysmon_analog_expect(1u, ADC_CH_MOT_VOLTAGE, 9000.0f, 11000.0f, 30.0f/130.0f );
    _indicate(TEST_MOTOR_ADC_LED, success);

    _indicate(TEST_TEST_DONE_LED, true);
}
