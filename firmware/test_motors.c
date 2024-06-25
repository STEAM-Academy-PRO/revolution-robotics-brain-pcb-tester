#include "test.h"
#include "test_utils.h"

// TODO: pass motor struct
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
        { .dr_en = false, .pwm_0 = false, .pwm_1 = false, .current_min = 0.0f, .current_max = 0.5f},
        { .dr_en = false, .pwm_0 = false, .pwm_1 = true,  .current_min = 0.0f, .current_max = 0.5f},
        { .dr_en = false, .pwm_0 = true,  .pwm_1 = false, .current_min = 0.0f, .current_max = 0.5f},
        { .dr_en = false, .pwm_0 = true,  .pwm_1 = true,  .current_min = 0.0f, .current_max = 0.5f},

        { .dr_en = true,  .pwm_0 = false, .pwm_1 = true,  .current_min = 0.8f, .current_max = 2.0f},
        { .dr_en = true,  .pwm_0 = false, .pwm_1 = false, .current_min = 0.0f, .current_max = 0.5f},

        { .dr_en = true,  .pwm_0 = true,  .pwm_1 = false, .current_min = 0.8f, .current_max = 2.0f},
        { .dr_en = true,  .pwm_0 = true,  .pwm_1 = true,  .current_min = 0.0f, .current_max = 0.5f},
    };

    bool success = true;

    float motor_voltage = _read_analog(1u, ADC_CH_MOT_VOLTAGE) * 130.0f / 30.0f;
    float dummy_resistance = 10.0f;
    float current_measure_resistance = 0.120f;
    float nominal_current = motor_voltage / (dummy_resistance + current_measure_resistance);
    float measured_voltage = current_measure_resistance * nominal_current;

    gpio_set_pin_direction(pwm_0, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(pwm_1, GPIO_DIRECTION_OUT);

    for (uint8_t i = 0u; i < ARRAY_SIZE(cases); i++)
    {
        gpio_set_pin_level(dr_en, cases[i].dr_en);
        gpio_set_pin_level(pwm_0, cases[i].pwm_0);
        gpio_set_pin_level(pwm_1, cases[i].pwm_1);
        delay_ms(10u);

        if (!_analog_expect(isen_adc, isen_ch, cases[i].current_min * measured_voltage, cases[i].current_max * measured_voltage)) /* measured on 120mOhm resistor */
        {
            success = false;
        }

        gpio_set_pin_level(pwm_0, false);
        gpio_set_pin_level(pwm_1, false);

        delay_ms(10u);

        gpio_set_pin_level(dr_en, false);
        delay_ms(10u);
    }

    gpio_set_pin_level(dr_en, false);
    gpio_set_pin_level(pwm_0, false);
    gpio_set_pin_level(pwm_1, false);

    return success;
}

void test_enable_motor_encoder_relays(void)
{
    gpio_set_pin_level(TEST_ENABLE, true);
    delay_ms(1u);
}

typedef struct {
    const char* name;
    uint32_t led_green;
    uint32_t led_yellow;
    uint32_t driver_en;
    uint32_t enc_a;
    uint32_t enc_b;
    uint32_t pwm_0;
    uint32_t pwm_1;
    uint32_t isen_adc_peripheral;
    uint32_t isen_adc_channel;
    uint8_t result_indicator;
} motor_t;

#define DECLARE_MOTOR_PORT(n, driver, channel) \
static motor_t m##n = { \
    .name       = "Motor " #n, \
    .led_green  = M ## n ## _GREEN_LED, \
    .led_yellow = MOTOR_DRIVER_ ## driver ## _YELLOW, \
    .driver_en  = MOTOR_DRIVER_ ## driver ## _EN, \
    .enc_a      = M ## n ## _ENC_A, \
    .enc_b      = M ## n ## _ENC_B, \
    .pwm_0      = MOTOR_DRIVER_ ## driver ## _CH_ ## channel ## _PWM0_PIN, \
    .pwm_1      = MOTOR_DRIVER_ ## driver ## _CH_ ## channel ## _PWM1_PIN, \
    .isen_adc_peripheral = M ## n ## _ISEN_ADC, \
    .isen_adc_channel = M ## n ## _ISEN_CH, \
    .result_indicator = TEST_M ## n ## _LED, \
}

DECLARE_MOTOR_PORT(0, 1, A);
DECLARE_MOTOR_PORT(1, 0, A);
DECLARE_MOTOR_PORT(2, 0, B);
DECLARE_MOTOR_PORT(3, 2, A);
DECLARE_MOTOR_PORT(4, 2, B);
DECLARE_MOTOR_PORT(5, 1, B);

static motor_t* motors[] = {
    &m0,
    &m1,
    &m2,
    &m3,
    &m4,
    &m5,
};

static void init_test_motor_port(motor_t* motor)
{
    gpio_set_pin_direction(motor->driver_en, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(motor->driver_en, false);
}

static void test_motor_port(motor_t* motor)
{
    bool success = true;

    if (!_test_gpio(ENCODER_DRIVER, motor->enc_a))
    {
        SEGGER_RTT_printf(0, "%s encoder A test failed\n", motor->name);
        success = false;
    }

    if (!_test_gpio(ENCODER_DRIVER, motor->enc_b))
    {
        SEGGER_RTT_printf(0, "%s encoder B test failed\n", motor->name);
        success = false;
    }

    if (!_test_motor_driver(motor->driver_en, motor->pwm_0, motor->pwm_1, motor->isen_adc_peripheral, motor->isen_adc_channel))
    {
        SEGGER_RTT_printf(0, "%s driver test failed\n", motor->name);
        success = false;
    }

    _indicate(motor->result_indicator, success);
}

void test_motor_ports(void)
{
    for (uint8_t i = 0u; i < ARRAY_SIZE(motors); i++)
    {
        init_test_motor_port(motors[i]);
    }

    for (uint8_t i = 0u; i < ARRAY_SIZE(motors); i++)
    {
        test_motor_port(motors[i]);
    }
}
