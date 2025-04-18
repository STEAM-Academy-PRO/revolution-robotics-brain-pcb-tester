#include "test.h"
#include "test_utils.h"

typedef struct {
    const char* name;
    gpio_t led_green;
    gpio_t led_yellow;
    uint32_t driver_en;
    gpio_t enc_a;
    gpio_t enc_b;
    uint32_t pwm_0;
    uint32_t pwm_1;
    uint32_t isen_adc_peripheral;
    uint32_t isen_adc_channel;
    uint8_t result_indicator;
} motor_t;

#define DECLARE_MOTOR_PORT(n, driver, channel) \
static motor_t m##n = { \
    .name       = "Motor " #n, \
    .led_green  = { .pin = M ## n ## _GREEN_LED, .name = "LED_GREEN" }, \
    .led_yellow = { .pin = MOTOR_DRIVER_ ## driver ## _YELLOW, .name = "LED_YELLOW" }, \
    .driver_en  = MOTOR_DRIVER_ ## driver ## _EN, \
    .enc_a      = { .pin = M ## n ## _ENC_A, .name = "MOT_INT0" }, \
    .enc_b      = { .pin = M ## n ## _ENC_B, .name = "MOT_INT1" }, \
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

static const motor_t* motors[] = {
    &m0,
    &m1,
    &m2,
    &m3,
    &m4,
    &m5,
};

static bool _analog_expect(uint32_t adc, uint32_t ch, float lower, float upper)
{
    return _sysmon_analog_expect(adc, ch, lower, upper, 1.0f);
}

static bool _test_motor_driver(const motor_t* motor)
{
    struct test_case {
        bool dr_en;
        bool pwm_0;
        bool pwm_1;
        float current_min;
        float current_max;
    };

    // TODO: double check current values - do they contain a factor that should be removed
    // now that _analog_expect doesn't have a divider?
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

    // Compare the measured current to the value calculated
    // from input voltage and nominal resistance values.
    float motor_input_voltage = _read_analog(1u, ADC_CH_MOT_VOLTAGE) * 130.0f / 30.0f;
    float dummy_resistance = 10.0f;
    float current_measure_resistance = 0.120f;
    float nominal_current = motor_input_voltage / (dummy_resistance + current_measure_resistance);
    float measured_voltage = current_measure_resistance * nominal_current;

    gpio_set_pin_direction(motor->pwm_0, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(motor->pwm_1, GPIO_DIRECTION_OUT);

    for (uint8_t i = 0u; i < ARRAY_SIZE(cases); i++)
    {
        gpio_set_pin_level(motor->driver_en, cases[i].dr_en);
        gpio_set_pin_level(motor->pwm_0, cases[i].pwm_0);
        gpio_set_pin_level(motor->pwm_1, cases[i].pwm_1);
        delay_ms(10u);

        if (!_analog_expect(motor->isen_adc_peripheral, motor->isen_adc_channel, cases[i].current_min * measured_voltage, cases[i].current_max * measured_voltage)) /* measured on 120mOhm resistor */
        {
            success = false;
        }

        gpio_set_pin_level(motor->pwm_0, false);
        gpio_set_pin_level(motor->pwm_1, false);

        delay_ms(10u);

        gpio_set_pin_level(motor->driver_en, false);
        delay_ms(10u);
    }

    gpio_set_pin_level(motor->driver_en, false);
    gpio_set_pin_level(motor->pwm_0, false);
    gpio_set_pin_level(motor->pwm_1, false);

    return success;
}

static void test_enable_motor_encoder_relays(void)
{
    gpio_set_pin_level(TEST_ENABLE, true);
    delay_ms(1u);
}

static void test_disable_motor_encoder_relays(void)
{
    gpio_set_pin_level(TEST_ENABLE, false);
    delay_ms(1u);
}

static bool _test_motor_pullups(const motor_t* motor)
{
    const gpio_t* pins[4] = {
        &motor->enc_a,
        &motor->enc_b,
        &motor->led_green,
        &motor->led_yellow,
    };

    return _test_pullups(motor->name, pins, ARRAY_SIZE(pins), "");
}

static bool _test_motor_inputs_for_shorts(const motor_t* motors[], uint8_t num_motors, uint8_t motor_idx)
{
    bool success = true;

    // We're pulling each of the encoder input pins down, one after the other.
    // Then we verify that no other pins, that are pulled high by default, are pulled low.
    // TODO: the driver is off, so we might be able to test other pins as well.
    const motor_t* motor = motors[motor_idx];
    for (uint8_t i = 0u; i < 2; i++)
    {
        const gpio_t* output_pin = i == 0 ? &motor->enc_a : &motor->enc_b;
        const gpio_t* sense_pin = i == 0 ? &motor->enc_b : &motor->enc_a;
        gpio_set_pin_direction(output_pin->pin, GPIO_DIRECTION_OUT);
        gpio_set_pin_level(output_pin->pin, false);

        delay_ms(1u);

        // Test against the other input of the same motor port
        const gpio_t* sense_pins[3] = {
            sense_pin,
            &motor->led_green,
            &motor->led_yellow,
        };

        success &= _assert_pins_high_for_short(output_pin, sense_pins, ARRAY_SIZE(sense_pins), motor->name, motor->name);

        // Test against both inputs of all the other motor ports
        for (uint8_t j = 0u; j < num_motors; j++)
        {
            // Skip the motor we're testing
            if (j == motor_idx)
            {
                continue;
            }

            const motor_t* sense_motor = motors[j];
            const gpio_t* sense_pins[4] = {
                &sense_motor->enc_a,
                &sense_motor->enc_b,
                &sense_motor->led_green,
                &sense_motor->led_yellow,
            };

            success &= _assert_pins_high_for_short(output_pin, sense_pins, ARRAY_SIZE(sense_pins), motor->name, sense_motor->name);
        }

        gpio_set_pin_direction(output_pin->pin, GPIO_DIRECTION_OFF);
    }

    return success;
}

static void init_test_motor_port(const motor_t* motor)
{
    gpio_set_pin_direction(motor->driver_en, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(motor->driver_en, false);
}

static void test_motor_ports_unconnected(void)
{
    bool success = true;

    // Test pullups while the connections are broken
    for (uint8_t i = 0u; i < ARRAY_SIZE(motors); i++)
    {
        success &= _test_motor_pullups(motors[i]);
        _indicate(motors[i]->result_indicator, success);
    }

    // TODO: test for short circuits
    for (uint8_t i = 0u; i < ARRAY_SIZE(motors); i++)
    {
        success &= _test_motor_inputs_for_shorts(motors, ARRAY_SIZE(motors), i);
        _indicate(motors[i]->result_indicator, success);
    }
}

static void test_motor_port(const motor_t* motor)
{
    bool success = true;

    gpio_t encoder_driver = {
        .pin = ENCODER_DRIVER,
        .name = "PB18",
    };

    // TODO: these failures mean that a series resistor (e.g. R7) is faulty. We have tested the
    // rest in the pullup tests. Testing separately, and not ENC_A to ENC_B means we can tell
    // which resistor.
    if (!_test_gpio(&encoder_driver, &motor->enc_a))
    {
        success = false;
    }

    if (!_test_gpio(&encoder_driver, &motor->enc_b))
    {
        success = false;
    }

    if (!_test_motor_driver(motor))
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

    test_motor_ports_unconnected();

    // Connect encoder lines
    test_enable_motor_encoder_relays();

    for (uint8_t i = 0u; i < ARRAY_SIZE(motors); i++)
    {
        test_motor_port(motors[i]);
    }

    // Disconnect encoder lines
    test_disable_motor_encoder_relays();
}
