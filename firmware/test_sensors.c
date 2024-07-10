#include "test.h"
#include "test_utils.h"

typedef struct {
    const char* name;
    uint32_t iovcc;
    uint32_t ain;
    uint32_t adc_per;
    uint32_t adc_ch;
    gpio_t gpio_out;
    gpio_t gpio_in;
    gpio_t sda;
    gpio_t scl;
    gpio_t led_green;
    gpio_t led_yellow;
    uint8_t result_indicator;
} sensor_t;

#define DECLARE_SENSOR_PORT(n) \
static sensor_t s##n = { \
    .name = "Sensor " #n, \
    .iovcc = S ## n ## _IOVCC, \
    .ain = S ## n ## _AIN, \
    .adc_per = S ## n ## _ADC_PER, \
    .adc_ch = S ## n ## _ADC_CH, \
    .gpio_out = (gpio_t) { .pin = S ## n ## _GPIO_OUT, .name = "GPIO_OUT" }, \
    .gpio_in = (gpio_t) { .pin = S ## n ## _GPIO_IN, .name = "GPIO_IN" }, \
    .sda = (gpio_t) { .pin = I2C ## n ## _SDApin, .name = "SDA" }, \
    .scl = (gpio_t) { .pin = I2C ## n ## _SCLpin, .name = "SCL" }, \
    .led_green = (gpio_t) { .pin = S ## n ## _LED_GREEN, .name = "LED_GREEN" }, \
    .led_yellow = (gpio_t) { .pin = S ## n ## _LED_YELLOW, .name = "LED_YELLOW" }, \
    .result_indicator = TEST_S ## n ## _LED, \
}

DECLARE_SENSOR_PORT(0);
DECLARE_SENSOR_PORT(1);
DECLARE_SENSOR_PORT(2);
DECLARE_SENSOR_PORT(3);

static const sensor_t* sensors[] = {
    &s0,
    &s1,
    &s2,
    &s3,
};

static bool _analog_expect(uint32_t adc, uint32_t ch, float lower, float upper)
{
    return _sysmon_analog_expect(adc, ch, lower, upper, 150.0f / 250.0f);
}

/**
 * Sensor port AIN tests.
 *
 * This test drives a sensor port digital pin with both level shifter settings:
 * 3V3 and 5V. The AIN pin is connected to the digital pin and the voltage is
 * measured with the ADC.
 */
static bool _test_analog(uint32_t driver, uint32_t iovcc, uint32_t adc, uint32_t adc_ch)
{
    bool success = true;

    gpio_set_pin_direction(driver, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(iovcc, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(driver, true);

    /* test 3V3 */
    gpio_set_pin_level(iovcc, false);
    delay_ms(10u);
    if (!_analog_expect(adc, adc_ch, 3000.0f, 3500.0f))
    {
        success = false;
    }

    // TODO: why are these commented?
    /*gpio_set_pin_level(driver, false);
    if (!_analog_expect(adc, adc_ch, 0.0f, 200.0f))
    {
        success = false;
    }*/

    /* test 5V */
    gpio_set_pin_level(iovcc, true);
    delay_ms(10u);

    if (!_analog_expect(adc, adc_ch, 4200.0f, 5200.0f))
    {
        success = false;
    }

    /*gpio_set_pin_level(driver, false);
    if (!_analog_expect(adc, adc_ch, 0.0f, 200.0f))
    {
        success = false;
    }*/

    /* reset to idle */
    gpio_set_pin_level(iovcc, false);
    gpio_set_pin_direction(driver, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(iovcc, GPIO_DIRECTION_OFF);

    return success;
}

void init_test_sensor_port(const sensor_t* sensor)
{
    gpio_set_pin_direction(sensor->led_green.pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(sensor->led_green.pin, false);
}

static bool test_sensor_pullups(const sensor_t* sensor)
{
    bool success = true;

    // IN pins are connected to AIN and SCL.
    // AIN has a 100K series resistor and a 150K pulldown.
    // IN and SCL are passing through a level shifter. The level shifter has internal 10K pullups.
    // AIN is tested separately during sensor port testing.
    // OUT pins are connected to SDA.
    // OUT and SDA are passing through the same level shifter as above.
    typedef struct {
        bool level;
        const char* message;
    } test_case_t;

    const test_case_t tests[2] = {
        { .level = false, .message = " with 3.3V" },
        { .level = true,  .message = " with 5V" },
    };

    gpio_set_pin_direction(sensor->iovcc, GPIO_DIRECTION_OUT);
    for (size_t i = 0; i < 2; i++)
    {
        const test_case_t* test = &tests[i];

        gpio_set_pin_level(sensor->iovcc, test->level);

        delay_ms(10u);

        const gpio_t* pins[4] = {
            &sensor->gpio_in,
            &sensor->gpio_out,
            &sensor->sda,
            &sensor->scl,
        };

        success &= _test_pullups(sensor->name, pins, ARRAY_SIZE(pins), test->message);
    }
    gpio_set_pin_level(sensor->iovcc, false);
    gpio_set_pin_direction(sensor->iovcc, GPIO_DIRECTION_OFF);

    // Sensor port green LEDs have a series resistor (R1xx) and a pullup (R4x). The pullup ensures
    // that the sensor port output power load switches are disabled by default.
    // These tests also ensure that the RJ45 ports are placed.
    const gpio_t* pins[2] = {
        &sensor->led_green,
        &sensor->led_yellow,
    };

    return _test_pullups(sensor->name, pins, ARRAY_SIZE(pins), "");
}

static bool _test_sensor_gpio_short(const sensor_t* sensors[], uint8_t num_sensors, uint8_t sensor_idx)
{
    bool success = true;

    // We're pulling each of the driver pins down, one after the other.
    // Then we verify that no other pins, that are pulled high by default, are pulled low.
    const sensor_t* sensor = sensors[sensor_idx];
    for (uint8_t i = 0u; i < 2; i++)
    {
        // Since IN is shorted to SDA in the test jig, it makes no sense to test it for shorts.
        // Instead, we test IN to SCL and then OUT to SDA.
        const gpio_t* output_pin = i == 0 ? &sensor->gpio_in : &sensor->gpio_out;
        const gpio_t* sense_pin = i == 0 ? &sensor->scl : &sensor->sda;
        gpio_set_pin_direction(output_pin->pin, GPIO_DIRECTION_OUT);
        gpio_set_pin_level(output_pin->pin, false);

        delay_ms(1u);

        // Test against the other input of the same motor port
        const gpio_t* sense_pins[3] = {
            sense_pin,
            &sensor->led_green,
            &sensor->led_yellow,
        };

        success &= _assert_pins_high_for_short(output_pin, sense_pins, ARRAY_SIZE(sense_pins), sensor->name, sensor->name);

        // Test against both inputs of all the other motor ports
        for (uint8_t j = 0u; j < num_sensors; j++)
        {
            // Skip the motor we're testing
            if (j == sensor_idx)
            {
                continue;
            }

            const sensor_t* sense_sensor = sensors[j];
            const gpio_t* sense_pins[6] = {
                &sense_sensor->gpio_in,
                &sense_sensor->gpio_out,
                &sense_sensor->sda,
                &sense_sensor->scl,
                &sense_sensor->led_green,
                &sense_sensor->led_yellow,
            };

            success &= _assert_pins_high_for_short(output_pin, sense_pins, ARRAY_SIZE(sense_pins), sensor->name, sense_sensor->name);
        }

        gpio_set_pin_direction(output_pin->pin, GPIO_DIRECTION_IN);
    }

    return success;
}

void test_sensor_port(const sensor_t* sensor)
{
    /* AIN pin is always connected to analog functions, no need to manually enable */
    bool success = true;

    success &= test_sensor_pullups(sensor);

    // TODO: these tests should be useful - why are they commented out?
    /*if (!_test_gpio(sensor->gpio_in, sensor->sda))
    {
        success = false;
    }
    if (!_test_gpio(sensor->sda, sensor->gpio_in))
    {
        success = false;
    }
    if (!_test_gpio(sensor->gpio_out, sensor->scl))
    {
        success = false;
    }
    if (!_test_gpio(sensor->scl, sensor->gpio_out))
    {
        success = false;
    }*/

    // Test GPIOs for shorts. Pins are shorted in two groups by the test jig:
    // - IN to SDA
    // - OUT to SCL and AIN
    // We need to test that IN and OUT are not shorted.
    for (uint8_t i = 0u; i < ARRAY_SIZE(sensors); i++)
    {
        success &= _test_sensor_gpio_short(sensors, ARRAY_SIZE(sensors), i);
        _indicate(sensors[i]->result_indicator, success);
    }

    // TODO: triple check these. GPIO_OUT should not be connected to AIN, GPIO_IN should be?
    if (!_test_analog(sensor->gpio_out.pin, sensor->iovcc, sensor->adc_per, sensor->adc_ch))
    {
        SEGGER_RTT_printf(0, "%s analog test failed\n", sensor->name);
        success = false;
    }
    _indicate(sensor->result_indicator, success);
}

void test_sensor_ports(void)
{
    for (uint8_t i = 0u; i < ARRAY_SIZE(sensors); i++)
    {
        init_test_sensor_port(sensors[i]);
    }

    for (uint8_t i = 0u; i < ARRAY_SIZE(sensors); i++)
    {
        test_sensor_port(sensors[i]);
    }
}
