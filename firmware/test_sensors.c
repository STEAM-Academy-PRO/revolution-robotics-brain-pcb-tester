#include "test.h"
#include "test_utils.h"

typedef struct {
    const char* name;
    uint32_t iovcc;
    uint32_t ain;
    uint32_t adc_per;
    uint32_t adc_ch;
    uint32_t gpio_out;
    uint32_t gpio_in;
    uint32_t sda;
    uint32_t scl;
    uint32_t led_green;
    uint32_t led_yellow;
    uint8_t result_indicator;
} sensor_t;

#define DECLARE_SENSOR_PORT(n) \
static sensor_t s##n = { \
    .name = "Sensor " #n, \
    .iovcc = S ## n ## _IOVCC, \
    .ain = S ## n ## _AIN, \
    .adc_per = S ## n ## _ADC_PER, \
    .adc_ch = S ## n ## _ADC_CH, \
    .gpio_out = S ## n ## _GPIO_OUT, \
    .gpio_in = S ## n ## _GPIO_IN, \
    .sda = I2C ## n ## _SDApin, \
    .scl = I2C ## n ## _SCLpin, \
    .led_green = S ## n ## _LED_GREEN, \
    .led_yellow = S ## n ## _LED_YELLOW, \
    .result_indicator = TEST_S ## n ## _LED, \
}

DECLARE_SENSOR_PORT(0);
DECLARE_SENSOR_PORT(1);
DECLARE_SENSOR_PORT(2);
DECLARE_SENSOR_PORT(3);

static sensor_t* sensors[] = {
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

void init_test_sensor_port(sensor_t* sensor)
{
    gpio_set_pin_direction(sensor->led_green, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(sensor->led_green, false);
}

void test_sensor_port(sensor_t* sensor)
{
    /* AIN pin is always connected to analog functions, no need to manually enable */
    bool success = true;
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

    // TODO: triple check these. GPIO_OUT should not be connected to AIN, GPIO_IN should be?
    if (!_test_analog(sensor->gpio_out, sensor->iovcc, sensor->adc_per, sensor->adc_ch))
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
