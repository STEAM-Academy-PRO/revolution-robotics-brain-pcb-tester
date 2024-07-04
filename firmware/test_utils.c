#include "test_utils.h"

#include <hal_delay.h>
#include "atmel_start_pins.h"
#include <hal_adc_sync.h>
#include "SEGGER_RTT.h"

static struct adc_sync_descriptor adcs[2];

static bool test_result;

void reset_result(void)
{
    test_result = true;
}

bool get_result(void)
{
    return test_result;
}

void adc_init(void)
{
    adc_sync_init(&adcs[0], ADC0);
    adc_sync_init(&adcs[1], ADC1);

    adc_sync_enable_channel(&adcs[0], 0u);
    adc_sync_enable_channel(&adcs[1], 0u);
}

void _indicate(uint8_t led, bool success)
{
    if (success)
    {
        WS2812_SetLed(led, COLOR_GREEN);
    }
    else
    {
        WS2812_SetLed(led, COLOR_RED);
        test_result = false;
    }
}

/**
 * Asserts that driving one GPIO pin to a certain level results in another GPIO pin
 * measuring the same level.
 */
bool _test_gpio_level(gpio_t* driver, gpio_t* sense, bool level)
{
    gpio_set_pin_level(driver->pin, level);
    delay_ms(10u);
    if (gpio_get_pin_level(sense->pin) != level)
    {
        const char* driver_level = level ? "high" : "low";
        const char* sense_level = level ? "high" : "low";
        SEGGER_RTT_printf(0, "%s is driven %s but %s measured %s\n", driver->name, driver_level, sense->name, sense_level);
        return false;
    }
    else
    {
        return true;
    }
}

// TODO: add a pin that must not change during this test
bool _test_gpio(gpio_t* driver, gpio_t* sense)
{
    bool success = true;

    gpio_set_pin_direction(driver->pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(sense->pin, GPIO_DIRECTION_IN);

    success &= _test_gpio_level(driver, sense, false);
    success &= _test_gpio_level(driver, sense, true);

    // Disable driver.
    gpio_set_pin_level(driver->pin, false);
    gpio_set_pin_direction(driver->pin, GPIO_DIRECTION_OFF);

    // Drive sense low to ensure capacitors are discharged.
    gpio_set_pin_direction(sense->pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(sense->pin, false);
    delay_ms(10u);
    gpio_set_pin_direction(sense->pin, GPIO_DIRECTION_OFF);

    return success;
}

float map(float in, float min_in, float max_in, float min_out, float max_out)
{
    float in_fs = max_in - min_in;
    float out_fs = max_out - min_out;

    return (in - min_in) * (out_fs / in_fs) + min_out;
}

float _read_analog(uint32_t adc, uint32_t ch)
{
    #define SKIPPED_EXTREMES (8u)
    uint16_t buffer[32];
    delay_ms(10u);
    adc_sync_set_inputs(&adcs[adc], ch, ADC_CHN_INT_GND, 0u);
    adc_sync_read_channel(&adcs[adc], 0u, (uint8_t*) buffer, sizeof(buffer));

    /* sort the buffer so the n smallest and greatest values can be ignored */
    for (uint32_t i = 0u; i < ARRAY_SIZE(buffer) - 1u; i++)
    {
        for (uint32_t j = i; j < ARRAY_SIZE(buffer); j++)
        {
            if (buffer[i] > buffer[j])
            {
                uint16_t temp = buffer[i];
                buffer[i] = buffer[j];
                buffer[j] = temp;
            }
        }
    }

    /* average the values that are not ignored */
    uint16_t sum = 0u;
    for (uint32_t i = SKIPPED_EXTREMES; i < ARRAY_SIZE(buffer) - SKIPPED_EXTREMES; i++)
    {
        sum += buffer[i];
    }
    sum = sum / (ARRAY_SIZE(buffer) - 2 * SKIPPED_EXTREMES);

    return map(sum, 0, 4095, 0, 3300); /* Vref = VDDANA */
}

bool _sysmon_analog_expect(uint32_t adc, uint32_t ch, float lower, float upper, float divider)
{
    float voltage = _read_analog(adc, ch) / divider;

    return lower < voltage && voltage < upper;
}

static void _pullup_test_fail(const char* gpio_name, const char* msg)
{
    SEGGER_RTT_printf(0, "Pullup test failed on pin %s: %s\n", gpio_name, msg);
}

/**
 * Assert that `gpio` has a pullup resistor.
 */
bool _test_pullup(gpio_t* gpio)
{
    /* by default they should be pulled up */
    gpio_set_pin_direction(gpio->pin, GPIO_DIRECTION_IN);
    bool initial = gpio_get_pin_level(gpio->pin) == 1u;
    if (!initial) {
        /* Try to detect if we have an external short circuit or just a broken connection. */
        gpio_set_pin_pull_mode(gpio->pin, GPIO_PULL_UP);
        delay_ms(1);
        bool internal = gpio_get_pin_level(gpio->pin) == 1u;
        if (internal) {
            _pullup_test_fail(gpio->name, "initial state not pulled up");
        } else {
            _pullup_test_fail(gpio->name, "line is shorted low");
        }

        gpio_set_pin_pull_mode(gpio->pin, GPIO_PULL_OFF);
    }

    /* pull them down and release them */
    gpio_set_pin_direction(gpio->pin, GPIO_DIRECTION_INOUT);
    gpio_set_pin_level(TEST_ENABLE, false);
    bool pulldown = gpio_get_pin_level(gpio->pin) == 0u;
    if (!pulldown) {
        _pullup_test_fail(gpio->name, "not pulled down");
    }

    delay_ms(1);

    gpio_set_pin_direction(gpio->pin, GPIO_DIRECTION_IN);
    delay_ms(1u);
    bool pullup = gpio_get_pin_level(gpio->pin) == 1u;
    if (!pullup) {
        _pullup_test_fail(gpio->name, "not pulled up");
    }

    return initial && pulldown && pullup;
}
