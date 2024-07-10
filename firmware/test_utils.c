#include "test_utils.h"

#include <hal_delay.h>
#include "atmel_start_pins.h"
#include <hal_adc_sync.h>
#include "SEGGER_RTT.h"

static struct adc_sync_descriptor adcs[2];

typedef enum {
    ResultNotTested,
    ResultSuccess,
    ResultFailure
} result_t;

static result_t test_results[16] = { ResultNotTested };

void reset_result(void)
{
    for (uint8_t i = 0u; i < ARRAY_SIZE(test_results); i++)
    {
        test_results[i] = ResultNotTested;
    }
}

static const char* result_to_str(result_t result)
{
    switch(result)
    {
        case ResultNotTested:
            return "Not tested";
        case ResultSuccess:
            return "Success";
        case ResultFailure:
            return "Failure";
    }

    return "Unknown";
}

bool get_result(void)
{
    for (uint8_t i = 0u; i < ARRAY_SIZE(test_results); i++)
    {
        SEGGER_RTT_printf(0, "Test %u: %s\n", i, result_to_str(test_results[i]));
        if (test_results[i] != ResultSuccess)
        {
            return false;
        }
    }

    return true;
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
    // Record/update the result
    if (success)
    {
        test_results[led] = ResultSuccess;
    }
    else
    {
        test_results[led] = ResultFailure;
    }

    // Update the LED
    switch(test_results[led])
    {
        case ResultNotTested:
            WS2812_SetLed(led, COLOR_BLUE);
            break;

        case ResultSuccess:
            WS2812_SetLed(led, COLOR_GREEN);
            break;

        case ResultFailure:
            WS2812_SetLed(led, COLOR_RED);
            break;
    }
}

/**
 * Asserts that driving one GPIO pin to a certain level results in another GPIO pin
 * measuring the same level.
 */
bool _test_gpio_level(const gpio_t* driver, const gpio_t* sense, bool level)
{
    gpio_set_pin_level(driver->pin, level);
    delay_ms(10u);

    if (gpio_get_pin_level(sense->pin) == level)
    {
        return true;
    }

    const char* driver_level = level ? "high" : "low";
    const char* sense_level = level ? "high" : "low";
    SEGGER_RTT_printf(0, "%s is driven %s but %s measured %s\n", driver->name, driver_level, sense->name, sense_level);
    return false;
}

bool _test_gpio(const gpio_t* driver, const gpio_t* sense)
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
bool _test_pullup(const gpio_t* gpio)
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

bool _test_pullups(const char* category, const gpio_t** pins, size_t pin_count, const char* message)
{
    bool success = true;

    for (size_t i = 0u; i < pin_count; i++)
    {
        if (!_test_pullup(pins[i]))
        {
            SEGGER_RTT_printf(0, "%s %s pullup test failed%s\n", category, pins[i]->name, message);
            success = false;
        }
    }

    return success;
}

bool _assert_pins_high_for_short(const gpio_t* output_pin, const gpio_t* sense_pins[], uint8_t num_pins, const char* port_name, const char* sense_port_name)
{
    bool success = true;

    for (uint8_t k = 0u; k < num_pins; k++)
    {
        const gpio_t* sense_pin = sense_pins[k];
        gpio_set_pin_direction(sense_pin->pin, GPIO_DIRECTION_IN);
        if (gpio_get_pin_level(sense_pin->pin) == 0u)
        {
            SEGGER_RTT_printf(0, "%s %s and %s %s are shorted\n", port_name, output_pin->name, sense_port_name, sense_pin->name);
            success = false;
        }
        gpio_set_pin_direction(sense_pin->pin, GPIO_DIRECTION_OFF);
    }

    return success;
}
