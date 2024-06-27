#include "test_utils.h"

#include <hal_delay.h>
#include "atmel_start_pins.h"
#include <hal_adc_sync.h>
#include "SEGGER_RTT.h"

static struct adc_sync_descriptor adcs[2];

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
    }
}

// TODO: add a pin that must not change during this test
bool _test_gpio(gpio_t* driver, gpio_t* sense)
{
    bool success = true;

    gpio_set_pin_direction(driver->pin, GPIO_DIRECTION_OUT);
    //gpio_set_pin_direction(sense, GPIO_DIRECTION_OUT);
    //gpio_set_pin_level(sense, false);
    //gpio_set_pin_level(driver, false);
    //delay_ms(10u);
    gpio_set_pin_direction(sense->pin, GPIO_DIRECTION_IN);

    gpio_set_pin_level(driver->pin, false);
    delay_ms(10u);
    if (gpio_get_pin_level(sense->pin) != 0u)
    {
        SEGGER_RTT_printf(0, "%s is driven low but %s measured high\n", driver->name, sense->name);
        success = false;
    }

    gpio_set_pin_level(driver->pin, true);
    delay_ms(10u);
    if (gpio_get_pin_level(sense->pin) != 1u)
    {
        SEGGER_RTT_printf(0, "%s is driven high but %s measured low\n", driver->name, sense->name);
        success = false;
    }

    gpio_set_pin_direction(sense->pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(driver->pin, false);
    gpio_set_pin_level(sense->pin, false);
    delay_ms(10u);
    gpio_set_pin_direction(driver->pin, GPIO_DIRECTION_OFF);
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

bool _analog_expect(uint32_t adc, uint32_t ch, float lower, float upper)
{
    float voltage = _read_analog(adc, ch) * (250.0f / 150.0f); /* compensate for voltage divider */

    return lower < voltage && voltage < upper;
}

bool _sysmon_analog_expect(uint32_t adc, uint32_t ch, float lower, float upper, float divider)
{
    float voltage = _read_analog(adc, ch) / divider;

    return lower < voltage && voltage < upper;
}
