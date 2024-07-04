#include "test.h"
#include "test_utils.h"
#include <tcc_lite.h>

void test_init(void)
{
    reset_result();
    WS2812_Init();

    gpio_set_pin_direction(TEST_ENABLE, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(TEST_CHARGER_EN, GPIO_DIRECTION_OUT);

    gpio_set_pin_level(TEST_ENABLE, false);
    gpio_set_pin_level(TEST_CHARGER_EN, false);

    gpio_set_pin_direction(S0_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S1_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S2_IOVCC, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(S3_IOVCC, GPIO_DIRECTION_OUT);
    // Set IOVCC to 3V3
    gpio_set_pin_level(S0_IOVCC, false);
    gpio_set_pin_level(S1_IOVCC, false);
    gpio_set_pin_level(S2_IOVCC, false);
    gpio_set_pin_level(S3_IOVCC, false);

    hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
    hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    adc_init();

    imu_init();
}

/**
 * Light up all RGB LEDs with the primary colors. Needs operator confirmation.
 */
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

/**
 * Test that pullup resistors (and associated components, MCU pins) are soldered correctly.
 */
void test_pullups(void)
{
    #define TEST_PULLUP(gpio, check) _test_pullup(&(gpio_t){ .pin = gpio, .name = #gpio })

    bool success = true;

    // 47k pulling up pin 1 of U16 to VDDIO. If VDDIO is low, the MCU should not come out of reset.
    success &= TEST_PULLUP(AMP_EN_sense, "R120");

    success &= test_motor_pullups();

    /* internal pullups, skip */
    // success &= TEST_PULLUP(CHARGER_STAT);
    // success &= TEST_PULLUP(CHARGER_STBY);

    _indicate(TEST_PULLUP_LED, success);
}

void test_charger(void)
{
    /* configure pins */
    gpio_set_pin_level(CHARGER_STAT, false);
    gpio_set_pin_level(CHARGER_STBY, false);

    gpio_set_pin_direction(CHARGER_STAT, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(CHARGER_STAT, GPIO_PULL_UP);

    gpio_set_pin_direction(CHARGER_STBY, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(CHARGER_STBY, GPIO_PULL_UP);

    /* execute test */
    gpio_set_pin_level(TEST_CHARGER_EN, false);

    delay_ms(200u);

    bool success = true;
    if (gpio_get_pin_level(CHARGER_STAT) != 1u)
    {
        SEGGER_RTT_printf(0, "CHARGER_STAT pin not high\n");
        success = false;
    }

    if (gpio_get_pin_level(CHARGER_STBY) != 1u)
    {
        SEGGER_RTT_printf(0, "CHARGER_STBY pin not high\n");
        success = false;
    }

    gpio_set_pin_level(TEST_CHARGER_EN, true);

    delay_ms(20u);

    bool changed = false;
    for (uint32_t i = 0u; i < 100000u; i++)
    {
        bool stat = gpio_get_pin_level(CHARGER_STAT) == 1u;
        bool stby = gpio_get_pin_level(CHARGER_STBY) == 1u;

        if (!stat || !stby)
        {
            changed = true;
            break;
        }
    }

    if (!changed)
    {
        SEGGER_RTT_printf(0, "CHARGER_STBY or CHARGER_STAT did not change after connecting power\n");
        success = false;
    }

    /* reset pins */

    // Keep the charger enabled so we charge the battery a bit.
    // gpio_set_pin_level(TEST_CHARGER_EN, false);

    gpio_set_pin_direction(CHARGER_STAT, GPIO_DIRECTION_OFF);
    gpio_set_pin_pull_mode(CHARGER_STAT, GPIO_PULL_OFF);
    gpio_set_pin_direction(CHARGER_STBY, GPIO_DIRECTION_OFF);
    gpio_set_pin_pull_mode(CHARGER_STBY, GPIO_PULL_OFF);

    _indicate(TEST_CHARGER_LED, success);
}

void test_imu(void)
{
    bool success = true;

    uint8_t whoami = imu_read_whoami();

    uint8_t accepted_whoami[] = { 0x6Au, 0x69u };

    bool whoami_ok = false;
    for (uint8_t i = 0u; i < ARRAY_SIZE(accepted_whoami); i++)
    {
        if (accepted_whoami[i] == whoami)
        {
            whoami_ok = true;
            break;
        }
    }

    if (!whoami_ok)
    {
        SEGGER_RTT_printf(0, "IMU WHOAMI not accepted: 0x%02X\n", whoami);
        success = false;
    }
    if (!imu_run_selftest())
    {
        SEGGER_RTT_WriteString(0, "IMU selftest failed\n");
        success = false;
    }

    _indicate(TEST_IMU_LED, success);
}

void test_supply_adc(void)
{
    /* TODO: these may be more strict */
    bool success;

    success = true;
    if (!_sysmon_analog_expect(1u, ADC_CH_BAT_VOLTAGE, 3000.0f, 4300.0f, 240.0f/340.0f))
    {
        SEGGER_RTT_WriteString(0, "Battery ADC test failed\n");
        success = false;
    }
    _indicate(TEST_BATTERY_ADC_LED, success);

    success = true;
    if (!_sysmon_analog_expect(1u, ADC_CH_MOT_VOLTAGE, 4000.0f, 11000.0f, 30.0f/130.0f))
    {
        SEGGER_RTT_WriteString(0, "Motor ADC test failed\n");
        success = false;
    }
    _indicate(TEST_MOTOR_ADC_LED, success);
}

void test_sound(void)
{
    hri_mclk_set_APBBMASK_TCC1_bit(MCLK);
    hri_gclk_write_PCHCTRL_reg(GCLK, TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    gpio_set_pin_direction(AMP_EN_sense, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(SOUND_TEST_PWM, GPIO_DIRECTION_OUT);
    gpio_set_pin_drive(SOUND_TEST_PWM, GPIO_DRIVE_STRONG);
    gpio_set_pin_function(SOUND_TEST_PWM, PINMUX_PD20F_TCC1_WO0);

    PWM_0_init();

    /* enable amplifier */
    gpio_set_pin_level(AMP_EN_sense, false);
    delay_ms(100u);

    /* about 1s */
    hri_tcc_set_CTRLA_CPTEN0_bit(TCC1);
    hri_tcc_write_CTRLA_ENABLE_bit(TCC1, 1 << TCC_CTRLA_ENABLE_Pos); /* Enable: enabled */

    uint16_t sin_vals[1000];
    for (uint32_t i = 0u; i < 1000u; i++)
    {
        float f = sinf(2.0f * (float)M_PI * i / 1001); /* 1kHz sine wave -1 .. 1 */
        float scaled = ((f + 1.0f) / 2.0f) * 800.0f; /* -1 -> 0; 1-> PER */

        sin_vals[i] = lroundf(scaled);
    }

    for (uint32_t i = 0u; i < 1u * 1000000u; i++)
    {
        uint16_t val = sin_vals[i % 1000];
        hri_tcc_write_CCBUF_reg(TCC1, 0u, val);
        hri_tcc_wait_for_sync(TCC1, TCC_SYNCBUSY_CC0);
    }

    hri_tcc_clear_CTRLA_ENABLE_bit(TCC1);

    /* disable amplifier */
    gpio_set_pin_level(AMP_EN_sense, true);

    _indicate(TEST_TEST_DONE_LED, true);

    gpio_set_pin_direction(AMP_EN_sense, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(SOUND_TEST_PWM, GPIO_DIRECTION_OFF);
}

void test_end(void)
{
    gpio_set_pin_level(TEST_ENABLE, false);

    gpio_set_pin_direction(TEST_ENABLE, GPIO_DIRECTION_OFF);
    gpio_set_pin_direction(TEST_CHARGER_EN, GPIO_DIRECTION_OFF);

    if (get_result())
    {
        SEGGER_RTT_WriteString(0, "Test passed\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "Test failed\n");
    }

    while (1) {
        __BKPT(1);
    }
}
