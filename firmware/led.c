/*
 * led.c
 *
 * Simple WS2812 driver, generates waveforms using SPI
 *
 * Created: 2019. 08. 06. 11:28:07
 *  Author: Dániel Buga
 */ 

#include "test.h"

#include <string.h>

#include <hal_spi_m_sync.h>

#define LED_VAL_ZERO    ((uint8_t) ~0xC0u)
#define LED_VAL_ONE     ((uint8_t) ~0xFCu)
#define LED_VAL_RES     ((uint8_t) ~0x00u)
#define LED_RESET_SIZE  ((size_t)     50u)

#define LED_BYTE_SIZE   8                   /* one LED control bit is coded as 8 MCU bits, so 1 byte -> 8 bytes */
#define LED_COLOR_SIZE  3 * LED_BYTE_SIZE   /* 24 LED bits in total */
#define LED_FRAME_SIZE  (LED_RESET_SIZE + (LED_COLOR_SIZE * 16))

static uint8_t frame_leds[LED_FRAME_SIZE];

static struct spi_m_sync_descriptor SPI_0;

static void _spi_Init(void)
{
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_mclk_set_APBDMASK_SERCOM4_bit(MCLK);

    spi_m_sync_init(&SPI_0, WS2812spi);
    spi_m_sync_enable(&SPI_0);

    gpio_set_pin_level(WS2812pin, false);
    gpio_set_pin_direction(WS2812pin, GPIO_DIRECTION_OUT);
    gpio_set_pin_pull_mode(WS2812pin, GPIO_PULL_OFF);
    gpio_set_pin_function(WS2812pin, WS2812pin_function);
}

static inline uint8_t getLedBitPattern(uint8_t bitValue)
{
    return (bitValue == 0u) ? LED_VAL_ZERO : LED_VAL_ONE;
}

static inline void write_led_byte(uint32_t led_idx, uint32_t byte_idx, uint8_t byte_value)
{
    const uint32_t startIdx = LED_RESET_SIZE + 8u * (3u * led_idx + byte_idx);
    for (uint32_t i = 0u; i < 8u; i++)
    {
        frame_leds[startIdx + i] = getLedBitPattern(byte_value & (0x80u >> i));
    }
}

static void _set_led(uint8_t led, uint8_t color)
{
    /* brightness scaling */
    uint8_t g, r, b;

    switch (color)
    {
        default:
        case COLOR_OFF:
            r = 0u;
            g = 0u;
            b = 0u;
            break;

        case COLOR_RED:
            r = 12u;
            g = 0u;
            b = 0u;
            break;

        case COLOR_GREEN:
            r = 0u;
            g = 12u;
            b = 0u;
            break;

        case COLOR_BLUE:
            r = 0u;
            g = 4u;
            b = 8u;
            break;
    }

    write_led_byte(led, 0u, g);
    write_led_byte(led, 1u, r);
    write_led_byte(led, 2u, b);
}

static void _send(void)
{
    const struct spi_xfer xfer = {
        .txbuf = frame_leds,
        .rxbuf = NULL,
        .size = sizeof(frame_leds)
    };
    
    gpio_set_pin_function(WS2812pin, WS2812pin_function);
    spi_m_sync_transfer(&SPI_0, &xfer);
    gpio_set_pin_level(WS2812pin, true);
    gpio_set_pin_function(WS2812pin, GPIO_PIN_FUNCTION_OFF);
}

void WS2812_Init(void)
{
    _spi_Init();

    uint8_t colors[16] = { 0 };

    for (uint8_t i = 0u; i < 16u; i++)
    {
        colors[i] = COLOR_BLUE;
    }

    memset(frame_leds, LED_VAL_RES, sizeof(frame_leds));

    WS2812_Write(colors);
}

void WS2812_Write(uint8_t colors[16])
{
    for (uint8_t i = 0u; i < 16; i++)
    {
        _set_led(i, colors[i]);
    }
    _send();
}

void WS2812_SetLed(uint8_t led, uint8_t color)
{
    if (led < 16u)
    {
        _set_led(led, color);
        _send();
    }
}
