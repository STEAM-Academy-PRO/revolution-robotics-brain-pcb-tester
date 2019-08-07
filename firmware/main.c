#include "test.h"
#include "driver_init.h"

#include <string.h>

#define TEST_PULLUP_LED   ((uint8_t) 0u)
#define TEST_CHARGER_LED  ((uint8_t) 1u)

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

int main(void)
{
    system_init();

    test_init();

    bool pullup_result = test_pullups();
    _indicate(TEST_PULLUP_LED, pullup_result);

    bool charger_result = test_charger();
    _indicate(TEST_CHARGER_LED, charger_result);

    test_sensor_ports();
    test_motor_ports();
    test_imu();

    while (1)
    {
    }
}

void NMI_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void HardFault_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void MemManage_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void BusFault_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void UsageFault_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void SVC_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
void DebugMon_Handler( void )
{
    while (1) {
        __BKPT(1);
    }
}
