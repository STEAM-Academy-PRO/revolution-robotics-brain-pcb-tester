#include "test.h"
#include "driver_init.h"

#include <string.h>

int main(void)
{
    system_init();

    test_init();

    test_pullups();
    test_charger();

    test_enable_connections();

    test_sensor_ports();
    test_motor_ports();

    test_supply_adc();

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
