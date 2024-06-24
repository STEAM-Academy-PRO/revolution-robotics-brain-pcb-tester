/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include "SEGGER_RTT.h"

//*********************************************************************************************
void system_init(void)
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    init_mcu();

    hri_mclk_set_APBAMASK_SUPC_bit(MCLK);
    hri_supc_write_VREF_SEL_bf(SUPC, SUPC_VREF_SEL_2V5_Val);

    /* ondemand mode so the proper temp channel will always be selected */
    hri_supc_set_VREF_ONDEMAND_bit(SUPC);
    hri_supc_set_VREF_TSEN_bit(SUPC);

    delay_init(SysTick);
}
