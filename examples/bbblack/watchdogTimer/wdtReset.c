/**
 * \file  wdtReset.c
 *
 * \brief  Sample application for WDT. The application will perform a system
 *         reset after a specified time.
 *
 *         Application Configuration:
 *
 *             Modules Used:
 *                 WDT1
 *                 UART0
 *
 *             Configurable parameters:
 *                 None.
 *
 *             Hard-coded configuration of other parameters:
 *                 1) Prescaler clock - Clock divider 1
 *                 2) Mode of WDT - Normal and Trigger mode
 *
 *         Application Use Case:
 *             The application demonstrates WDT in Normal and Trigger mode
 *             of operation
 *
 *         Running the example:
 *             On execution, the example will count for 4 seconds and if a key
 *             is not pressed within this time then WDT will perform a system
 *             reset.
 *
**/

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "consoleUtils.h"
#include "watchdog.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "beaglebone.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define INITIAL_COUNT_VALUE          (0xFFFE0000u)
#define RELOAD_COUNT_VALUE           (0xFFFE0000u)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void WatchdogTimerSetUp(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/


/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
int main(void)
{
    unsigned int triggerValue = 0;
    unsigned char inputVal = 0u;

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable the WDT clocks */
    WatchdogTimer1ModuleClkConfig();

    /* Reset the Watchdog Timer */
    WatchdogTimerReset(SOC_WDT_1_REGS);

    /* Disable the Watchdog timer */
    WatchdogTimerDisable(SOC_WDT_1_REGS);
                                               
    /* Perform the initial settings for the Watchdog Timer */
    WatchdogTimerSetUp();

    /* Send the message to UART console */
    ConsoleUtilsPrintf("Program Reset!");
    ConsoleUtilsPrintf("Input any key at least once in every 4 seconds");
    ConsoleUtilsPrintf(" to avoid a further reset.\n\r");

    /* Enable the Watchdog Timer */
    WatchdogTimerEnable(SOC_WDT_1_REGS);

    while(1)
    {
        /* Wait for an input through console. If no input is given,
        ** the WDT will timeout and reset will occur.
        */
        if(ConsoleUtilsScanf("%c", &inputVal))
        {

            triggerValue += 1;

            /* Write into the trigger register. This will load the value from the 
            ** load register into the counter register and hence timer will start 
            ** from the initial count.
            */
            WatchdogTimerTriggerSet(SOC_WDT_1_REGS, triggerValue);
        }
    }
}

/*
** This function will perform the necessary initialization for 
** the Watchdog Timer.
*/
static void WatchdogTimerSetUp(void)
{
    /* Configure and enable the pre-scaler clock */
    WatchdogTimerPreScalerClkEnable(SOC_WDT_1_REGS, WDT_PRESCALER_CLK_DIV_1);

    /* Set the count value into the counter register */
    WatchdogTimerCounterSet(SOC_WDT_1_REGS, INITIAL_COUNT_VALUE);

    /* Set the reload value into the load register */
    WatchdogTimerReloadSet(SOC_WDT_1_REGS, RELOAD_COUNT_VALUE);
}

