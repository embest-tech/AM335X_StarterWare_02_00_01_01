/*
 * \file   irqPreemption.c
 *
 * \brief  This is a sample application file which demonstrates
 *         IRQ preemption. This demonstrates three levels of
 *         interrupt preemption.
 */

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

#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "hw_types.h"
#include "rtc.h"
#include "dmtimer.h"
#include "uartStdio.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define BAUD_RATE_115200                (115200)
#define UART_MODULE_INPUT_CLK           (48000000)

#define TIMER_INITIAL_COUNT             (0xFF000000u)
#define TIMER_RLD_COUNT                 (0xFF000000u)
#define TIMER_FINAL_COUNT               (0x0FFFFu)

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void UARTIsr(void);
static void DMTimerIsr(void);
static void RTCIsr(void);
static void TimerSetupAndEnable(void);
static void RTCSetupAndEnable(void);

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
static volatile unsigned int preemptFlag = 0;

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

int main()
{
    unsigned int divisorValue = 0;

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Performing the Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Performing a module reset. */
    UARTModuleReset(SOC_UART_0_REGS);

    UARTFIFOConfig(SOC_UART_0_REGS,
                   UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                    UART_TRIG_LVL_GRANULARITY_1,
                                    1, 1, 1, 1, UART_DMA_EN_PATH_SCR,
                                     UART_DMA_MODE_0_ENABLE));


   /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         BAUD_RATE_115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(SOC_UART_0_REGS, divisorValue);

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(SOC_UART_0_REGS, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(SOC_UART_0_REGS);

    /* Disabling Break Control. */
    UARTBreakCtl(SOC_UART_0_REGS, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(SOC_UART_0_REGS, UART16x_OPER_MODE);
    UARTIntEnable(SOC_UART_0_REGS, (UART_INT_LINE_STAT | UART_INT_THR |
                                    UART_INT_RHR_CTI));


    /* Registering the Interrupt Service Routines */
    IntRegister(SYS_INT_RTCINT, RTCIsr);
    IntRegister(SYS_INT_UART0INT, UARTIsr);
    IntRegister(SYS_INT_TINT2, DMTimerIsr);

    /*
    ** Setting the priority for the system interrupt in AINTC.
    ** Timer interrupt is given highest priority - 1
    ** RTC interrupt is given medium priority - 2
    ** UART interrupt is given lowest priority - 4
    */
    IntPrioritySet(SYS_INT_TINT2, 1, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_RTCINT, 2, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_UART0INT, 4, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC for UART */
    IntSystemEnable(SYS_INT_UART0INT);

    IntMasterIRQEnable();

    while(1);
}

/*
** RTC configuration function
*/
static void RTCSetupAndEnable(void)
{
    /* Performing the System Clock configuration for RTC. */
    RTCModuleClkConfig();

    /* Disabling Write Protection for RTC registers.*/
    RTCWriteProtectDisable(SOC_RTC_0_REGS);

    /* Selecting Internal Clock source for RTC. */
    RTC32KClkSourceSelect(SOC_RTC_0_REGS, RTC_INTERNAL_CLK_SRC_SELECT);

    /* Enabling RTC to receive the Clock inputs. */
    RTC32KClkClockControl(SOC_RTC_0_REGS, RTC_32KCLK_ENABLE);

    /* Enable the RTC module. */
    RTCEnable(SOC_RTC_0_REGS);

    /* Dummy: Programming calendar information in the Calendar registers. */
    RTCCalendarSet(SOC_RTC_0_REGS, 0x29111000 | RTC_DOTW_MON);

    /* Dummy: Programming the time information in the Time registers. */
    RTCTimeSet(SOC_RTC_0_REGS, (0x08473100 | RTC_ANTE_MERIDIEM));

    /* Set the 32KHz counter to run. */
    RTCRun(SOC_RTC_0_REGS);

    /* Enabling RTC interrupts. Configuring RTC to interrupt every second.*/
    RTCIntTimerEnable(SOC_RTC_0_REGS, RTC_INT_EVERY_SECOND);

    /* Enabling the system interrupt in AINTC for RTC. */
    IntSystemEnable(SYS_INT_RTCINT);
}

/*
** Configure and start the Timer
*/
static void TimerSetupAndEnable(void)
{
    /* This function will enable clocks for the DMTimer2 instance */
    DMTimer2ModuleClkConfig();

    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_RLD_COUNT);

    /* Configure the DMTimer for Auto-reload and compare mode */
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_2_REGS);

    IntSystemEnable(SYS_INT_TINT2);
}

/*
** DMTimer interrupt service routine.
*/
static void DMTimerIsr(void)
{
    UARTPuts("                Timer ISR Entry.\r\n", -1);

    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

    /* Stop the DMTimer */
    DMTimerDisable(SOC_DMTIMER_2_REGS);

    preemptFlag = 4;

    UARTPuts("                Timer ISR Exit.\r\n", -1);
}

/*
** This is the Interrupt Service Routine(ISR) for RTC.
*/
static void RTCIsr(void)
{
    RTCIntTimerDisable(SOC_RTC_0_REGS);

    UARTPuts("        RTC ISR Entry.\r\n", -1);

    TimerSetupAndEnable();

    preemptFlag = 1;

    /*Wait till any higher priority ISR changes the flag */
    while(1 == preemptFlag);

    UARTPuts("        RTC ISR Exit.\r\n", -1);
}

/*
** Interrupt Service Routine for UART.
*/
static void UARTIsr(void)
{
    /* Reconfiguring the UART STDIO instance. */
    UARTStdioInit();
    UARTPuts("StarterWare Interrupt Preemption Demonstration.\r\n", -2);

    UARTPuts("UART ISR Entry.\r\n", -1);

    RTCSetupAndEnable();

    /* Wait till any higher priority ISR changes preemptFlag */
    while(0 == preemptFlag);

    UARTPuts("UART ISR Exit.\r\n", -1);
}

/******************************* End of file *********************************/
