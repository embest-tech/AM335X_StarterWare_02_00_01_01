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
#include "evmskAM335x.h"
#include "hw_types.h"
#include "rtc.h"
#include "dmtimer.h"
#include "uartStdio.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define BAUD_RATE_115200                (115200u)
#define UART_MODULE_INPUT_CLK           (48000000u)

#define TIMER_INITIAL_COUNT             (0xFF000000u)
#define TIMER_RLD_COUNT                 (0xFF000000u)
#define TIMER_FINAL_COUNT               (0x0FFFFu)

#define RTC_CAL_VAL_DUMMY               (0x29111000u | RTC_DOTW_MON)
#define RTC_TIME_VAL_DUMMY              (0x08473100u | RTC_ANTE_MERIDIEM)

#define IRQ_PRIORITY_TIMER              (1u)
#define IRQ_PRIORITY_RTC                (2u)
#define IRQ_PRIORITY_UART               (4u)

#define PREEMPT_FLAG_DEFAULT            (0u)
#define PREEMPT_FLAG_TIMER_ISR          (4u)
#define PREEMPT_FLAG_RTC_ISR            (1u)

#define UART_INT_NUM                    SYS_INT_UART0INT
#define RTC_INT_NUM                     SYS_INT_RTCINT 
#define TIMER_INT_NUM                   SYS_INT_TINT2 
                   
#define UART_INST_BASE                  SOC_UART_0_REGS
#define RTC_INST_BASE                   SOC_RTC_0_REGS
#define TIMER_INST_BASE                 SOC_DMTIMER_2_REGS

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
static volatile unsigned int preemptFlag = PREEMPT_FLAG_DEFAULT;

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/
/*
** RTC configuration function
*/
static void RTCSetupAndEnable(void)
{
    /* Performing the System Clock configuration for RTC. */
    RTCModuleClkConfig();

    /* Disabling Write Protection for RTC registers.*/
    RTCWriteProtectDisable(RTC_INST_BASE);

    /* Selecting Internal Clock source for RTC. */
    RTC32KClkSourceSelect(RTC_INST_BASE, RTC_INTERNAL_CLK_SRC_SELECT);

    /* Enabling RTC to receive the Clock inputs. */
    RTC32KClkClockControl(RTC_INST_BASE, RTC_32KCLK_ENABLE);

    /* Enable the RTC module. */
    RTCEnable(RTC_INST_BASE);

    /* Dummy: Programming calendar information in the Calendar registers. */
    RTCCalendarSet(RTC_INST_BASE, RTC_CAL_VAL_DUMMY);

    /* Dummy: Programming the time information in the Time registers. */
    RTCTimeSet(RTC_INST_BASE, RTC_TIME_VAL_DUMMY);

    /* Set the 32KHz counter to run. */
    RTCRun(RTC_INST_BASE);

    /* Enabling RTC interrupts. Configuring RTC to interrupt every second.*/
    RTCIntTimerEnable(RTC_INST_BASE, RTC_INT_EVERY_SECOND);

    /* Enabling the system interrupt in AINTC for RTC. */
    IntSystemEnable(RTC_INT_NUM);
}

/*
** Configure and start the Timer
*/
static void TimerSetupAndEnable(void)
{
    /* This function will enable clocks for the DMTimer2 instance */
    DMTimer2ModuleClkConfig();

    /* Load the counter with the initial count value */
    DMTimerCounterSet(TIMER_INST_BASE, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    DMTimerReloadSet(TIMER_INST_BASE, TIMER_RLD_COUNT);

    /* Configure the DMTimer for Auto-reload and compare mode */
    DMTimerModeConfigure(TIMER_INST_BASE, DMTIMER_AUTORLD_NOCMP_ENABLE);

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(TIMER_INST_BASE, DMTIMER_INT_OVF_EN_FLAG);

    /* Start the DMTimer */
    DMTimerEnable(TIMER_INST_BASE);

    IntSystemEnable(TIMER_INT_NUM);
}

/*
** DMTimer interrupt service routine.
*/
static void DMTimerIsr(void)
{
    UARTPuts("                Timer ISR Entry.\r\n", -1);

    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(TIMER_INST_BASE, DMTIMER_INT_OVF_EN_FLAG);

    /* Stop the DMTimer */
    DMTimerDisable(TIMER_INST_BASE);

    preemptFlag = PREEMPT_FLAG_TIMER_ISR;

    UARTPuts("                Timer ISR Exit.\r\n", -1);
}

/*
** This is the Interrupt Service Routine(ISR) for RTC.
*/
static void RTCIsr(void)
{
    RTCIntTimerDisable(RTC_INST_BASE);

    UARTPuts("        RTC ISR Entry.\r\n", -1);

    TimerSetupAndEnable();

    preemptFlag = PREEMPT_FLAG_RTC_ISR;

    /*Wait till any higher priority ISR changes the flag */
    while(PREEMPT_FLAG_RTC_ISR == preemptFlag);

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
    while(PREEMPT_FLAG_DEFAULT == preemptFlag);

    UARTPuts("UART ISR Exit.\r\n", -1);
}

/*
** The main function
*/
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
    UARTModuleReset(UART_INST_BASE);

    UARTFIFOConfig(UART_INST_BASE,
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
    UARTDivisorLatchWrite(UART_INST_BASE, divisorValue);

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(UART_INST_BASE, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(UART_INST_BASE,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabe write access to Divisor Latches. */
    UARTDivisorLatchDisable(UART_INST_BASE);

    /* Disable Break Control. */
    UARTBreakCtl(UART_INST_BASE, UART_BREAK_COND_DISABLE);

    /* Switch to UART16x operating mode. */
    UARTOperatingModeSelect(UART_INST_BASE, UART16x_OPER_MODE);
    UARTIntEnable(UART_INST_BASE, (UART_INT_LINE_STAT | UART_INT_THR |
                                    UART_INT_RHR_CTI));


    /* Register the Interrupt Service Routines */
    IntRegister(RTC_INT_NUM, RTCIsr);
    IntRegister(UART_INT_NUM, UARTIsr);
    IntRegister(TIMER_INT_NUM, DMTimerIsr);

    /*
    ** Setting the priority for the system interrupt in AINTC.
    ** Timer interrupt is given highest priority - 1
    ** RTC interrupt is given medium priority - 2
    ** UART interrupt is given lowest priority - 4
    */
    IntPrioritySet(TIMER_INT_NUM, IRQ_PRIORITY_TIMER, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(RTC_INT_NUM, IRQ_PRIORITY_RTC, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(UART_INT_NUM, IRQ_PRIORITY_UART, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC for UART */
    IntSystemEnable(UART_INT_NUM);

    IntMasterIRQEnable();

    while(1);
}


