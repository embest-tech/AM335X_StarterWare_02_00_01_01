/**
 * \file    demoTimer.c
 *
 * \brief   This file contains Timer related functions.
 *
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

#include "demoTimer.h"
#include "demoCfg.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "interrupt.h"
#include "dmtimer.h"
#include "clock.h"


/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TIMER_INITIAL_COUNT             (0xFFF00000u)
#define TIMER_RLD_COUNT                 (0xFFF00000u)
#define TMR_STEP_CNT                    (7u)

#define TIMER4_INITIAL_COUNT            (0xFE000000u)
#define TIMER6_INITIAL_COUNT            (0xEA8AD5FF)

/*	1 count at 32.768 KHz takes 31.25us	*/
/*	0x1900 counts takes 200ms */
#define TIMER_200MS_DELAY               (0xFFFFE6FF)

/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void Timer2Isr(void);
static void Timer4Isr(void);
static void Timer6Isr(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

unsigned int tmrStepVary = FALSE;
unsigned int tmr2Flag = FALSE;
volatile  unsigned int tmr4Flag = FALSE;

unsigned int timerCount[10] = 
						{
							0xFFF00000u,
							0xFFE00000u,
							0xFFC00000u,
							0xFFA00000u,
							0xFF800000u,
							0xFF600000u,
							0xFF400000u,
							0xFF200000u,
							0xFF000000u,
							0xFD000000u,
						};

/*******************************************************************************
**                     FUNCTION DEFINITIONS TIMER2
*******************************************************************************/
/*
** Registers the Timer2 ISR.
*/
void Timer2IntRegister(void)
{
    IntRegister(SYS_INT_TINT2, Timer2Isr);
	  
    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT2);
}

/*
** Configures the Timer2 for 32 bit
*/
void Timer2Config(void)
{
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    //DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_RLD_COUNT);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_ONESHOT_NOCMP_ENABLE);
	
	Timer2Stop();
}

/*
** Enables the Timer2 Interrupts
*/
void Timer2IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

}

/*
** Starts the Timer
*/
void Timer2Start(void)
{
    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_2_REGS);
}

/*
** Stops the Timer. The Timer Counter is Reset.
*/
void Timer2Stop(void)
{
    DMTimerDisable(SOC_DMTIMER_2_REGS);
    //DMTimerCounterSet(SOC_DMTIMER_2_REGS, 0);
}

/*
** Timer 2 Interrupt Service Routine
*/
static void Timer2Isr(void)
{
	static unsigned int index = 0;
	
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
    
    tmr2Flag = TRUE;
	
	DMTimerCounterSet(SOC_DMTIMER_2_REGS, timerCount[index++%10]);
	
	DMTimerEnable(SOC_DMTIMER_2_REGS);	
	
}


/*******************************************************************************
**                     FUNCTION DEFINITIONS TIMER4
*******************************************************************************/
/*
** Registers the Timer4 ISR.
*/
void Timer4IntRegister(void)
{
    IntRegister(SYS_INT_TINT4, Timer4Isr);
	  
    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT4, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT4);
}

/*
** Configures the Timer4 for 32 bit
*/
void Timer4Config(void)
{
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_4_REGS, TIMER4_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    DMTimerReloadSet(SOC_DMTIMER_4_REGS, TIMER_200MS_DELAY);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_4_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
	
	Timer4Stop();
}

/*
** Enables the Timer4 Interrupts
*/
void Timer4IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);

}

/*
** Starts the Timer
*/
void Timer4Start(void)
{
    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_4_REGS);
}

/*
** Stops the Timer. The Timer Counter is Reset.
*/
void Timer4Stop(void)
{
    DMTimerDisable(SOC_DMTIMER_4_REGS);
}

/*
** Timer 4 Interrupt Service Routine
*/
static void Timer4Isr(void)
{
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
    
    tmr4Flag = TRUE;
	
}

/*******************************************************************************
**                     FUNCTION DEFINITIONS TIMER6
*******************************************************************************/
/*
** Registers the Timer6 ISR.
*/
void Timer6IntRegister(void)
{
    /* Register timer 6 ISR */
    IntRegister(SYS_INT_TINT6, Timer6Isr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT6, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT6);
}

/*
** Configures the Timer6 for 32 bit
*/
void Timer6Config(void)
{
    /* Register DMTimer6 interrupts */
    Timer6IntRegister();

    /* Enable DMTimer6 module clocks */
    DMTimer6ModuleClkConfig();

    /* Set counter value to overflow in 15 seconds */
    DMTimerCounterSet(SOC_DMTIMER_6_REGS, TIMER6_INITIAL_COUNT);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_6_REGS, DMTIMER_ONESHOT_NOCMP_ENABLE);

    /* Stop the timer */
    Timer6Stop();

    /* Enable DMTimer6 interrupts */
    Timer6IntEnable();
}

/*
** Enables the Timer6 Interrupts
*/
void Timer6IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_6_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

/*
** Starts the Timer
*/
void Timer6Start(void)
{
    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_6_REGS);
}

/*
** Stops the Timer. The Timer Counter is Reset.
*/
void Timer6Stop(void)
{
    /* Stop the timer */
    DMTimerDisable(SOC_DMTIMER_6_REGS);
}

/*
** Timer 6 Interrupt Service Routine
*/
static void Timer6Isr(void)
{
    /* Stop the timer */
    DMTimerDisable(SOC_DMTIMER_6_REGS);

    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_6_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

void initializeTimer1(void)
{
	enableModuleClock(CLK_TIMER1);
	
	/*	wake up configs	*/
	HWREG(0x44e31010) = 0x214;
	
	/*	enable overflow int	*/
	HWREG(0x44e3101c) = 0x2;
	
	/*	enable overflow wakeup	*/
	HWREG(0x44e31020) = 0x2;
	
}

void setTimerCount(unsigned int count)
{
	/*	Set timer counter	*/
	HWREG(0x44e31028) = count;
	
	/*	Start timer	*/
	HWREG(0x44e31024) =  0x23;	
}


void clearTimerInt(void)
{
	HWREG(0x44e31018) = 0x2;
}

/******************************** End of file **********************************/



