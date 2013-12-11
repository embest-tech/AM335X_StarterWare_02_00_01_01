/**
 * \file    demoTimer.c
 *
 * \brief   This file contains Timer related functions.
 */

/* Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 6288 of the EK-LM3S2965 Firmware Package.
 * This file is modified to make it work for StarterWare. */


#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "dmtimer.h"
#include "gameTimer.h"

/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TMR_PERIOD                          (0xFFFFFFFFu)

#define TIMER_INITIAL_COUNT             (0x0u)
#define TIMER_RLD_COUNT                 (0x0u)
#define TMR_STEP_CNT                    (7u)

/*	1 count at 32.768 KHz takes 30.52us	*/
/*	0x1900 counts takes 200ms */
#define TIMER_200MS_DELAY               (0xFFFFE6FF)

/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void Timer2Isr(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
unsigned int tmrStepVary = FALSE;
unsigned int tmrFlag = FALSE;

/*******************************************************************************
**                     FUNCTION DEFINITIONS
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
	Timer2IntRegister();
	
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    //DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_RLD_COUNT);

    /* Configure the DMTimer for one shot mode */
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_TCLR_AR_ONESHOT);
	
	Timer2Stop();
}

/*
** Enables the Timer2 Interrupts
*/
void Timer2IntEnable(void)
{
    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_IRQENABLE_SET_OVF_EN_FLAG_ENABLE);

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
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_IRQENABLE_SET_OVF_EN_FLAG_ENABLE);
    
	DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);
	
	DMTimerEnable(SOC_DMTIMER_2_REGS);	
	
}

/******************************** End of file **********************************/



