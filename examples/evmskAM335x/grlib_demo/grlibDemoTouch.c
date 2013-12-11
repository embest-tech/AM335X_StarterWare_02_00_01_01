/**
 * \file    grlibDemoTouch.c
 *
 * \brief   This file contains Touch Screen related functions.
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

#include "interrupt.h"
#include "grlibDemoTouch.h"
#include "soc_AM335x.h"
#include "hw_types.h"
#include "evmskAM335x.h"
#include "tsc_adc.h"
#include "dmtimer.h"
#include "grlib.h"
#include "grlib_demo.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define SAMPLES                                 5u

/*******************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void TouchScreenIsr(void);
static void Timer3Isr(void);
static void StepConfigX(unsigned int);
static void StepConfigY(unsigned int);

/*******************************************************************************
**                     INTERNAL VARIALE DEFINITIONS
*******************************************************************************/
volatile unsigned int x_data[2];
volatile unsigned int y_data[2];
volatile unsigned int IsTSPress = 0;
unsigned int touchRelease = 0;
unsigned int dbidx = 0;

unsigned int xdata = 0;
unsigned int ydata = 0;

/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Initializes the touch screen 
*/
void InitTouchScreen(void)
{
    unsigned int i;  

    /* TSC clock config */
    TSCADCModuleClkConfig();

    /* TSC pin mux */
    TSCADCPinMuxSetUp();

    /* Timer3 Clock config */
    DMTimer3ModuleClkConfig();

    /* configures ADC to 3Mhz */
    TSCADCConfigureAFEClock(SOC_ADC_TSC_0_REGS, 24000000, 3000000);

    /* Enable Transistor bias */
    TSCADCTSTransistorConfig(SOC_ADC_TSC_0_REGS, TSCADC_TRANSISTOR_ENABLE);

    /* Map hardware event to Pen Touch IRQ */
    TSCADCHWEventMapSet(SOC_ADC_TSC_0_REGS, TSCADC_PEN_TOUCH);

    /* Set 4 Wire or 5 wire touch screen  mode */
    TSCADCTSModeConfig(SOC_ADC_TSC_0_REGS, TSCADC_FOUR_WIRE_MODE);

    TSCADCStepIDTagConfig(SOC_ADC_TSC_0_REGS, 1);

    /* Disable Write Protection of Step Configuration regs*/
    TSCADCStepConfigProtectionDisable(SOC_ADC_TSC_0_REGS);

    /* Configure ADC to Single ended operation mode */
    TSCADCIdleStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                       TSCADC_SINGLE_ENDED_OPER_MODE);

    /* Configure reference volatage and input to idlestep */
    TSCADCIdleStepConfig(SOC_ADC_TSC_0_REGS, TSCADC_NEGATIVE_REF_VSSA,
                         TSCADC_POSITIVE_INP_CHANNEL1, TSCADC_NEGATIVE_INP_ADCREFM,
                         TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCIdleStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_OFF,
                                     TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_OFF);

    /*
    **Configure the Analong Ground of Touch screen.
    */
    TSCADCIdleStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_OFF,
                                     TSCADC_YPNSW_PIN_ON, TSCADC_YNNSW_PIN_ON,
                                     TSCADC_WPNSW_PIN_OFF);


    /* Configure ADC to Single ended operation mode */
    TSCADCChargeStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                         TSCADC_SINGLE_ENDED_OPER_MODE);

    /* Configure reference volatage and input to charge step*/
    TSCADCChargeStepConfig(SOC_ADC_TSC_0_REGS, TSCADC_NEGATIVE_REF_XNUR,
                           TSCADC_POSITIVE_INP_CHANNEL2, TSCADC_NEGATIVE_INP_CHANNEL2,
                           TSCADC_POSITIVE_REF_XPUL);

    /* Configure the Analog Supply to Touch screen */
    TSCADCChargeStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_ON,
                                       TSCADC_XNPSW_PIN_OFF, TSCADC_XPPSW_PIN_OFF);

    /* Configure the Analong Ground to Touch screen */
    TSCADCChargeStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_OFF,
                                       TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_ON,
                                       TSCADC_WPNSW_PIN_OFF);

    TSCADCTSChargeStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, 0x200);

    for(i = 0; i < SAMPLES; i++)
    {
         StepConfigX(i);

         TSCADCTSStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, i, 0x98);
    }

    for(i = SAMPLES; i < (2 * SAMPLES); i++)
    {
         StepConfigY(i);

         TSCADCTSStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, i, 0x98);
    }

    /* Configure FIFO1 threshold value */
    TSCADCFIFOIRQThresholdLevelConfig(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1, 5);

    IsTSPress = 0;
    touchRelease = 0;

    /* timer setup for touch screen */
    IntPrioritySet(SYS_INT_TINT3, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT3);
    IntRegister(SYS_INT_TINT3, Timer3Isr);

    DMTimerModeConfigure(SOC_DMTIMER_3_REGS, DMTIMER_ONESHOT_CMP_ENABLE);
    DMTimerReloadSet(SOC_DMTIMER_3_REGS, 0xffffffff);
    DMTimerCompareSet(SOC_DMTIMER_3_REGS, 0xfffff); 

    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);

    /* Interrupt register */
    TouchIntEnable();

    /* enable touch    */
    TouchEnable();
}

/*
** Timer 3 ISR
*/
static void Timer3Isr(void)
{
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);

    DMTimerDisable(SOC_DMTIMER_3_REGS);
    DMTimerCounterSet(SOC_DMTIMER_3_REGS, 0);
   
    touchRelease = 1;
}

/*
** This function tells if a touch is detected. 
*/
unsigned int TouchDetect(void)
{
    if(IsTSPress)
    {
        IsTSPress = 0;
        return TRUE;
    }

    else
    {
        return FALSE;
    }
}

/*
** This function tells if a touch is detected. 
*/
unsigned int TouchReleaseDetect(void)
{
    if(touchRelease)
    {
        touchRelease = 0;
        return TRUE;
    }

    else
    {
        return FALSE;
    }
}

/*
** This function resolves the coordinates of the location on the 
** touch screen being touched.
*/
void TouchCoOrdGet(int *pX, int *pY)
{
    *pX = (((x_data[dbidx] * 1278) + (y_data[dbidx] * 4))/10000) - 24;
    *pY = (((x_data[dbidx] * 5) +  (y_data[dbidx] * 788))/10000) - 22;

    *pX = LCD_WIDTH - *pX;
    *pY = LCD_HEIGHT - *pY;

    if(*pX > LCD_WIDTH)
    {
        *pX = LCD_WIDTH;
    }

    if(*pY > LCD_HEIGHT)
    {
        *pY = LCD_HEIGHT;
    }
}


/*
** Enables Touch Screen Interrupt
*/
void TouchIntEnable(void)
{
   /* Enable the FIFO Threshold interrupt */
   TSCADCEventInterruptEnable(SOC_ADC_TSC_0_REGS, TSCADC_FIFO1_THRESHOLD_INT);
}

/*
** Registers Touch Screen Interrupt
*/
void TouchIntRegister(void)
{
    IntRegister(SYS_INT_ADC_TSC_GENINT, TouchScreenIsr);

    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
}

/* Enables Touch Screen */
void TouchEnable(void)
{
    /* The Sequencer will start in IDLE state */
    TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_ENABLE);
    StepEnable();
}

/*
** ISR for Touch Screen
*/
static void TouchScreenIsr(void)
{
    volatile unsigned int wordsLeft = 0;
    unsigned int status;
    unsigned int i = 0;
    unsigned int prevVal_x = 0xFFFFFFFF;
    unsigned int prevVal_y = 0xFFFFFFFF;
    unsigned int currDiff_x;
    unsigned int currDiff_y;
    unsigned int prevDiff_x = 0xFFFFFFFF;
    unsigned int prevDiff_y = 0xFFFFFFFF;
    unsigned int readx1;
    unsigned int ready1;

    status = TSCADCIntStatus(SOC_ADC_TSC_0_REGS);

    wordsLeft = TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);

    if(status & TSCADC_FIFO1_THRESHOLD_INT)
    {
        for (i = 0; i < wordsLeft; i++)
        {
            readx1 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);
            readx1 = readx1 & 0xfff;

            if (readx1 > prevVal_x)
            {
                currDiff_x = readx1 - prevVal_x;
            }
            else
            {
                currDiff_x = prevVal_x - readx1;
            }
            if (currDiff_x < prevDiff_x)
            {
                prevDiff_x = currDiff_x;
                xdata = readx1;
            }
            prevVal_x = readx1;
 
            ready1 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);
            ready1 &= 0xfff;

            if (ready1 > prevVal_y)
            {
                currDiff_y = ready1 - prevVal_y;
            }
            else
            {
                currDiff_y = prevVal_y - ready1;
            }
            if (currDiff_y < prevDiff_y)
            {
                prevDiff_y = currDiff_y;
                ydata = ready1;
            }
            prevVal_y = ready1;
            wordsLeft = TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);
        }

        x_data[dbidx] = xdata;
        y_data[dbidx] = ydata;
        dbidx = (dbidx + 1) & 0x01;
    }

    /* Restart the timer counter */
    DMTimerCounterSet(SOC_DMTIMER_3_REGS, 0);
    DMTimerEnable(SOC_DMTIMER_3_REGS);

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS,
                         TSCADC_FIFO1_THRESHOLD_INT    |
                         TSCADC_ASYNC_HW_PEN_EVENT_INT |
                         TSCADC_SYNC_PEN_EVENT_INT     |
                         TSCADC_FIFO0_UNDER_FLOW_INT   |
                         TSCADC_FIFO1_UNDER_FLOW_INT   |
                         TSCADC_END_OF_SEQUENCE_INT    |
                         TSCADC_FIFO0_THRESHOLD_INT    |
                         TSCADC_FIFO0_OVER_RUN_INT     |
                         TSCADC_FIFO1_OVER_RUN_INT     |
                         TSCADC_OUT_OF_RANGE_INT       |
                         TSCADC_PEN_UP_EVENT_INT);

    IsTSPress = 1;
    touchRelease = 0;
   
    StepEnable();
}

/* Enables step */
void StepEnable(void)
{
    unsigned int i = 0;

    for(i = 0; i < 11; i++)
    {
        TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, i, 1);
    }
}

/* Disables step */
void StepDisable(void)
{
    unsigned int i = 0;

    for(i = 0; i < 11; i++)
    {
        TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, i, 0);
    }
}

static void StepConfigX(unsigned int stepSelc)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                     TSCADC_SINGLE_ENDED_OPER_MODE,
                                     stepSelc);

    /* Configure reference volatage and input to charge step*/
    TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSelc,
                       TSCADC_NEGATIVE_REF_VSSA, TSCADC_POSITIVE_INP_CHANNEL3,
                       TSCADC_NEGATIVE_INP_CHANNEL1, TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_ON,
                                   TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_OFF,
                                   stepSelc);

    /* Configure the Analong Ground to Touch screen */
    TSCADCTSStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_ON,
                                   TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_OFF,
                                   TSCADC_WPNSW_PIN_OFF, stepSelc);

    /* select fifo 0 */
    TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_FIFO_0);

    /* Configure in One short hardware sync mode */
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSelc,
                           TSCADC_ONE_SHOT_HARDWARE_SYNC);

    TSCADCTSStepAverageConfig(SOC_ADC_TSC_0_REGS, stepSelc,
                              TSCADC_SIXTEEN_SAMPLES_AVG);
}

static void StepConfigY(unsigned int stepSelc)
{

    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                     TSCADC_SINGLE_ENDED_OPER_MODE,
                                     stepSelc);

    /* Configure reference volatage and input to charge step*/
    TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_NEGATIVE_REF_VSSA,
                       TSCADC_POSITIVE_INP_CHANNEL1, TSCADC_NEGATIVE_INP_ADCREFM,
                       TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_OFF,
                                   TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_ON,
                                   stepSelc);

    /* Configure the Analong Ground to Touch screen */
    TSCADCTSStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_OFF,
                                   TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_ON,
                                   TSCADC_WPNSW_PIN_OFF, stepSelc);

    /* select fifo 0 */
    TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_FIFO_1);

    /* Configure in One short hardware sync mode */
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSelc,
                           TSCADC_ONE_SHOT_HARDWARE_SYNC);

    TSCADCTSStepAverageConfig(SOC_ADC_TSC_0_REGS, stepSelc,
                              TSCADC_SIXTEEN_SAMPLES_AVG);
}


void clearTSFifos(void)
{
    while(TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0))
    {
        TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);
    }

    while(TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1))
    {
        TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);
    }

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS,
                         TSCADC_FIFO1_THRESHOLD_INT |
                         TSCADC_FIFO0_THRESHOLD_INT);
}

/****************************** End of file **********************************/

