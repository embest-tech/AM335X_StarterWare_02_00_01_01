/**
 * \file    touch.c
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
#include "touch.h"
#include "soc_AM335x.h"
#include "hw_types.h"
#include "evmskAM335x.h"
#include "tsc_adc.h"
#include "dmtimer.h"


/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define SAMPLES                       5u
#define TSC_MODULE_CLK                24000000
#define TSC_INPUT_CLK                 3000000
#define TSC_CHARGESTEP_OPEN_DELAY     0x200
#define TSC_STEP_OPEN_DELAY           0x98
#define TSC_FIFO_THRLD_VALUE          5
#define DMTIMER_RELOAD_VAL            0xffffffff
#define DMTIMER_COMPARE_VAL           0xfffff
#define DMTIMER_CNT_VAL               0

/*******************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void TouchScreenIsr(void);
static void StepEnable(void);
static void Timer3Isr(void);
static void StepConfigX(unsigned int);
static void StepConfigY(unsigned int);

/*******************************************************************************
**                     INTERNAL VARIALE DEFINITIONS
*******************************************************************************/
volatile unsigned int x_data[2];
volatile unsigned int y_data[2];
volatile unsigned int error;
volatile unsigned int IsTSPress = 0;
volatile unsigned int penUp = 1;
volatile unsigned int numOfInt = 0;
unsigned int touchRelease = 0;
unsigned int dbidx = 0;


/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  Initializes the touch screen.\n                                     *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void TouchInit(void)
{
    unsigned int i;

     TSCADCModuleClkConfig();

     TSCADCPinMuxSetUp();

    /* configures ADC to 3Mhz */
    TSCADCConfigureAFEClock(SOC_ADC_TSC_0_REGS, TSC_MODULE_CLK, TSC_INPUT_CLK);

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

    TSCADCTSChargeStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, TSC_CHARGESTEP_OPEN_DELAY);
    for(i = 0; i < SAMPLES; i++)
    {
         StepConfigX(i);

         TSCADCTSStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, i, TSC_STEP_OPEN_DELAY);
    }

    for(i = SAMPLES; i < (2 * SAMPLES); i++)
    {
         StepConfigY(i);

         TSCADCTSStepOpenDelayConfig(SOC_ADC_TSC_0_REGS, i, TSC_STEP_OPEN_DELAY);
    }

    /* Configure FIFO1 threshold value */
    TSCADCFIFOIRQThresholdLevelConfig(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1, TSC_FIFO_THRLD_VALUE);

    IsTSPress = 0;
    touchRelease = 0;

    /* timer setup for touch screen */
    IntRegister(SYS_INT_TINT3, Timer3Isr);

    DMTimerModeConfigure(SOC_DMTIMER_3_REGS, DMTIMER_ONESHOT_CMP_ENABLE);
    DMTimerReloadSet(SOC_DMTIMER_3_REGS, DMTIMER_RELOAD_VAL);
    DMTimerCompareSet(SOC_DMTIMER_3_REGS, DMTIMER_COMPARE_VAL);

    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);
}

/******************************************************************************
*                                                                             *
* \brief  Timer 3 ISR.\n                                                      *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void Timer3Isr(void)
{
    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_MAT_EN_FLAG);

    DMTimerDisable(SOC_DMTIMER_3_REGS);
    DMTimerCounterSet(SOC_DMTIMER_3_REGS, DMTIMER_CNT_VAL);

    touchRelease = 1;
}

/******************************************************************************
*                                                                             *
* \brief  This function tells if a touch is detected.\n                       *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return TRUE : If touch is detected.\n                                      *
*                                                                             *
* \return FALSE: If touch is not detected.\n                                  *
*                                                                             *
******************************************************************************/
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

/******************************************************************************
*                                                                             *
* \brief  This function tells if a touch release is detected.\n               *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return TRUE : If touch release is detected.\n                              *
*                                                                             *
* \return FALSE: If touch release is not detected.\n                          *
*                                                                             *
******************************************************************************/
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

/******************************************************************************
*                                                                             *
* \brief  This function resolves the coordinates of the location on the       *
*         touch screen being touched.\n                                       *
*                                                                             *
* \param                                                                      *
*           pX : Address of variable which carries x coordinate.              *
*                                                                             *
*           pY : Address of variable which carries Y coordinate.              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void TouchCoOrdGet(int *pX, int *pY)
{
    *pX = (((x_data[dbidx] * 1278) -  (y_data[dbidx] * 4))/10000) - 24;
    *pY = (((x_data[dbidx] * 5) +  (y_data[dbidx] * 788))/10000) -  22;

    *pX = 480 - *pX;
    *pY = 272 - *pY;

    if(*pX > 480)
    {
        *pX = 480;
    }

    if(*pY > 272)
    {
        *pY = 272;
    }
}

/******************************************************************************
*                                                                             *
* \brief  Enables Touch Screen Interrupt.\n                                   *
*                                                                             *
* \param  none.\n                                                             *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void TouchIntEnable(void)
{
   /* Enable the FIFO Threshold interrupt */
   TSCADCEventInterruptEnable(SOC_ADC_TSC_0_REGS, TSCADC_FIFO1_THRESHOLD_INT);
}

/******************************************************************************
*                                                                             *
* \brief  Registers Touch Screen Interrupt.\n                                 *
*                                                                             *
* \param  none.\n                                                             *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void TouchIntRegister(void)
{
    IntRegister(SYS_INT_ADC_TSC_GENINT, TouchScreenIsr);
}

/******************************************************************************
*                                                                             *
* \brief  Enables Touch Screen.\n                                             *
*                                                                             *
* \param  none.\n                                                             *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void TouchEnable(void)
{
    /* The Sequencer will start in IDLE state */
    TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_ENABLE);
    StepEnable();
}

/******************************************************************************
*                                                                             *
* \brief  ISR for Touch Screen.\n                                             *
*                                                                             *
* \param  none.\n                                                             *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
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
    unsigned int xdata = 0;
    unsigned int ydata = 0;

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

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS,  TSCADC_FIFO1_THRESHOLD_INT);

    IsTSPress = 1;
    touchRelease = 0;

    StepEnable();
}

/******************************************************************************
*                                                                             *
* \brief  Enables step.\n                                                     *
*                                                                             *
* \param  none.\n                                                             *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void StepEnable(void)
{
    unsigned int i = 0;

    for(i = 0; i < 11; i++)
    {
         TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, i, 1);
    }
}

/******************************************************************************
*                                                                             *
* \brief  Step Config for X.\n                                                *
*                                                                             *
* \param  stepSelc   Step config register value.\n                            *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void StepConfigX(unsigned int stepSelc)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                    TSCADC_SINGLE_ENDED_OPER_MODE, stepSelc);

    /* Configure reference volatage and input to charge step*/
    TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSelc,TSCADC_NEGATIVE_REF_VSSA,
                       TSCADC_POSITIVE_INP_CHANNEL3,TSCADC_NEGATIVE_INP_CHANNEL1,
                       TSCADC_POSITIVE_REF_VDDA);

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
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_ONE_SHOT_HARDWARE_SYNC);

    TSCADCTSStepAverageConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_SIXTEEN_SAMPLES_AVG);
}

/******************************************************************************
*                                                                             *
* \brief  Step Config for Y.\n                                                *
*                                                                             *
* \param  stepSelc   Step config register value.\n                            *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void StepConfigY(unsigned int stepSelc)
{

    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                     TSCADC_SINGLE_ENDED_OPER_MODE, stepSelc);

    /* Configure reference volatage and input to charge step*/
    TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_NEGATIVE_REF_VSSA,
                       TSCADC_POSITIVE_INP_CHANNEL1, TSCADC_NEGATIVE_INP_ADCREFM,
                       TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_OFF,
                                   TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_ON, stepSelc);

    /* Configure the Analong Ground to Touch screen */
    TSCADCTSStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_OFF,
                                   TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_ON,
                                   TSCADC_WPNSW_PIN_OFF, stepSelc);

    /* select fifo 0 */
    TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_FIFO_1);

    /* Configure in One short hardware sync mode */
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_ONE_SHOT_HARDWARE_SYNC);

    TSCADCTSStepAverageConfig(SOC_ADC_TSC_0_REGS, stepSelc, TSCADC_SIXTEEN_SAMPLES_AVG);
}

/****************************** End of file **********************************/



