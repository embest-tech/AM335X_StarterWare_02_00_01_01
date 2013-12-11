/**
 * \file     adcVoltMeasure.c
 *
 * \brief    This application for demonstrates the configuration and usage of 
 *           ADC. 
 *
 *           Application Configuration:
 *
 *               Modules Used:
 *                   ADC_TouchScreen controller
 *                   Interrupt Controller
 *                   UART0
 *
 *               Configurable parameters(Runtime):
 *                   None
 *
 *               Hard coded configurations(compile time)
 *                   FIFO instance  - FIFO-0/1
 *
 *           Application Use Cases:
 *               The application demonstrates the following features
 *               of the ADC:
 *               1) Single ended operation mode for General purpose mode.
 *               2) Use of End of Sequence Interrupts.
 *               This application uses ADC to sense the voltage on AN0 and
 *               AN1 lines and display the voltage on the serial console.
 *
 *           Running the example:
 *               1. A serial terminal application should be running on the host.
 *               2. Console displays the voltage read across AN0 and AN1 lines.
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
* 
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
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "hw_types.h"
#include "tsc_adc.h"


/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define RESOL_X_MILLION            (439u)

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void ADCIsr();
static void SetupIntc(void);
static void ADCConfigure(void);
static void CleanUpInterrupts(void);
static void StepConfigure(unsigned int, unsigned int, unsigned int);
/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
volatile unsigned int flag = 1;
unsigned int sample1;
unsigned int sample2;
unsigned int val1;
unsigned int val2;

/****************************************************************************/
/*             LOCAL FUNCTION DEFINITIONS                                   */
/****************************************************************************/

int main(void)
{
    SetupIntc();

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    ADCConfigure();

    while(flag);

    val1 = (sample1 * RESOL_X_MILLION) / 1000;

    ConsoleUtilsPrintf("Voltage sensed on the AN0 line : ");

    ConsoleUtilsPrintf("%d", val1);

    ConsoleUtilsPrintf("mV\r\n");

    val2 = (sample2 * RESOL_X_MILLION) / 1000;

    ConsoleUtilsPrintf("Voltage sensed on the AN1 line : ");

    ConsoleUtilsPrintf("%d", val2);

    ConsoleUtilsPrintf("mV\r\n");

    while(1);

}

/* ADC is configured */
static void ADCConfigure(void)
{
    /* Enable the clock for touch screen */
    TSCADCModuleClkConfig();

    TSCADCPinMuxSetUp();

    /* Configures ADC to 3Mhz */
    TSCADCConfigureAFEClock(SOC_ADC_TSC_0_REGS, 24000000, 3000000);

    /* Enable Transistor bias */
    TSCADCTSTransistorConfig(SOC_ADC_TSC_0_REGS, TSCADC_TRANSISTOR_ENABLE);

    TSCADCStepIDTagConfig(SOC_ADC_TSC_0_REGS, 1);

    /* Disable Write Protection of Step Configuration regs*/
    TSCADCStepConfigProtectionDisable(SOC_ADC_TSC_0_REGS);

    /* Configure step 1 for channel 1(AN0)*/
    StepConfigure(0, TSCADC_FIFO_0, TSCADC_POSITIVE_INP_CHANNEL1);

    /* Configure step 2 for channel 2(AN1)*/
    StepConfigure(1, TSCADC_FIFO_1, TSCADC_POSITIVE_INP_CHANNEL2);

    /* General purpose inputs */
    TSCADCTSModeConfig(SOC_ADC_TSC_0_REGS, TSCADC_GENERAL_PURPOSE_MODE);

    /* Enable step 1 */
    TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 1, 1);

    /* Enable step 2 */
    TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 2, 1);

    /* Clear the status of all interrupts */
    CleanUpInterrupts();

    /* End of sequence interrupt is enable */
    TSCADCEventInterruptEnable(SOC_ADC_TSC_0_REGS, TSCADC_END_OF_SEQUENCE_INT);

    /* Enable the TSC_ADC_SS module*/
    TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_ENABLE);
}

/* Configures the step */
void StepConfigure(unsigned int stepSel, unsigned int fifo,
                   unsigned int positiveInpChannel)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                  TSCADC_SINGLE_ENDED_OPER_MODE, stepSel);

    /* Configure step to select Channel, refernce voltages */
    TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSel, TSCADC_NEGATIVE_REF_VSSA,
                    positiveInpChannel, TSCADC_NEGATIVE_INP_CHANNEL1, TSCADC_POSITIVE_REF_VDDA);

    /* XPPSW Pin is on, Which pull up the AN0 line*/
    TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_ON, TSCADC_XNPSW_PIN_OFF,
                                TSCADC_YPPSW_PIN_OFF, stepSel);

    /* XNNSW Pin is on, Which pull down the AN1 line*/
    TSCADCTSStepAnalogGroundConfig(SOC_ADC_TSC_0_REGS, TSCADC_XNNSW_PIN_ON, TSCADC_YPNSW_PIN_OFF,
                                TSCADC_YNNSW_PIN_OFF,  TSCADC_WPNSW_PIN_OFF, stepSel);

    /* select fifo 0 or 1*/
    TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSel, fifo);

    /* Configure ADC to one short mode */
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSel,  TSCADC_ONE_SHOT_SOFTWARE_ENABLED);
}

/* Clear status of all interrupts */
static void CleanUpInterrupts(void)
{
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, 0x7FF);
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS ,0x7FF);
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, 0x7FF);
}

/* Reads the data from FIFO 0 and FIFO 1 */
static void ADCIsr()
{
    volatile unsigned int status;

    status = TSCADCIntStatus(SOC_ADC_TSC_0_REGS);

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, status);

    if(status & TSCADC_END_OF_SEQUENCE_INT)
    {
         /* Read data from fifo 0 */
         sample1 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);

         /* Read data from fif 1*/
         sample2 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);

         flag = 0;
    }
}

static void SetupIntc(void)
{
    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    IntRegister(SYS_INT_ADC_TSC_GENINT, ADCIsr);

    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
}

