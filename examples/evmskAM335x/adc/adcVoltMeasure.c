/**
 * \file     adcVoltMeasure.c
 *
 * \brief    This application demonstrates the configuration and usage of 
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
 *               Hard coded configurations (compile time)
 *                   FIFO instance  - FIFO-0/1
 *
 *           Application Use Cases:
 *               The application demonstrates the following features
 *               of the ADC:
 *               1) Single ended operation mode for General purpose mode.
 *               2) Use of End of Sequence Interrupts.
 *               This application uses ADC to sense the voltage on AN0 and
 *               AN1 lines and displays the voltage on the serial console.
 *
 *           Running the example:
 *               1. A serial terminal application should be running on the host.
 *               2. Console displays the voltage read across AN0 and AN1 lines.
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

#include "consoleUtils.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "interrupt.h"
#include "hw_types.h"
#include "tsc_adc.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define  TSC_ADC_INSTANCE             SOC_ADC_TSC_0_REGS

#define  RESOL_X_MILLION              (439u)

#define  START_ADDR_DDR               (0x80000000)

#define  START_ADDR_DEV               (0x44000000)

#define  START_ADDR_OCMC              (0x40300000)

#define  NUM_SECTIONS_DDR             (512)

#define  NUM_SECTIONS_DEV             (960)

#define  NUM_SECTIONS_OCMC            (1)


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

/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif

/****************************************************************************/
/*             LOCAL FUNCTION DEFINITIONS                                   */
/****************************************************************************/

/*
** Function to setup MMU. This function Maps three regions ( 1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes 
    ** specified here are, 
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given. 
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}


int main(void)
{
    MMUConfigAndEnable();

    CacheEnable(CACHE_ALL);

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
    TSCADCConfigureAFEClock(TSC_ADC_INSTANCE, 24000000, 3000000);

    /* Enable Transistor bias */
    TSCADCTSTransistorConfig(TSC_ADC_INSTANCE, TSCADC_TRANSISTOR_ENABLE);

    TSCADCStepIDTagConfig(TSC_ADC_INSTANCE, 1);

    /* Disable Write Protection of Step Configuration regs*/
    TSCADCStepConfigProtectionDisable(TSC_ADC_INSTANCE);

    /* Configure step 1 for channel 1(AN0)*/
    StepConfigure(0, TSCADC_FIFO_0, TSCADC_POSITIVE_INP_CHANNEL1);

    /* Configure step 2 for channel 2(AN1)*/
    StepConfigure(1, TSCADC_FIFO_1, TSCADC_POSITIVE_INP_CHANNEL2);

    /* General purpose inputs */
    TSCADCTSModeConfig(TSC_ADC_INSTANCE, TSCADC_GENERAL_PURPOSE_MODE);

    /* Enable step 1 */
    TSCADCConfigureStepEnable(TSC_ADC_INSTANCE, 1, 1);

    /* Enable step 2 */
    TSCADCConfigureStepEnable(TSC_ADC_INSTANCE, 2, 1);

    /* Clear the status of all interrupts */
    CleanUpInterrupts();

    /* End of sequence interrupt is enable */
    TSCADCEventInterruptEnable(TSC_ADC_INSTANCE, TSCADC_END_OF_SEQUENCE_INT);

    /* Enable the TSC_ADC_SS module*/
    TSCADCModuleStateSet(TSC_ADC_INSTANCE, TSCADC_MODULE_ENABLE);
}

/* Configures the step */
void StepConfigure(unsigned int stepSel, unsigned int fifo,
                   unsigned int positiveInpChannel)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(TSC_ADC_INSTANCE,
                                  TSCADC_SINGLE_ENDED_OPER_MODE, stepSel);

    /* Configure step to select Channel, refernce voltages */
    TSCADCTSStepConfig(TSC_ADC_INSTANCE, stepSel, TSCADC_NEGATIVE_REF_VSSA,
                    positiveInpChannel, TSCADC_NEGATIVE_INP_CHANNEL1, TSCADC_POSITIVE_REF_VDDA);

    /* XPPSW Pin is on, Which pull up the AN0 line*/
    TSCADCTSStepAnalogSupplyConfig(TSC_ADC_INSTANCE, TSCADC_XPPSW_PIN_ON, TSCADC_XNPSW_PIN_OFF,
                                TSCADC_YPPSW_PIN_OFF, stepSel);

    /* XNNSW Pin is on, Which pull down the AN1 line*/
    TSCADCTSStepAnalogGroundConfig(TSC_ADC_INSTANCE, TSCADC_XNNSW_PIN_ON, TSCADC_YPNSW_PIN_OFF,
                                TSCADC_YNNSW_PIN_OFF,  TSCADC_WPNSW_PIN_OFF, stepSel);

    /* select fifo 0 or 1*/
    TSCADCTSStepFIFOSelConfig(TSC_ADC_INSTANCE, stepSel, fifo);

    /* Configure ADC to one short mode */
    TSCADCTSStepModeConfig(TSC_ADC_INSTANCE, stepSel,  TSCADC_ONE_SHOT_SOFTWARE_ENABLED);
}

/* Clear status of all interrupts */
static void CleanUpInterrupts(void)
{
    TSCADCIntStatusClear(TSC_ADC_INSTANCE, 0x7FF);
    TSCADCIntStatusClear(TSC_ADC_INSTANCE ,0x7FF);
    TSCADCIntStatusClear(TSC_ADC_INSTANCE, 0x7FF);
}

/* Reads the data from FIFO 0 and FIFO 1 */
static void ADCIsr()
{
    volatile unsigned int status;

    status = TSCADCIntStatus(TSC_ADC_INSTANCE);

    TSCADCIntStatusClear(TSC_ADC_INSTANCE, status);

    if(status & TSCADC_END_OF_SEQUENCE_INT)
    {
         /* Read data from fifo 0 */
         sample1 = TSCADCFIFOADCDataRead(TSC_ADC_INSTANCE, TSCADC_FIFO_0);

         /* Read data from fif 1*/
         sample2 = TSCADCFIFOADCDataRead(TSC_ADC_INSTANCE, TSCADC_FIFO_1);

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

