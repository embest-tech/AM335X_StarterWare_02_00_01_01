/**
 * \file    tscCalibrate.c
    *
 * \brief    This application demonstrates the configuration and usage of TSC.
 *
 *           Application Configuration:
 *               Modules Used:
 *                   ADC_TouchScreen controller
 *                   Interrupt Controller
 *                   UART0
 *
 *               Configurable Parameters(Runtime)
 *                   None
 *
 *              Hard coded configurations (compile time)
 *                   FIFO instance  - Fixed at FIFO-0
 *                   FIFO threshold level - Fixed at 5
 *                   Step Averaging - Fixed at 16 samples
 *                   Step configuration - Fixed at 5
 *
 *           Application Use Cases:
 *               The application demonstrates the following features
 *               of the TouchScreen:
 *               1) Single ended operation mode for a 4 wire touchscreen mode.
 *               2) Use of FIFO threshold level interrupts
 *               This example calibrates the TouchScreen by asking user for some
 *               touch inputs. Later on it displays the coordinates for every 
 *               touch event on the display Panel.
 *
 *           Running the example:
 *               1. A serial terminal application should be running on the host.
 *               2. Console displays message saying that Touch Bottom,Touch
 *                  Left etc to calibrate co-ordinates.
 *               3. It will display x and y-coordinates of Touch Screen panel.
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
#include "interrupt.h"
#include "hw_types.h"
#include "evmskAM335x.h"
#include "tsc_adc.h"
#include "dmtimer.h"
#include "hw_types.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void SetupIntc(void);
static void StepEnable(void);
static void FIFOConfigure(void);
static void TSModuleEnable(void);
static void TouchScreenIsr(void);
static void TouchCalibrate(void);
static void TouchScreenInit(void);
static void SetUPTSADCControl(void);
static void ReadTouchScreenPress(void);
static void ReadTouchScreenPress(void);
static void StepConfigX(unsigned int);
static void StepConfigY(unsigned int);
static void IdleStepConfig(void);
static void TSchargeStepConfig(void);
int setCalibrationMatrix(POINT * displayPtr,
                         POINT * screenPtr,
                         MATRIX * matrixPtr);

int getDisplayPoint(POINT * displayPtr,
                    POINT * screenPtr,
                    MATRIX * matrixPtr);


/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define TSC_ADC_INSTANCE     SOC_ADC_TSC_0_REGS

#define SAMPLES              5

#define LCD_WIDTH            480

#define LCD_HEIGHT           272

#define  START_ADDR_DDR      (0x80000000)

#define  START_ADDR_DEV      (0x44000000)

#define  START_ADDR_OCMC     (0x40300000)

#define  NUM_SECTIONS_DDR    (512)

#define  NUM_SECTIONS_DEV    (960)

#define  NUM_SECTIONS_OCMC   (1)

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
volatile unsigned int x_val[2];
volatile unsigned int y_val[2];
volatile unsigned int error;
volatile unsigned int IsTSPress;
volatile unsigned int numOfInt = 0;
volatile unsigned int Index =0;
MATRIX stMatrix;

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

    /* This function will enable clocks for the DMTimer2 instance */
    DMTimer2ModuleClkConfig();

    TouchScreenInit();

    return 0;
}

static void SetupIntc(void)
{
    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    IntRegister(SYS_INT_ADC_TSC_GENINT, TouchScreenIsr);

    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
}

static void TouchScreenInit(void)
{
    unsigned int i = 0;
      
    TSCADCModuleClkConfig();

    TSCADCPinMuxSetUp();

    /* configures ADC to 3Mhz */ 
    TSCADCConfigureAFEClock(TSC_ADC_INSTANCE, 24000000, 3000000);

    SetUPTSADCControl();

    /* Disable Write Protection of Step Configuration regs*/
    TSCADCStepConfigProtectionDisable(TSC_ADC_INSTANCE);

    /* Touch Screen detection Configuration*/
    IdleStepConfig();

    /* Configure the Charge step */
    TSchargeStepConfig();

    for(i = 0; i < SAMPLES; i++)
    {
         StepConfigX(i);

         TSCADCTSStepOpenDelayConfig(TSC_ADC_INSTANCE, i, 0x98);
    }

    for(i = SAMPLES; i < (2 * SAMPLES); i++)
    {
         StepConfigY(i);

         TSCADCTSStepOpenDelayConfig(TSC_ADC_INSTANCE, i, 0x98);
   }

   /* Configure FIFO */
   FIFOConfigure();

   /* Enable the FIFO Threshold interrupt */
   TSCADCEventInterruptEnable(TSC_ADC_INSTANCE, TSCADC_FIFO1_THRESHOLD_INT);

   /* Enable the Touchscreen module */
   TSModuleEnable();

   /* Enable the steps */
   StepEnable();

   /* Calibrate the touch Screen */
   TouchCalibrate();

   /* Software is ready to measure the Touch Position */
   ReadTouchScreenPress();
}


static void SetUPTSADCControl(void)
{
   /* Enable Transistor bias */
   TSCADCTSTransistorConfig(TSC_ADC_INSTANCE, TSCADC_TRANSISTOR_ENABLE);

   /* Map hardware event to Pen Touch IRQ */
   TSCADCHWEventMapSet(TSC_ADC_INSTANCE, TSCADC_PEN_TOUCH);

   /* Set 4 Wire or 5 wire touch screen  mode */
   TSCADCTSModeConfig(TSC_ADC_INSTANCE, TSCADC_FOUR_WIRE_MODE);

   TSCADCStepIDTagConfig(TSC_ADC_INSTANCE, 1);
}

static void FIFOConfigure(void)
{
    /* Configure FIFO1 threshold value */
    TSCADCFIFOIRQThresholdLevelConfig(TSC_ADC_INSTANCE, TSCADC_FIFO_1, 5);
}

static void IdleStepConfig(void)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCIdleStepOperationModeControl(TSC_ADC_INSTANCE, 
                                    TSCADC_SINGLE_ENDED_OPER_MODE);

    /* Configure reference volatage and input to idlestep */ 
    TSCADCIdleStepConfig(TSC_ADC_INSTANCE, TSCADC_NEGATIVE_REF_VSSA,
                         TSCADC_POSITIVE_INP_CHANNEL1, TSCADC_NEGATIVE_INP_ADCREFM,
                         TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCIdleStepAnalogSupplyConfig(TSC_ADC_INSTANCE, TSCADC_XPPSW_PIN_OFF,
                                     TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_OFF);

     /* 
     **Configure the Analong Ground of Touch screen.
     */
    TSCADCIdleStepAnalogGroundConfig(TSC_ADC_INSTANCE, TSCADC_XNNSW_PIN_OFF,
                                     TSCADC_YPNSW_PIN_ON, TSCADC_YNNSW_PIN_ON,
                                     TSCADC_WPNSW_PIN_OFF);
}

static void TSchargeStepConfig(void)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCChargeStepOperationModeControl(TSC_ADC_INSTANCE, 
                                         TSCADC_SINGLE_ENDED_OPER_MODE);

    /* Configure reference volatage and input to charge step*/ 
    TSCADCChargeStepConfig(TSC_ADC_INSTANCE, TSCADC_NEGATIVE_REF_XNUR,
                           TSCADC_POSITIVE_INP_CHANNEL2, TSCADC_NEGATIVE_INP_CHANNEL2,
                           TSCADC_POSITIVE_REF_XPUL);

    /* Configure the Analog Supply to Touch screen */
    TSCADCChargeStepAnalogSupplyConfig(TSC_ADC_INSTANCE, TSCADC_XPPSW_PIN_ON,
                                       TSCADC_XNPSW_PIN_OFF, TSCADC_XPPSW_PIN_OFF);

    /* Configure the Analong Ground to Touch screen */
    TSCADCChargeStepAnalogGroundConfig(TSC_ADC_INSTANCE, TSCADC_XNNSW_PIN_OFF,
                                       TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_ON,
                                       TSCADC_WPNSW_PIN_OFF);

    TSCADCTSChargeStepOpenDelayConfig(TSC_ADC_INSTANCE, 0x200);
}

static void StepConfigX(unsigned int stepSelc)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(TSC_ADC_INSTANCE,
                                    TSCADC_SINGLE_ENDED_OPER_MODE, stepSelc);

    /* Configure reference volatage and input to charge step*/ 
    TSCADCTSStepConfig(TSC_ADC_INSTANCE, stepSelc,TSCADC_NEGATIVE_REF_VSSA,
                       TSCADC_POSITIVE_INP_CHANNEL3,TSCADC_NEGATIVE_INP_CHANNEL1,
                       TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCTSStepAnalogSupplyConfig(TSC_ADC_INSTANCE, TSCADC_XPPSW_PIN_ON,
                                   TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_OFF,
                                   stepSelc);

    /* Configure the Analong Ground to Touch screen */
    TSCADCTSStepAnalogGroundConfig(TSC_ADC_INSTANCE, TSCADC_XNNSW_PIN_ON,
                                   TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_OFF,
                                   TSCADC_WPNSW_PIN_OFF, stepSelc);

    /* select fifo 0 */
    TSCADCTSStepFIFOSelConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_FIFO_0);

    /* Configure in One short hardware sync mode */
    TSCADCTSStepModeConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_ONE_SHOT_HARDWARE_SYNC);
    
    TSCADCTSStepAverageConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_SIXTEEN_SAMPLES_AVG); 
}

static void StepConfigY(unsigned int stepSelc)
{

    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(TSC_ADC_INSTANCE, 
                                     TSCADC_SINGLE_ENDED_OPER_MODE, stepSelc);

    /* Configure reference volatage and input to charge step*/ 
    TSCADCTSStepConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_NEGATIVE_REF_VSSA,
                       TSCADC_POSITIVE_INP_CHANNEL1, TSCADC_NEGATIVE_INP_ADCREFM,
                       TSCADC_POSITIVE_REF_VDDA);

    /* Configure the Analog Supply to Touch screen */
    TSCADCTSStepAnalogSupplyConfig(TSC_ADC_INSTANCE, TSCADC_XPPSW_PIN_OFF,
                                   TSCADC_XNPSW_PIN_OFF, TSCADC_YPPSW_PIN_ON, stepSelc);

    /* Configure the Analong Ground to Touch screen */
    TSCADCTSStepAnalogGroundConfig(TSC_ADC_INSTANCE, TSCADC_XNNSW_PIN_OFF,
                                   TSCADC_YPNSW_PIN_OFF, TSCADC_YNNSW_PIN_ON,
                                   TSCADC_WPNSW_PIN_OFF, stepSelc);

    /* select fifo 0 */
    TSCADCTSStepFIFOSelConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_FIFO_1);

    /* Configure in One short hardware sync mode */
    TSCADCTSStepModeConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_ONE_SHOT_HARDWARE_SYNC);

    TSCADCTSStepAverageConfig(TSC_ADC_INSTANCE, stepSelc, TSCADC_SIXTEEN_SAMPLES_AVG); 
}

static void StepEnable(void)
{
    unsigned int i = 0;

    for(i = 0; i < 11; i++)
    {
         TSCADCConfigureStepEnable(TSC_ADC_INSTANCE, i, 1);
    }
}

static void TSModuleEnable(void)
{
    /* The Sequencer will start in IDLE state */
    TSCADCModuleStateSet(TSC_ADC_INSTANCE, TSCADC_MODULE_ENABLE);
}


static void TouchScreenIsr()
{
    unsigned int wordsLeft = 0;
    unsigned int status;
    unsigned int arr_x[5] = {0,0,0,0,0};
    unsigned int arr_y[5] = {0,0,0,0,0};
    unsigned int x_data = 0;
    unsigned int y_data = 0;
    unsigned int i = 0;
    unsigned int sum = 0;

    status = TSCADCIntStatus(TSC_ADC_INSTANCE);

    if(status & TSCADC_FIFO1_THRESHOLD_INT)
    {
         wordsLeft = TSCADCFIFOWordCountRead(TSC_ADC_INSTANCE, TSCADC_FIFO_0);

         while(wordsLeft)
         {
              x_data = TSCADCFIFOADCDataRead(TSC_ADC_INSTANCE, TSCADC_FIFO_0);

              arr_x[i++] = x_data;

              wordsLeft = TSCADCFIFOWordCountRead(TSC_ADC_INSTANCE, TSCADC_FIFO_0);
         }

         sum = arr_x[1] + arr_x[2] + arr_x[3];

         sum = sum / 3;

         x_data = sum;

         wordsLeft = TSCADCFIFOWordCountRead(TSC_ADC_INSTANCE, TSCADC_FIFO_1);

         i = 0;

         while(wordsLeft)
         {
              y_data = TSCADCFIFOADCDataRead(TSC_ADC_INSTANCE, TSCADC_FIFO_1);

              arr_y[i++] = y_data;

              wordsLeft = TSCADCFIFOWordCountRead(TSC_ADC_INSTANCE, TSCADC_FIFO_1);
         }

         sum = arr_y[1] + arr_y[2] + arr_y[3];

         sum = sum / 3;

         y_data = sum;

    }

    x_val[Index] = x_data;
    y_val[Index] = y_data;

    Index = (Index + 0x01) & 0x01;

    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, 0);

    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_2_REGS);

    TSCADCIntStatusClear(TSC_ADC_INSTANCE,  TSCADC_FIFO1_THRESHOLD_INT);

    IsTSPress = 1;

    StepEnable();
}

/*     Function: TouchCalibrate()
 *
 *     Description: Ask the user to touch the predefined coordinates
 *                  and read the correspoding touch screen driver values.
 *                  Collect 3 sets of values and pass to the
 *                  setCalibrationMatrix() function.
 */

static void TouchCalibrate(void)
{
    unsigned char i;
 
    POINT stDisplayPoint[3] = {{0, 0},{LCD_WIDTH, 0}, {0, LCD_HEIGHT}};
    POINT stTouchScreenPoint[3];

    ConsoleUtilsPrintf("Touch at Right bottom");

    while(!IsTSPress);

    IsTSPress = 1; 

    for(i = 0; i < 3; i++)
    {
         while(DMTimerCounterGet(SOC_DMTIMER_2_REGS) < 0xffffff);

         DMTimerDisable(SOC_DMTIMER_2_REGS);

         DMTimerCounterSet(SOC_DMTIMER_2_REGS, 0);

	 stTouchScreenPoint[i].x = x_val[0];
	 stTouchScreenPoint[i].y = y_val[0];

         if(i == 0)
         {
              ConsoleUtilsPrintf("\r\n");
              ConsoleUtilsPrintf("Touch at Left bottom");

         }
         else if(i == 1)
         {
              ConsoleUtilsPrintf("\r\n");
              ConsoleUtilsPrintf("Touch at Right Top");
         }
         else
         {
              ConsoleUtilsPrintf("\r\n");
         }
    }

    setCalibrationMatrix( stDisplayPoint, stTouchScreenPoint, &stMatrix);
}
  

static void ReadTouchScreenPress(void)
{
   int xDpos;
   int yDpos;
   POINT stDisplayPoint;

   POINT stTouchScreenPoint;
   do {
              if(IsTSPress) 
              {  
                   IsTSPress = 0;                 
 		   stTouchScreenPoint.x = x_val[0];
		   stTouchScreenPoint.y = y_val[0];
		   getDisplayPoint(&stDisplayPoint, &stTouchScreenPoint, &stMatrix);
                
                   xDpos = stDisplayPoint.x;

                   xDpos = LCD_WIDTH - xDpos;

                   if(xDpos < 0)
                   {
                        xDpos = 0;
                   }

                   if(xDpos > LCD_WIDTH)
                   {
                        xDpos = LCD_WIDTH;
                   }

                   ConsoleUtilsPrintf("xDpos=");

                   ConsoleUtilsPrintf("%d", xDpos);

                   yDpos = stDisplayPoint.y;

                   yDpos = LCD_HEIGHT - yDpos;

                   if(yDpos < 0)
                   {
                        yDpos = 0;
                   }

                   if(yDpos > LCD_HEIGHT)
                   {
                        yDpos = LCD_HEIGHT;
                   }
                   
                   ConsoleUtilsPrintf("yDpos=");

                   ConsoleUtilsPrintf("%d", yDpos);

                   ConsoleUtilsPrintf("\r\n");
                  
                 }
    } while (1);
}

/**********************************************************************
 *  
 *       Function: setCalibrationMatrix()
 *    
 *       Description: Calling this function with valid input data
 *                    in the display and screen input arguments 
 *                    causes the calibration factors between the
 *                    screen and display points to be calculated,
 *                    and the output argument - matrixPtr - to be 
 *                    populated.
 *           
 *                    This function needs to be called only when new
 *                    calibration factors are desired.
 *                             
 *                 
 *       Argument(s): displayPtr (input) - Pointer to an array of three 
 *                                         sample, reference points.
 *                    screenPtr (input) - Pointer to the array of touch 
 *                                         screen points corresponding 
 *                                         to the reference display points.
 *                    matrixPtr (output) - Pointer to the calibration 
 *                                         matrix computed for the set
 *                                         of points being provided.
 */

int setCalibrationMatrix(POINT *displayPtr,
                         POINT *screenPtr,
                         MATRIX *matrixPtr)
{
        int  temp;
        int  temp1;
        int  temp2;
        int  retValue = 0;

        matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                             ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));

        if( matrixPtr->Divider == 0 )
        {
                retValue = -1 ;
        }
        else
        {
              temp = ((displayPtr[0].x - displayPtr[2].x) 
                      * (screenPtr[1].y - screenPtr[2].y));

              temp1 = ((displayPtr[1].x - displayPtr[2].x) 
                      * (screenPtr[0].y - screenPtr[2].y));

              matrixPtr->An =((temp - temp1)) / (matrixPtr->Divider /10000);

              temp = ((screenPtr[0].x - screenPtr[2].x) 
                     * (displayPtr[1].x - displayPtr[2].x));

              temp1 = ((displayPtr[0].x - displayPtr[2].x) 
                      * (screenPtr[1].x - screenPtr[2].x));

              matrixPtr->Bn = ((temp - temp1)) / (matrixPtr->Divider / 10000);

              temp  = ((screenPtr[2].x * displayPtr[1].x 
                      - screenPtr[1].x * displayPtr[2].x)) * screenPtr[0].y;

              temp1 = ((screenPtr[0].x * displayPtr[2].x 
                      - screenPtr[2].x * displayPtr[0].x)) * screenPtr[1].y;

              temp2 = ((screenPtr[1].x * displayPtr[0].x 
                      - screenPtr[0].x * displayPtr[1].x)) * screenPtr[2].y;

              matrixPtr->Cn = ((temp + temp1 + temp2)) / matrixPtr->Divider;

              temp  = ((displayPtr[0].y - displayPtr[2].y) 
                      * (screenPtr[1].y - screenPtr[2].y));

              temp1 = ((displayPtr[1].y - displayPtr[2].y) 
                      * (screenPtr[0].y - screenPtr[2].y));

              matrixPtr->Dn = ((temp - temp1) / (matrixPtr->Divider / 10000));

              temp =  ((screenPtr[0].x - screenPtr[2].x) 
                      * (displayPtr[1].y - displayPtr[2].y));

              temp1 = ((displayPtr[0].y - displayPtr[2].y) 
                      * (screenPtr[1].x - screenPtr[2].x));

              matrixPtr->En= ((temp - temp1)) / (matrixPtr->Divider / 10000);

              temp =  (screenPtr[2].x * displayPtr[1].y 
                      - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y;

              temp1 = (screenPtr[0].x * displayPtr[2].y 
                      - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y;

              temp2 = (screenPtr[1].x * displayPtr[0].y 
                      - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;

              matrixPtr->Fn = (temp + temp1 + temp2) /matrixPtr->Divider;
        }

        return( retValue ) ;


} /* end of setCalibrationMatrix() */

/**********************************************************************
 *  
 *        Function: getDisplayPoint()
 *    
 *       Description: Given a valid set of calibration factors and a point
 *                    value reported by the touch screen, this function
 *                    calculates and returns the true (or closest to true)
 *                    display point below the spot where the touch screen 
 *                    was touched.
 *           
 *           
 *             
 *       Argument(s): displayPtr (output) - Pointer to the calculated
 *                                          (true) display point.
 *                    ScreenPtr (input) -   Pointer to the reported touch
 *                                          screen point.
 *                    matrixPtr (input) -   Pointer to calibration factors
 *                                          matrix previously calculated
 *                                          from a call to setCalibrationMatrix()
 */
int getDisplayPoint(POINT *displayPtr,
                    POINT *screenPtr,
                    MATRIX *matrixPtr )
{
    int  retValue = 0 ;
    
    if( matrixPtr->Divider != 0 )
    {
         displayPtr->x = (((matrixPtr->An * screenPtr->x + 
                            matrixPtr->Bn * screenPtr->y) / 10000)
                            + (matrixPtr->Cn));

	 displayPtr->y = (((matrixPtr->Dn * screenPtr->x +
			    matrixPtr->En * screenPtr->y) / 10000) 
                            + (matrixPtr->Fn));
    }
    else
    {
         retValue = -1 ;
    }

    return(retValue) ;

} /* end of getDisplayPoint() */

