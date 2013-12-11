/**
 * \file    demoMain.c
 *
 * \brief   This is a OOB (Out-Of-Box) demo application. This demonstrates
 *          use case of selected peripherals.
 *
 *          Application Configuration:
 *
 *              Modules Used:
 *                  Cortex-M3
 *                  EDMA
 *                  Ethernet
 *                  GPIO0-1
 *                  I2C0
 *                  Interrupt
 *                  Mailbox0
 *                  MMCSD0
 *                  Power Management
 *                  PRCM
 *                  RTC0
 *                  Timer1-7
 *                  UART0
 *
 *              Configurable Parameters:
 *                  Power Management - Sleep Mode: DS0
 *                                                 DS1
 *                                                 Standby
 *                                   - Wake Source: UART0
 *                                                  Timer1
 *                                                  Timer6
 *                  RTC - Set Timer
 *
 *              Hard-coded configuration of other parameters:
 *                  IP Address - DHCP is default configuration. For manual
 *                               configuration use STATIC_IP_ADDRESS_PORT1
 *                               in lwipopts.h.
 *                  Console interface - Uart interface is default configuration.
 *                                      For semihosting set ConsoleUtilsSetType
 *                                      to CONSOLE_DEBUGGER and set compile time
 *                                      macro CONSOLE as SEMIHOSTING.
 *
 *          Application Usecase:
 *              Cortex-M3 - Perform suspend/resume of MPU
 *              Ethernet - Host web page for selection of use case
 *              I2C0 - PMIC control
 *              MMCSD0 - Perform commandline operation on SD Card
 *              PRCM - Configure the operating voltage and frequency of MPU
 *              Power Management - Demonstrate execution of sleep modes
 *              RTC0 - Set Timer for seconds tick
 *              Timer2 - Overflow counter
 *              UART0 - Console operation
 *
 *          Running example:
 *              1. Load example and run.
 *              2. Connect board to network and use WWW page to configure the IP
 *                 address. Use web page to navigate use cases.
 *              3. UART console provides interface to configure the parameters
 *                 listed under configurable parameters.
 *              4. Select RTC page to set time and date through console
 *              5. Select Timer page to switch on and off LED timer overflow.
 *              6. Select GPIO page to switch on LED.
 *              7. Select MMCSD page to perform commandline operations on SD
 *                 card formatted for FAT filesystem.
 *              8. Select PM page to configure the wake source and execute sleep
 *                 modes.
 *
 *          Limitations:
 *              None.
 *
 *          Note:
 *              1. OOB is not fully complaint to semihosting.  To navigate OOB
 *                 through console interface non-blocking uart interface is used.
 *                 The same is not possible with semihosting console interface.
 *                 If semihosting is enabled for this example, commandline instr
 *                 has to be provided through uart but the same is displayed on
 *                 debugger console interface. Same is applicable for MMCSD
 *                 usecase. It is recommended to use Uart console interface only.
 *              2. On timer wake source selection timer1 is configured as wake
 *                 source for DS0 and DS1 mode and timer6 is configured as wake
 *                 source for standby.
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

#include "demoPwrMgmnt.h"
#include "interrupt.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "demoCfg.h"
#include "demoTimer.h"
#include "demoEnet.h"
#include "demoLedIf.h"
#include "demoRtc.h"
#include "error.h"
#include "cache.h"
#include "mmu.h"
#include "board.h"
#include "device.h"
#include "beaglebone.h"
#include "uartStdio.h"
#include "consoleUtils.h"
#include "demoSdRw.h"
#include "demoSdFs.h"
#include "delay.h"
#include <string.h>
#include "mmcsd_proto.h"
#include "gpio_v2.h"
#include "pin_mux.h"
#include "clock.h"
#include "demoI2c.h"
#include "uart_irda_cir.h"
#include "cm3wkup_proxy.h"
#include "clock.h"
#include "demoDvfs.h"

/****************************************************************************
**                   INTERNAL MACRO DEFINITIONS                                       
****************************************************************************/
#define NUM_OF_PAGES                       (7)
#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)
/****************************************************************************
**                   LOCAL FUNCTION PROTOTYPES                                
****************************************************************************/
static void ActionEnetInit(void);
static void PeripheralsSetUp(void);
static void ClickAction(void);
static void ContextReset(void);
static void dummyIsr(void);
static void uartIsr(void);
static void MMUConfigAndEnable(void);
static void gpioIsr(void);
static void ActionMenu(void);
static void ActionMenuWeb(void);
static void ActionMenuMMCSD(void);
static void ActionMenuRTC(void);
static void ActionMenuTimer(void);
static void ActionMenuGPIO(void);
static void ActionMenuPM(void);
static void ActionMMCSD(void);
static void ActionRTC(void);
static void ActionDeepSleep0(void);
static void ActionDeepSleep1(void);
static void ActionStandBy(void);
static void ActionWakeOnTimer(void);
static void ActionWakeOnUart(void);

/****************************************************************************
**                   EXTERNAL VARIABLE DECLARATIONS                             
****************************************************************************/
extern unsigned int ipAddr;
extern volatile tBoolean bConnected;
extern unsigned int iram_start;
extern unsigned int iram_size;
extern unsigned int relocstart;
extern unsigned int relocend;
extern unsigned int printtemp;
extern volatile unsigned int wakeSource;
extern deepSleepData ds0Data;
extern deepSleepData ds1Data;
extern deepSleepData standbyData;

/*******************************************************************************
**                     EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void etharp_tmr(void);
extern void romRestoreLocation(void);
void clearTimerInt(void);

/****************************************************************************
**                  GLOBAL VARIABLES DEFINITIONS                                         
****************************************************************************/
unsigned int clickIdx = 0;

/*
** The variable which indicates version of SoC
*/
unsigned int deviceVersion = 0;

/*
** The variable which indicates whether an action is called as a result of prev
** button click.
*/
unsigned int prevAction = 0;

/*
** The variable which is used to keep track of the page.
*/
volatile unsigned int pageIndex = 0;

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

static tPageNavSpec const pageNavSpecMenu[NUM_ICON_MENU] =
{
    {ActionMenuWeb, "WWW"},
    {ActionMenuRTC, "RTC demo"},
    {ActionMenuGPIO, "GPIO demo"},
    {ActionMenuTimer, "Timer demo"},
    {ActionMenuMMCSD, "MMCSD info"},
    {ActionMenuPM, "PM demo"},
};

static tPageNavSpec const pageNavSpecWeb[NUM_ICON_WWW] =
{
    {ActionMenuRTC, "Next slide"},
    {ActionMenuPM, "Prev slide"},
    {ActionMenu, "Home slide"},
    {ActionEnetInit, "Execute demo"}
};

static tPageNavSpec const pageNavSpecRtc[NUM_ICON_RTC] =
{
    {ActionMenuGPIO, "Next slide"},
    {ActionMenuWeb, "Prev slide"},
    {ActionMenu, "Home slide"},
    {ActionRTC, "Execute demo"}
};

static tPageNavSpec const pageNavSpecGPIO[NUM_ICON_GPIO] =
{
    {ActionMenuTimer, "Next slide"},
    {ActionMenuRTC, "Prev slide"},
    {ActionMenu, "Home slide"},
};

static tPageNavSpec const pageNavSpecTimer[NUM_ICON_TMR] =
{
    {ActionMenuMMCSD, "Next slide"},
    {ActionMenuGPIO, "Prev slide"},
    {ActionMenu, "Home slide"}
};

static tPageNavSpec const pageNavSpecMMCSD[NUM_ICON_MMCSD] =
{
    {ActionMenuPM, "Next slide"},
    {ActionMenuTimer, "Prev slide"},
    {ActionMenu, "Home slide"},
    {ActionMMCSD, "Execute demo"}
};

static tPageNavSpec const pageNavSpecPM[NUM_ICON_PM] =
{
    {ActionMenuWeb, "Next slide"},
    {ActionMenuMMCSD, "Prev slide"},
    {ActionMenu, "Home slide"},
    {ActionDeepSleep0, "DS0 power mode"},
    {ActionDeepSleep1, "DS1 power mode"},
    {ActionStandBy, "Standby mode"},
    {ActionWakeOnTimer, "Timer wake source"},
    {ActionWakeOnUart, "Uart wake source"},
};

/*
** Context information.
** Number of icons in the Page, specification.
*/
tPageContext contextInfo[NUM_OF_PAGES] =
{
    {NUM_ICON_MENU, pageNavSpecMenu},
    {NUM_ICON_WWW, pageNavSpecWeb},
    {NUM_ICON_RTC, pageNavSpecRtc},
    {NUM_ICON_GPIO, pageNavSpecGPIO},
    {NUM_ICON_TMR, pageNavSpecTimer},
    {NUM_ICON_MMCSD, pageNavSpecMMCSD},
    {NUM_ICON_PM, pageNavSpecPM},
};

/****************************************************************************
**                      FUNCTION DEFINITIONS                                         
****************************************************************************/
/*
** Function to setup MMU. This function Maps three regions (1. DDR
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

/*
** Print on Uart console the available configuration to navigate.
*/
void UpdateUartConsoleHelp(void)
{
    unsigned int idx = 0;
    unsigned int numIcon = contextInfo[pageIndex].numIcon;
    tPageNavSpec const *pageNavSpec;

    ConsoleUtilsPrintf("\n\r\n\r Select/Execute/Goto: ");

    for(idx = 0; idx < numIcon; idx++)
    {
        pageNavSpec = contextInfo[pageIndex].pageNavSpec + idx;

        ConsoleUtilsPrintf("\n\r  %d. %s", idx, pageNavSpec->helpStr);
    }

    ConsoleUtilsPrintf("\n\r Selected:  ");
}

/*
** Take the actions on the uart.
*/
static void UartAction(unsigned int actionInput)
{
    unsigned int numIcon = contextInfo[pageIndex].numIcon;
    tPageNavSpec const *pageNavSpec;

    if(actionInput < numIcon)
    {
        pageNavSpec = contextInfo[pageIndex].pageNavSpec + actionInput;

        if((1 == actionInput) && (0 != pageIndex))
        {
            prevAction = 1;
        }

        (pageNavSpec->action)();
    }
    else
    {
        ConsoleUtilsPrintf("Invalid Selection!!\r\n");
        ConsoleUtilsPrintf("\n\r Selected:  ");
        return;
    }
}

/*
** Enable all the peripherals in use
*/
static void PeripheralsSetUp(void)
{
    enableModuleClock(CLK_UART0);
    enableModuleClock(CLK_I2C0);
    /* Timer6 is used for Standby wakeup */
    enableModuleClock(CLK_TIMER6);
    GPIO0ModuleClkConfig();
    DMTimer2ModuleClkConfig();
    DMTimer3ModuleClkConfig();
    DMTimer4ModuleClkConfig();
    RTCModuleClkConfig();
    CPSWPinMuxSetup();
    CPSWClkEnable();
    EDMAModuleClkConfig();
    GPIO1ModuleClkConfig();
    GPIO1Pin23PinMuxSetup();
    HSMMCSDPinMuxSetup();
    HSMMCSDModuleClkConfig();
    I2CPinMuxSetup(0);
}

/*
** Resets the state
*/
static void ContextReset(void)
{
    tmrFlag  = FALSE;
    LedOff();
    rtcSetFlag = FALSE;
    rtcSecUpdate = FALSE;
    sdCardAccessFlag = FALSE;
}

/*
** Take the actions on click.
*/
static void ClickAction(void)
{
    tPageNavSpec const *clickSpec;

    pageIndex = clickIdx;

    /*
    ** Get the spec. Assumed that the last touch spec only will give
    ** action for the next image.
    */
    clickSpec = contextInfo[pageIndex - 1].pageNavSpec;

    (clickSpec->action)();
}

/*
** Main function. The application starts here.
*/
int main(void)
{
    unsigned char rxByte;
    unsigned int value = (unsigned int)E_FAIL;

    #ifdef __TMS470__
    /* Relocate the required section to internal RAM */
    memcpy((void *)(&relocstart), (const void *)(&iram_start),
           (unsigned int)(&iram_size));
    #elif defined(__IAR_SYSTEMS_ICC__)
    #pragma section = "CodeRelocOverlay"
    #pragma section = "DataRelocOverlay"
    #pragma section = "DataOverlayBlk"
    #pragma section = "CodeOverlayBlk"
    char* srcAddr = (__section_begin("CodeRelocOverlay"));
    char* endAddr = (__section_end("DataRelocOverlay"));

    memcpy((void *)(__section_begin("CodeRelocOverlay")),
           (const void *)(__section_begin("CodeOverlayBlk")),
           endAddr - srcAddr);

    #else
    memcpy((void *)&(relocstart), (const void *)&(iram_start),
           (unsigned int)(((&(relocend)) -
            (&(relocstart))) * (sizeof(unsigned int))));
    #endif

    MMUConfigAndEnable();    

    /* Enable Instruction Cache */
    CacheEnable(CACHE_ALL);

    PeripheralsSetUp();

    /* Initialize the ARM Interrupt Controller */
    IntAINTCInit();

    /* Register the ISRs */  
    Timer2IntRegister();
    Timer4IntRegister();
    EnetIntRegister();
    RtcIntRegister();
    CM3IntRegister();
    HSMMCSDIntRegister();
    IntRegister(127, dummyIsr);

    IntMasterIRQEnable();

    pageIndex = 0;
    prevAction = 0;

    /* Enable system interrupts */
    IntSystemEnable(SYS_INT_RTCINT);
    IntPrioritySet(SYS_INT_RTCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT2);
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT4);
    IntPrioritySet(SYS_INT_TINT4, 0, AINTC_HOSTINT_ROUTE_IRQ);	
    IntSystemEnable(SYS_INT_MMCSD0INT);
    IntPrioritySet(SYS_INT_MMCSD0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_EDMACOMPINT);
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_M3_TXEV, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntSystemEnable(SYS_INT_M3_TXEV);
    IntSystemEnable(127);
    IntPrioritySet(127, 0, AINTC_HOSTINT_ROUTE_IRQ);

    IntSystemEnable(SYS_INT_UART0INT);
    IntPrioritySet(SYS_INT_UART0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntRegister(SYS_INT_UART0INT, uartIsr);

     /*	GPIO interrupts	*/
    IntSystemEnable(SYS_INT_GPIOINT0A);
    IntPrioritySet(SYS_INT_GPIOINT0A, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntRegister(SYS_INT_GPIOINT0A, gpioIsr);
    IntSystemEnable(SYS_INT_GPIOINT0B);
    IntPrioritySet(SYS_INT_GPIOINT0B, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntRegister(SYS_INT_GPIOINT0B, gpioIsr);

    BoardInfoInit();
    deviceVersion = DeviceVersionGet();

    CM3EventsClear();
    CM3LoadAndRun();
    waitForM3Txevent();

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /*
    ** Select the console type based on compile time check
    ** Note: This example is not fully complaint to semihosting. It is
    **       recommended to use Uart console interface only.
    */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Print Board and SoC information on console */
    ConsoleUtilsPrintf("\n\r Board Name          : %s", BoardNameGet());
    ConsoleUtilsPrintf("\n\r Board Version       : %s", BoardVersionGet());
    ConsoleUtilsPrintf("\n\r SoC Version         : %d", deviceVersion);

    /* On CM3 init firmware version is loaded onto the IPC Message Reg */
    ConsoleUtilsPrintf("\n CM3 Firmware Version: %d", readCM3FWVersion());

    I2CIntRegister(I2C_0);
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_I2C0INT);
    I2CInit(I2C_0);

    IntSystemEnable(SYS_INT_TINT1_1MS);
    IntPrioritySet(SYS_INT_TINT1_1MS, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntRegister(SYS_INT_TINT1_1MS,clearTimerInt);

    configVddOpVoltage();
    RtcInit();
    HSMMCSDContolInit();
    DelayTimerSetup();

    initializeTimer1();
    ConsoleUtilsPrintf("\r\n After intializing timer");
    Timer2Config();
    Timer4Config();
    LedIfConfig();
    MailBoxInit();

    Timer2IntEnable();
    Timer4IntEnable();
    RtcSecIntEnable();
    	
    Timer4Start(); 
    while(FALSE == tmr4Flag);
    tmr4Flag = FALSE;
    Timer4Stop();

    ConsoleUtilsPrintf("\n\r Configuring for maximum OPP");
    mpuOpp = ConfigMaximumOPP();

    mpuFreq = FrequencyGet(mpuOpp);
    mpuVdd1 = VddVoltageGet(mpuOpp);
    PrintConfigDVFS();

    /*  Create menu page */
    pageIndex = MENU_IDX_MAIN;

    ActionEnetInit();

    /*
    ** Loop for ever. Necessary actions shall be taken
    ** after detecting the click.
    */
    while(1)
    {
        /*
        ** Check for any any activity on Uart Console and process it.
        */
        if (true == UARTCharsAvail(SOC_UART_0_REGS))
        {

            /* Receiving bytes from the host machine through serial console. */
            rxByte = UARTGetc();

            /*
            ** Checking if the entered character is a carriage return.
            ** Pressing the 'Enter' key on the keyboard executes a
            ** carriage return on the serial console.
            */
            if('\r' == rxByte)
            {
                ConsoleUtilsPrintf("\n");
                UartAction(value);
                value = (unsigned int)E_FAIL;
                rxByte = 0;
            }

            /*
            ** Checking if the character entered is one among the decimal
            ** number set 0,1,2,3,....9
            */
            if(('0' <= rxByte) && (rxByte <= '9'))
            {
                ConsoleUtilsPrintf("%c", rxByte);

                if((unsigned int)E_FAIL == value)
                {
                    value = 0;
                }

                value = value*10 + (rxByte - 0x30);
            }

        }

         /*
         ** Check if click is detected
         */
         if(clickIdx != 0)
         {
             /*
             ** Take the Action for click
             */
             ClickAction();

             clickIdx = 0;
         }
       
         /*
         ** Check if the Timer Expired
         */ 
         if(TRUE == tmrFlag)
         {
             /* Toggle the LED state */
             LedToggle();
             tmrFlag = FALSE;
         }
 
         /*
         ** Check if RTC Time is set
         */
         if(TRUE == rtcSetFlag)
         {
             if(TRUE == rtcSecUpdate)
             { 
                 rtcSecUpdate = FALSE;
                 RtcTimeCalDisplay();
                 ConsoleUtilsPrintf(" --- Selected:  ");
             }
         } 
   
         if(TRUE == tmr4Flag)
         {
            tmr4Flag = FALSE;
             /* Make sure that interrupts are disabled and no lwIP functions
                are executed while calling an lwIP exported API */
             IntMasterIRQDisable();
             etharp_tmr();
             IntMasterIRQEnable();
         }
    }
}

/*
** Action to be taken when the demo is to be driven via Ethernet
*/
static void ActionEnetInit(void)
{
    unsigned int linkFlag = FALSE;

    if(!EnetIfIsUp())
    {
        ContextReset();
        linkFlag = FALSE;
        EnetHttpServerInit();

        if(ipAddr)
        {
            linkFlag = TRUE;
        }
    }
    else
    {
        if(EnetLinkIsUp())
        {
            linkFlag = TRUE;
        }
        else
        {
            ContextReset();
            linkFlag = FALSE;
        }
    }

    if((TRUE == linkFlag) && (ipAddr != 0))
    {
        ConsoleUtilsPrintf("\n\rAccess the home page using http://");
        IpAddrDisplay();
        ConsoleUtilsPrintf("/index.html \n\r");
    }
    else
    {
        ConsoleUtilsPrintf("\n\rNetwork Connection failed.\n\r");
    }

    UpdateUartConsoleHelp();
}

/*
** Action for Menu
*/
static void ActionMenu(void)
{
    pmFlag = FALSE;
    sdCardAccessFlag = FALSE;
    tmrFlag  = FALSE;
    tmrClick = FALSE;
    rtcSetFlag = FALSE;
    Timer2Stop();
    LedOff();
    pageIndex = MENU_IDX_MAIN;
    UpdateUartConsoleHelp();
}

/*
** Action for Web Menu
*/
static void ActionMenuWeb(void)
{
    pmFlag = FALSE;
    sdCardAccessFlag = FALSE;
    tmrFlag  = FALSE;
    tmrClick = FALSE;
    rtcSetFlag = FALSE;
    Timer2Stop();
    LedOff();
    pageIndex = MENU_IDX_WWW;
    UpdateUartConsoleHelp();
}

/*
** Action for menu MMCSD icon click
*/
static void ActionMenuMMCSD(void)
{
    pmFlag = FALSE;
    rtcSetFlag = FALSE;
    tmrClick = FALSE;
    tmrFlag  = FALSE;
    LedOff();
    pageIndex = MENU_IDX_SD;
    UpdateUartConsoleHelp();
}

/*
** Action for menu RTC icon click
*/
static void ActionMenuRTC(void)
{
    pmFlag = FALSE;
    sdCardAccessFlag = FALSE;
    tmrFlag  = FALSE;
    tmrClick = FALSE;
    LedOff();
    pageIndex = MENU_IDX_RTC;
    UpdateUartConsoleHelp();
}

/*
** Action for menu timer icon click
*/
static void ActionMenuTimer(void)
{
    pmFlag = FALSE;
    sdCardAccessFlag = FALSE;
    rtcSetFlag = FALSE;
    tmrClick = TRUE;
    tmrFlag  = FALSE;
    Timer2Start();
    pageIndex = MENU_IDX_TIMER;
    UpdateUartConsoleHelp();
}

/*
** Action for menu GPIO icon click
*/
static void ActionMenuGPIO(void)
{
    pmFlag = FALSE;
    sdCardAccessFlag = FALSE;
    Timer2Stop();
    LedOn();
    tmrClick = FALSE;
    rtcSetFlag = FALSE;
    pageIndex = MENU_IDX_LED;
    UpdateUartConsoleHelp();
}

/*
** Action for menu PM icon click
*/
static void ActionMenuPM(void)
{
    pmFlag = TRUE;
    rtcSetFlag = FALSE;
    tmrClick = FALSE;
    tmrFlag  = FALSE;
    LedOff();
    Timer2Stop();
    pageIndex = MENU_IDX_PM;
    UpdateUartConsoleHelp();
}

/*
** Set RTC Timer
*/
static void ActionRTC(void)
{
    RtcTimeCalSet();
    pageIndex = MENU_IDX_RTC;
    UpdateUartConsoleHelp();
}

/*
** Perform file operations on MMCSD
*/
static void ActionMMCSD(void)
{
    pmFlag = FALSE;
    rtcSetFlag = FALSE;
    tmrClick = FALSE;
    tmrFlag  = FALSE;
    LedOff();

    if(TRUE == HSMMCSDCardPresentStat())
    {
        sdCardAccessFlag = TRUE;
    }
    else
    {
        sdCardAccessFlag = FALSE;
        ConsoleUtilsPrintf("\n\rSD card not present. Please insert an"
                           " SD card and try again! \n\r");
    }

    /*
    ** Check for SD Card
    */
    if(TRUE == sdCardAccessFlag)
    {
         HSMMCSDCardAccessSetup();
    }

    pageIndex = MENU_IDX_SD;
    UpdateUartConsoleHelp();
}

/*
** Steps to be taken when DS1 is selected
*/
void ActionDeepSleep1(void)
{
    ConsoleUtilsPrintf("\n\rEntering DS1 Power Saving Mode...");
    PowerSaveModeEnter(ds1Data, SLEEP_MODE_DS1);

    /* update the console */
    UpdateUartConsoleHelp();
}

/*
** Steps to be taken when Standby mode is selected
*/
void ActionStandBy(void)
{
    ConsoleUtilsPrintf("\n\rEntering Standby Power Saving Mode...");

    /* Enter PM Standby mode. */
    PowerSaveModeEnter(standbyData, SLEEP_STAND_BY_MODE);

    /* update the console */
    UpdateUartConsoleHelp();
}

/*
** Steps to be taken when DS0 is selected
*/
void ActionDeepSleep0(void)
{
    ConsoleUtilsPrintf("\n\rEntering DS0 Power Saving Mode...");
    PowerSaveModeEnter(ds0Data, SLEEP_MODE_DS0);

    /* update the console */
    UpdateUartConsoleHelp();
}

/*
** Select wake source
*/
void ActionWakeOnTimer()
{
    /* update wake source */
    wakeSource = WAKE_SOURCE_TMR;
    ConsoleUtilsPrintf("\n\n Timer wake selected \r\n");

    /* update console */
    UpdateUartConsoleHelp();
}

/*
** Select wake source
*/
void ActionWakeOnUart()
{
    /* update wake source */
    wakeSource = WAKE_SOURCE_UART;
    ConsoleUtilsPrintf("\n\n Uart wake selected \r\n");

    /* update the console */
    UpdateUartConsoleHelp();
}

/*
** Dummy ISR to handle spurious interrupts
*/
static void dummyIsr(void)
{
    ; /* Perform nothing */
}

/*
** Uart ISR to read the inputs
*/
static void uartIsr(void)
{
	volatile unsigned char rxByte;
    ; /* Perform nothing */
	rxByte = UARTCharGetNonBlocking(SOC_UART_0_REGS);
	UARTCharPutNonBlocking(SOC_UART_0_REGS, rxByte);
}

/*
** Uart ISR to read the inputs
*/
static void gpioIsr(void)
{
	/*	Clear wake interrupt	*/
	HWREG(SOC_GPIO_0_REGS + 0x2C) = 0x4;
	HWREG(SOC_GPIO_0_REGS + 0x30) = 0x4;
}
/****************************** End of file *********************************/
  
