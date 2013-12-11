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
 *                  I2C0-1
 *                  Interrupt
 *                  LCD Controller
 *                  McASP0
 *                  Mailbox0
 *                  Power Management
 *                  PRCM
 *                  PWMSS0
 *                  RTC0
 *                  Timer1-7
 *                  Touchscreen
 *                  UART0
 *
 *              Configurable Parameters:
 *                  Power Management - Sleep Mode: DS0
 *                                                 DS1
 *                                                 Standby
 *                                   - Wake Source: UART0
 *                                                  Timer1
 *                                                  Timer6
 *                                                  RTC Alarm
 *                                                  GPIO0
 *                                                  Touchscreen
 *                  RTC - Set Timer
 *
 *              Hard-coded configuration of other parameters:
 *                  IP Address - DHCP is default configuration. For manual
 *                               configuration use STATIC_IP_ADDRESS_PORT1
 *                               or STATIC_IP_ADDRESS_PORT2 in lwipopts.h.
 *                  Console interface - Uart interface is default configuration.
 *                                      For semihosting set ConsoleUtilsSetType
 *                                      to CONSOLE_DEBUGGER and set compile time
 *                                      macro CONSOLE as SEMIHOSTING.
 *
 *          Application Usecase:
 *              Cortex-M3 - Perform suspend/resume of MPU
 *              Ethernet - Host web page for selection of use case
 *              GPIO0 - Led blink
 *              I2C0 - PMIC control
 *              I2C1 - Temperature sensor
 *              LCD Controller - Porting of grlib to display menu
 *              McASP0 - Audio playback
 *              Power Management - Demonstrate execution of sleep modes
 *              PRCM - Configure the operating voltage and frequency of MPU
 *              PWMSS0 - Backlight control through eCAP
 *              RTC0 - Set Timer for seconds tick
 *              Timer2 - Overflow counter
 *              Touchscreen - Menu navigation
 *              UART0 - Console operation
 *
 *          Running example:
 *              1. Load example and run. Use touchscreen to navigate and execute
 *                 use cases from matrix menu.
 *              2. Connect board to network and use Web menu to configure the IP
 *                 address. Use web page to navigate use cases.
 *              3. UART console provides interface to configure the parameters
 *                 listed under configurable parameters.
 *              4. Select RTC page to set time and date through console
 *              5. Select Timer page to configure timer switch between blue and
 *                 red color on verflow overflow.
 *              6. Select eCAP page to gradually dim and resume brightness of
 *                 backlight.
 *              7. Select GPIO page to blink led.
 *              8. Select I2C page to show temperature.
 *              9. Select PM page to configure the wake source and execute sleep
 *                 modes.
 *             10. Select DVFS page to configure MPU for different OPP setting.
 *
 *          Limitations:
 *              1. RTC alarm wake source is not supported for SoC Version 1.0.
 *              2. Nitro mode is not supported for SoC Version 1.0.
 *
 *          Note:
 *              1. OOB is not fully complaint to semihosting.  To navigate OOB
 *                 through console interface non-blocking uart interface is used.
 *                 The same is not possible with semihosting console interface.
 *                 If semihosting is enabled for this example, commandline instr
 *                 has to be provided through uart but the same is displayed on
 *                 debugger console interface. It is recommended to use Uart
 *                 console interface only.
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

#include "interrupt.h"
#include "demoRaster.h"
#include "demoTouch.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "demoToneLoop.h"
#include "demoCfg.h"
#include "demoAic31.h"
#include "demoTimer.h"
#include "demoEnet.h"
#include "demoRtc.h"
#include "evmskAM335x.h"
#include "consoleUtils.h"
#include "uartStdio.h"
#include "demoEcap.h"
#include "demoGpio.h"
#include "error.h"
#include "cache.h"
#include "mmu.h"
#include "board.h"
#include "device.h"
#include "demoPwrMgmnt.h"
#include "demoI2c.h"
#include "demoGrlib.h"
#include "grlib.h"
#include "bannerImage.h"
#include "baseImage.h"
#include "delay.h"
#include "clock.h"
#include "uart_irda_cir.h"
#include "demoDvfs.h"
#include "rtc.h"

#include "pin_mux.h"
#include "cm3wkup_proxy.h"
#include <string.h>

/****************************************************************************
**                   INTERNAL MACRO DEFINITIONS                                       
****************************************************************************/
#define NUM_OF_IMAGES                         (15u)

#define START_ADDR_DDR             (0x80000000)
#define START_ADDR_DEV             (0x44000000)
#define START_ADDR_OCMC            (0x40300000)
#define NUM_SECTIONS_DDR           (512)
#define NUM_SECTIONS_DEV           (960)
#define NUM_SECTIONS_OCMC          (1)
 
/****************************************************************************
**                   LOCAL FUNCTION PROTOTYPES                                
****************************************************************************/
static void CoOrdAction(int x, int y);
static void PeripheralsSetUp(void);
static void ClickAction(void);
static void PrevContextClear(void);
static void ActionIdle(void);
static void ActionTimeSet(void);
static void ActionEnetInit(void);
static void ActionECAPdemo(void);
static void MMUConfigAndEnable(void);
static void dummyIsr(void);
static void uartIsr(void);
static void gpioIsr(void);
static void ActionMenu(void);
static void ActionMenuIntro(void);
static void ActionMenuWebDemo(void);
static void ActionMenuMcASP(void);
static void ActionMenuMMCSD(void);
static void ActionMenuUart(void);
static void ActionMenuRTC(void);
static void ActionMenuEthernet(void);
static void ActionMenuTimer(void);
static void ActionMenuECAP(void);
static void ActionMenuGPIO(void); 
static void ActionMenuI2C(void);
static void ActionMenuPM(void);
static void toggleColors(void);
static void ActionMenuDVFS(void);
static void ActionDeepSleep0(void);
static void ActionDeepSleep1(void);
static void ActionStandBy(void);
static void ActionWakeOnTsc(void);
static void ActionWakeOnTimer(void);
static void ActionWakeOnUart(void);
static void ActionWakeOnGpio(void);
static void ActionWakeOnRTC(void);

/*******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned char g_pucBuffer[2][GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
extern unsigned char baseUnCompImage[];

extern tContext sContext[];

extern unsigned int ipAddr;
extern volatile tBoolean bConnected;
extern unsigned int iram_start;
extern unsigned int iram_size;
extern unsigned int relocstart;
extern unsigned int relocend;
extern unsigned int ipAddr1;
extern unsigned int ipAddr2;
extern volatile unsigned int wakeSource;
extern deepSleepData ds0Data;
extern deepSleepData ds1Data;
extern deepSleepData standbyData;

/*******************************************************************************
**                     EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void etharp_tmr(void);
extern void romRestoreLocation(void);

/****************************************************************************
**                  GLOBAL VARIABLES DEFINITIONS                                         
****************************************************************************/
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/*
** Coordinates for each icon.
*/
int const xyDefault[4] =  {0, 0, 0, 0};

int const xyNext[4] = {XMIN_NEXT, XMAX_NEXT, YMIN_NEXT, YMAX_NEXT};

int const xyPrev[4] = {XMIN_PREV, XMAX_PREV, YMIN_PREV, YMAX_PREV};    
         
int const xyHome[4] = {XMIN_HOME, XMAX_HOME, YMIN_HOME, YMAX_HOME};

int const xyTimeSet[4] = {XMIN_RTC_STD, XMAX_RTC_STD,
                          YMIN_RTC_STD, YMAX_RTC_STD};
                          
int const xyIntro[4] =  { XMIN_INTRO, XMAX_INTRO, YMIN_INTRO, YMAX_INTRO};

int const xyWebDemo[4] = { XMIN_WEB_DEMO, XMAX_WEB_DEMO,
                           YMIN_WEB_DEMO, YMAX_WEB_DEMO};
                           
int const xyMcASP[4] = { XMIN_MCASP, XMAX_MCASP, YMIN_MCASP, YMAX_MCASP};

int const xyUart[4] =  { XMIN_UART, XMAX_UART, YMIN_UART, YMAX_UART}; 

int const xyRTC[4] = { XMIN_RTC, XMAX_RTC, YMIN_RTC, YMAX_RTC}; 

int const xyTimer[4] = { XMIN_TIMER, XMAX_TIMER, YMIN_TIMER, YMAX_TIMER};

int const xyEthernet[4] = { XMIN_ETHERNET, XMAX_ETHERNET, 
                            YMIN_ETHERNET, YMAX_ETHERNET }; 

int const xyMMCSD[4] = { XMIN_MMCSD, XMAX_MMCSD, 
                                   YMIN_MMCSD, YMAX_MMCSD }; 
								   
int const xyEcapDemo[4] = {XMIN_ECAP, XMAX_ECAP,
                       YMIN_ECAP, YMAX_ECAP};								   

int const xyEcapMenu[4] = {XMIN_ECAP_MENU, XMAX_ECAP_MENU,
						   YMIN_ECAP_MENU, YMAX_ECAP_MENU};	
						   
int const xyGpioMenu[4] = {XMIN_GPIO_MENU, XMAX_GPIO_MENU,
						   YMIN_GPIO_MENU, YMAX_GPIO_MENU};	
						   
int const xyI2CMenu[4] = {XMIN_I2C_MENU, XMAX_I2C_MENU,
						  YMIN_I2C_MENU, YMAX_MOUSE_LEFT};	

int const xyPMMenu[4] = {XMIN_PM_MENU, XMAX_PM_MENU,
						 YMIN_PM_MENU, YMAX_PM_MENU};	

int const xyPMstandbyDemo[4] = {XMIN_PM_STANDBY_DEMO, XMAX_PM_STANDBY_DEMO,
                           YMIN_PM_STANDBY_DEMO, YMAX_PM_STANDBY_DEMO};

int const xyPMds0Demo[4] = {XMIN_PM_DS0_DEMO, XMAX_PM_DS0_DEMO,
                           YMIN_PM_DS0_DEMO, YMAX_PM_DS0_DEMO};								 

int const xyPMds1Demo[4] = {XMIN_PM_DS1_DEMO, XMAX_PM_DS1_DEMO,
                           YMIN_PM_DS1_DEMO, YMAX_PM_DS1_DEMO};							   

int const xyPMwksTsc[4] = {XMIN_PM_WAKE_TSC, XMAX_PM_WAKE_TSC,
                           YMIN_PM_WAKE_TSC, YMAX_PM_WAKE_TSC};							   						   						   
						   
int const xyPMwksTmr[4] = {XMIN_PM_WAKE_TMR, XMAX_PM_WAKE_TMR,
                           YMIN_PM_WAKE_TMR, YMAX_PM_WAKE_TMR};							   
						   
int const xyPMwksUart[4] = {XMIN_PM_WAKE_UART, XMAX_PM_WAKE_UART,
                           YMIN_PM_WAKE_UART, YMAX_PM_WAKE_UART};		

int const xyPMwksGpio[4] = {XMIN_PM_WAKE_GPIO, XMAX_PM_WAKE_GPIO,
                           YMIN_PM_WAKE_GPIO, YMAX_PM_WAKE_GPIO};						   

int const xyPMwksRTC[4] = {XMIN_PM_WAKE_RTC, XMAX_PM_WAKE_RTC,
                           YMIN_PM_WAKE_RTC, YMAX_PM_WAKE_RTC};

int const xyDVFSMenu[4] = {XMIN_DVFS_MENU, XMAX_DVFS_MENU,
                           YMIN_DVFS_MENU, YMAX_DVFS_MENU};


int const xyDVFSOpp50[4] = {XMIN_DVFS_OPP50, XMAX_DVFS_OPP50,
                            YMIN_DVFS_OPP50, YMAX_DVFS_OPP50};
int const xyDVFSOpp100[4] = {XMIN_DVFS_OPP100, XMAX_DVFS_OPP100,
                            YMIN_DVFS_OPP100, YMAX_DVFS_OPP100};
int const xyDVFSOpp120[4] = {XMIN_DVFS_OPP120, XMAX_DVFS_OPP120,
                            YMIN_DVFS_OPP120, YMAX_DVFS_OPP120};
int const xyDVFSSrTurbo[4] = {XMIN_DVFS_SRTURBO, XMAX_DVFS_SRTURBO,
                            YMIN_DVFS_SRTURBO, YMAX_DVFS_SRTURBO};
int const xyDVFSNitro[4] = {XMIN_DVFS_NITRO, XMAX_DVFS_NITRO, YMIN_DVFS_NITRO,
                            YMAX_DVFS_NITRO};

/*
** Specifications for each icon
*/
static TOUCHSPEC const touchSpecBanner[NUM_ICON_BANNER] =
                    {
                       {xyDefault, ActionIdle}
                    };

static TOUCHSPEC const touchSpecMenu[NUM_ICON_MENU] =
                    {
                       {xyIntro, ActionMenuIntro, "Introduction"},
                       {xyWebDemo, ActionMenuWebDemo, "WWW"},
                       {xyMcASP, ActionMenuMcASP, "Audio demo"},
                       {xyMMCSD, ActionMenuMMCSD, "MMCSD info"},
                       {xyUart, ActionMenuUart, "Uart info"},
                       {xyRTC, ActionMenuRTC, "RTC demo"},
                       {xyTimer, ActionMenuTimer, "Timer demo"},
                       {xyEthernet, ActionMenuEthernet, "Ethernet info"},
                       {xyEcapMenu, ActionMenuECAP, "eCAP demo"},
                       {xyGpioMenu, ActionMenuGPIO, "GPIO demo"},
                       {xyI2CMenu, ActionMenuI2C, "I2C demo"},
                       {xyPMMenu, ActionMenuPM, "PM demo"},
                       {xyDVFSMenu, ActionMenuDVFS, "DVFS demo"},
                    };
                    
static TOUCHSPEC const touchSpecIntro[NUM_ICON_INTRO] = 
                    {
                       {xyNext, ActionMenuWebDemo, "Next slide"},
                       {xyPrev, ActionMenu, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };

static TOUCHSPEC const touchSpecUart[NUM_ICON_UART] = 
                    {
                       {xyNext, ActionMenuRTC, "Next slide"},
                       {xyPrev, ActionMenuMMCSD, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };
					
static TOUCHSPEC const touchSpecRtc[NUM_ICON_RTC] = 
                    {
                       {xyNext, ActionMenuTimer, "Next slide"},
                       {xyPrev, ActionMenuUart, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                       {xyTimeSet, ActionTimeSet, "Execute demo"}
                    };

static TOUCHSPEC const touchSpecMmcsd[NUM_ICON_MMCSD] =
                    {
                       {xyNext, ActionMenuUart, "Next slide"},
                       {xyPrev, ActionMenuMcASP, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };

static TOUCHSPEC const touchSpecWeb[NUM_ICON_CHOICE] =
                    {
                       {xyNext, ActionMenuMcASP, "Next slide"},
                       {xyPrev, ActionMenuIntro, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };

static TOUCHSPEC const touchSpecMcASP[NUM_ICON_MCASP] = 
                    {
                       {xyNext, ActionMenuMMCSD, "Next slide"},
                       {xyPrev, ActionMenuWebDemo, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };
                    
static TOUCHSPEC const touchSpecTimer[NUM_ICON_TMR] =
                    {
                       {xyNext, ActionMenuEthernet, "Next slide"},
                       {xyPrev, ActionMenuRTC, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };
static TOUCHSPEC const touchSpecEthernet[NUM_ICON_ENET] =
                    {
                       {xyNext, ActionMenuECAP, "Next slide"},
                       {xyPrev, ActionMenuTimer, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"}
                    };      

static TOUCHSPEC const touchSpecECAP[NUM_ICON_ECAP] =
                    {
                       {xyNext, ActionMenuGPIO, "Next slide"},
                       {xyPrev, ActionMenuEthernet, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                       {xyEcapDemo, ActionECAPdemo, "Execute demo"},
                    };  					
					
static TOUCHSPEC const touchSpecGPIO[NUM_ICON_GPIO] =
                    {
                       {xyNext, ActionMenuI2C, "Next slide"},
                       {xyPrev, ActionMenuECAP, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                    };  	

static TOUCHSPEC const touchSpecI2C[NUM_ICON_I2C] =
                    {
                       {xyNext, ActionMenuPM, "Next slide"},
                       {xyPrev, ActionMenuGPIO, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                    };  						
					
static TOUCHSPEC const touchSpecPM[NUM_ICON_PM] =
                    {
                       {xyNext, ActionMenuDVFS, "Next slide"},
                       {xyPrev, ActionMenuI2C, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                       {xyPMds0Demo, ActionDeepSleep0, "DS0 power mode"},
                       {xyPMds1Demo, ActionDeepSleep1, "DS1 power mode"},
                       {xyPMstandbyDemo, ActionStandBy, "Standby mode"},
                       {xyPMwksTsc, ActionWakeOnTsc, "Touchscreen wake source"},
                       {xyPMwksTmr, ActionWakeOnTimer, "Timer wake source"},
                       {xyPMwksUart, ActionWakeOnUart, "Uart wake source"},
                       {xyPMwksGpio, ActionWakeOnGpio, "GPIO wake source"},
                       {xyPMwksRTC, ActionWakeOnRTC, "RTC Alarm wake source"},
                    };  

static TOUCHSPEC const touchSpecDVFS[NUM_ICON_DVFS] =
                    {
                       {xyNext, ActionMenu, "Next slide"},
                       {xyPrev, ActionMenuPM, "Prev slide"},
                       {xyHome, ActionMenu, "Home slide"},
                       {xyDVFSOpp50, ActionDVFSOpp50, "OPP50"},
                       {xyDVFSOpp100, ActionDVFSOpp100, "OPP100"},
                       {xyDVFSOpp120, ActionDVFSOpp120, "OPP120"},
                       {xyDVFSSrTurbo, ActionDVFSSrTurbo, "SR TURBO"},
                       {xyDVFSNitro, ActionDVFSNitro, "Nitro"},
                    };
							
                    
/*
** Context information.
** Image, number of icons in the image, specification.
** The next icon will come as the last specification for each image.
*/
IMAGECONTEXT contextInfo[NUM_OF_IMAGES] =             
                    {
                       {bannerImage, NUM_ICON_BANNER, 	touchSpecBanner},
                       {bannerImage, NUM_ICON_MENU, 	touchSpecMenu},
                       {bannerImage, NUM_ICON_INTRO, 	touchSpecIntro},
                       {bannerImage, NUM_ICON_CHOICE, 	touchSpecWeb},
                       {bannerImage, NUM_ICON_MCASP, 	touchSpecMcASP},
	               {bannerImage, NUM_ICON_MMCSD, 	touchSpecMmcsd},
                       {bannerImage, NUM_ICON_UART, 	touchSpecUart},
                       {bannerImage, NUM_ICON_RTC, 		touchSpecRtc},
                       {bannerImage, NUM_ICON_TMR, 		touchSpecTimer},                       
                       {bannerImage, NUM_ICON_ENET, 	touchSpecEthernet},                       
                       {bannerImage, NUM_ICON_ECAP, 	touchSpecECAP},                       
                       {bannerImage, NUM_ICON_GPIO, 	touchSpecGPIO},                       
                       {bannerImage, NUM_ICON_I2C, 		touchSpecI2C},                       
                       {bannerImage, NUM_ICON_PM, 		touchSpecPM},                       
                       {bannerImage, NUM_ICON_DVFS, 		touchSpecDVFS},                       
                    };

/*
** The variable which is used to keep track of the current slide.
*/
volatile unsigned int imageCount = 0;

/*
** The variable which is used to keep track of the current Frame buffer.
*/
volatile unsigned int frameBufIdx = 0;

/*
** The variable which indicates which peripheral demo to be shown.
*/
unsigned int clickIdx = 0;

/*
** The variable which indicates whether ethernet is initialized.
*/
unsigned int enetInitFlag = FALSE;

/*
** The variable which indicates whether I2C slide is active.
*/
unsigned int I2CDemoFlag = FALSE;

/*
** The variable which indicates version of SoC
*/
unsigned int deviceVersion = 0;

/****************************************************************************
**                      FUNCTION DEFINITIONS                                         
****************************************************************************/
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

/*
** Enable all the peripherals in use
*/
static void PeripheralsSetUp(void)
{
    enableModuleClock(CLK_UART0);
	enableModuleClock(CLK_LCDC);
	enableModuleClock(CLK_TIMER2);
	enableModuleClock(CLK_TIMER3);
	enableModuleClock(CLK_TIMER4);
    /* Timer6 is used for Standby wakeup */
    enableModuleClock(CLK_TIMER6);
	//enableModuleClock(CLK_TIMER7);
    enableModuleClock(CLK_I2C0);	
	enableModuleClock(CLK_I2C1);
	enableModuleClock(CLK_MCASP1);
	//enableModuleClock(CLK_GPIO1);
	//enableModuleClock(CLK_GPIO0);
	enableModuleClock(CLK_ADC_TSC);
	enableModuleClock(CLK_TPTC2);
	enableModuleClock(CLK_TPTC1);
	enableModuleClock(CLK_TPTC0);
	enableModuleClock(CLK_TPCC);
	
	GPIO1ModuleClkConfig();
	GPIO0ModuleClkConfig();
	
    EDMAModuleClkConfig();	

    RTCModuleClkConfig();
    CPSWClkEnable();	

    LCDPinMuxSetup();
    CPSWPinMuxSetup();
	I2CPinMuxSetup(0);
    I2CPinMuxSetup(1);
    McASP1PinMuxSetup();
    TSCADCPinMuxSetUp();
    ECAPPinMuxSetup(2);

    GPIO_PMUX_OFFADDR_VALUE(0, 30,PAD_FS_RXE_PU_PUPDE(7));
	
    GPIO1Pin4PinMuxSetup();
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
	HWREG(SOC_GPIO_0_REGS + 0x2C) = 0x40000000;
	HWREG(SOC_GPIO_0_REGS + 0x30) = 0x40000000;
}

/*
** This function clears any previous context info
*/
static void PrevContextClear(void)
{
    /* Close the previous state */
    Timer2Stop();
    tmr2Flag  = FALSE;
    rtcSetFlag = FALSE;
	I2CDemoFlag = FALSE;
}

/*
** Print on Uart console the available configuration to navigate.
*/
void UpdateUartConsoleHelp(void)
{
    unsigned int idx = 0;
    unsigned int numIcon = contextInfo[imageCount].numIcon;
    TOUCHSPEC const *touchSpec;

    ConsoleUtilsPrintf("\n\r\n\r Select/Execute/Goto: ");

    for(idx = 0; idx < numIcon; idx++)
    {
        touchSpec = contextInfo[imageCount].touchSpec + idx;

        ConsoleUtilsPrintf("\n\r  %d. %s", idx, touchSpec->helpStr);
    }

    ConsoleUtilsPrintf("\n\r Selected:  ");
}

/*
** Take the actions on the uart.
*/
static void UartAction(unsigned int actionInput)
{
    unsigned int numIcon = contextInfo[imageCount].numIcon;
    TOUCHSPEC const *touchSpec;

    if(actionInput < numIcon)
    {
        touchSpec = contextInfo[imageCount].touchSpec + actionInput;

        PrevContextClear();
        (touchSpec->action)();
        clickIdx = 0u;
    }
    else
    {
        ConsoleUtilsPrintf("Invalid Selection!!\r\n");
        ConsoleUtilsPrintf("\n\r Selected:  ");
    }
}

/*
** Take the actions on the touch.
** The coordinates are given by the parameters
*/
static void CoOrdAction(int x, int y)
{
    int const *coOrd;
    unsigned int cnt;
    unsigned int numIcon = contextInfo[imageCount].numIcon;
    TOUCHSPEC const *touchSpec;

    for(cnt = 0; cnt < numIcon; cnt++)
    {
        touchSpec = contextInfo[imageCount].touchSpec + cnt;
        coOrd = touchSpec->coOrd;

        /* Take action only for proper coordinates */
        if((x >= coOrd[0]) && (x <= coOrd[1]) &&
           ((y >= coOrd[2]) && (y <= coOrd[3])))
        {
            PrevContextClear();            
            (touchSpec->action)();
            break;
        }
    }    
}

/*
** Take the actions on click. 
*/
static void ClickAction(void)
{
    TOUCHSPEC const *clickSpec;

    /*
    ** Get the spec. Assumed that the last touch spec only will give 
    ** action for the next image.
    */
    clickSpec = contextInfo[clickIdx - 1].touchSpec;

    PrevContextClear();

    (clickSpec->action)();
}

/*
** Main function. The application starts here.
*/
int main(void)
{
    int x;
    int y;    
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

    /* Enable cache */
    CacheEnable(CACHE_ALL);

    PeripheralsSetUp();

    /* Initialize the ARM Interrupt Controller */
    IntAINTCInit();

    /* Register the ISRs */  
    Raster0IntRegister();
    Timer2IntRegister();
    Timer4IntRegister();
    EnetIntRegister();
    RtcIntRegister();
    TouchIntRegister();
    CM3IntRegister();
    IntRegister(127, dummyIsr); 
 
    IntMasterIRQEnable();

    /* Enable system interrupts */
    IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_LCDCINT);
    IntPrioritySet(SYS_INT_RTCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_RTCINT);
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT2);
    IntPrioritySet(SYS_INT_TINT3, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT3);
    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
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

    DelayTimerSetup();
    EcapInit();

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

    Raster0Init();

    I2CIntRegister(I2C_0);
    I2CIntRegister(I2C_1);
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntSystemEnable(SYS_INT_I2C0INT);
    IntSystemEnable(SYS_INT_I2C1INT);		
    I2CInit(I2C_0);
    I2CInit(I2C_1);
	
	IntSystemEnable(SYS_INT_TINT1_1MS);
    IntPrioritySet(SYS_INT_TINT1_1MS, 0, AINTC_HOSTINT_ROUTE_IRQ);
	IntRegister(SYS_INT_TINT1_1MS,clearTimerInt);	

    AudioCodecInit();
    ToneLoopInit();

    gpioLedInit();
	
    configVddOpVoltage();
    TouchInit();

    RtcInit();

	configWakeGpio();

	initializeTimer1();
    Timer2Config();
    Timer4Config();
    GrlibInit();
	initAccelerometer();
    EthernetInit();

    MailBoxInit();

    Raster0EOFIntEnable();
    Timer2IntEnable();
    Timer4IntEnable();
    RtcSecIntEnable();
    TouchIntEnable();


    imageCount = 0;
	
    frameBufIdx = 0;
	
    /* Extract banner image to Frame buffer */
    ImageArrExtract(bannerImage,  
                    (unsigned int*)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET));

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));

    Raster0Start();
    EcapBkLightEnable();

    Timer4Start();
    while(FALSE == tmr4Flag);
    tmr4Flag = FALSE;
    Timer4Stop();
	
    /* Extract base image to uncomp buffer */
    ImageArrExtract(baseImage, (unsigned int*)baseUnCompImage);
  
    /* Copy base image to FB */
    memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), 
           (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
	
    /* Start playing the tone */
    AudioTxActivate();

    mpuOpp = OppGet();

    if(!mpuOpp)
    {
        ConsoleUtilsPrintf("\n\r Not valid OPP. Configuring for maximum OPP");
        mpuOpp = ConfigMaximumOPP();
    }

    mpuFreq = FrequencyGet(mpuOpp);
    mpuVdd1 = VddVoltageGet(mpuOpp);
    PrintConfigDVFS();

    /*	Create menu page */
    imageCount = 1;
    updatePage(imageCount);
    UpdateUartConsoleHelp();

    TouchEnable();

    /*
    ** Loop for ever. Necessary actions shall be taken
    ** after detecting touch, based on the coordinates
    */
	TouchReleaseDetect();
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
         ** Check for touch and released on any icon
         */
         if (TRUE == TouchReleaseDetect())
         {
             value = 0;
             TouchCoOrdGet(&x, &y);

             /*
             ** Validate the coordinates and take action 
             */
             CoOrdAction(x, y);
         }

         /*
         ** Check if click is detected
         */
         if(clickIdx != 0)
         {
             value = 0;

             /*
             ** Take the Action for click
             */
             ClickAction();

             clickIdx = 0;
         }
        
         /*
         ** Check if the Timer Expired
         */ 
         if(TRUE == tmr2Flag)
         {
             tmr2Flag = FALSE;
             toggleColors();
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
		 
         /*
         ** Timer demo
         */
         if(TRUE == tmr4Flag)
	     {
             tmr4Flag = FALSE;
 
             /* Make sure that interrupts are disabled and no lwIP functions
                are executed while calling an lwIP exported API */
             IntMasterIRQDisable();
             etharp_tmr();
             IntMasterIRQEnable();
         }
		 
         /*
         ** I2C demo (Temperature Sensor, Accelerometer)
         */
		 if(TRUE == I2CDemoFlag)
	     {
			updateI2CDemo();
         }
    }
}

/*
** Action to be taken when the demo is to be driven via Ethernet
*/
static void ActionEnetInit(void)
{
    char ipMsg[60] = {"http://"};
    unsigned int i_index, i_msg = 7, ipByte = 0;
    unsigned int Port1linkFlag = FALSE;	
    unsigned int Port2linkFlag = FALSE;	
	
    GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm18b);
    GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
    GrStringDrawCentered(&sContext[frameBufIdx], "Checking Ethernet link and acquiring IP address...", -1, 228,
		        (42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    updatePage(imageCount);
   
    if(!EnetPort1IfIsUp())
    {
        Port1linkFlag = FALSE;
        EnetPort1HttpServerInit();

        if(ipAddr1)
        {
            Port1linkFlag = TRUE;
        }
    }
    else
    {
        if(EnetPort1LinkIsUp())
        {
            Port1linkFlag = TRUE;
        }
   
        else
        {
            Port1linkFlag = FALSE;
        }
    }

    if(!EnetPort2IfIsUp())
    {
        Port2linkFlag = FALSE;
        EnetPort2HttpServerInit();

        if(ipAddr2)
        {
            Port2linkFlag = TRUE;
        }
    }
    else
    {
        if(EnetPort2LinkIsUp())
        {
            Port2linkFlag = TRUE;
        }
        else
        {
            Port2linkFlag = FALSE;
        }
    }


 
    GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
    GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
    if((TRUE == Port1linkFlag) && (ipAddr1 != 0))
    {
        UARTPuts("\n\r\n\rTo control via Port1 of Ethernet, use the IP address ", -1);
                IpAddrDisplay(ipAddr1);
         GrStringDrawCentered(&sContext[frameBufIdx], "To control via Port1 of Ethernet, type the address", -1, 222,
			     (42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			
        for(i_index = 0; i_index < 4; i_index++)
        {
            ipByte = 0x000000FF & (ipAddr1 >> ((i_index)*8) );

            if(ipByte/100)
            {
                ipMsg[i_msg++] = (ipByte/100) + 48;
                ipByte = ipByte%100;
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte/10)
            {
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte)
            {
                ipMsg[i_msg++] = ipByte + 48;
            }

            ipMsg[i_msg++] = '.';
        }
        ipMsg[--i_msg] = '\0';

        strcat(ipMsg, "/index.html");
        GrStringDrawCentered(&sContext[frameBufIdx], ipMsg, -1, 135,
					(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
	GrStringDrawCentered(&sContext[frameBufIdx], " in the host web", -1, 280,
					(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
        GrStringDrawCentered(&sContext[frameBufIdx], "browser to access embedded demo page on the target.", -1, 215,
					(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);					
    }
    else
    {
        UARTPuts("\n\rNetwork Connection to Port1 failed.\n\r", -1);
		GrStringDrawCentered(&sContext[frameBufIdx], "Network Connection to Port1 failed !!!", -1, 240,
						(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }

    if((TRUE == Port2linkFlag) && (ipAddr2 != 0))
    {
        UARTPuts("\n\rTo control via Port2 of Ethernet, use the IP address ", -1);
        IpAddrDisplay(ipAddr2);
  
        GrStringDrawCentered(&sContext[frameBufIdx], "To control via Port2 of Ethernet, type the address", -1, 222,
			     (42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	i_msg = 7;

        for(i_index = 0; i_index < 4; i_index++)
        {
            ipByte = 0x000000FF & (ipAddr2 >> ((i_index)*8) );

            if(ipByte/100)
            {
                ipMsg[i_msg++] = (ipByte/100) + 48;
                ipByte = ipByte%100;
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte/10)
            {
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte)
            {
                ipMsg[i_msg++] = ipByte + 48;
            }

            ipMsg[i_msg++] = '.';
        }
        ipMsg[--i_msg] = '\0';
   
        strcat(ipMsg, "/index.html");
        GrStringDrawCentered(&sContext[frameBufIdx], ipMsg, -1, 135,
					(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	GrStringDrawCentered(&sContext[frameBufIdx], " in the host web", -1, 280,
					(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	GrStringDrawCentered(&sContext[frameBufIdx], "browser to access embedded demo page on the target.", -1, 215,
					(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }
    else
    {
        UARTPuts("\n\rNetwork Connection to Port2 failed.\n\r", -1);
	GrStringDrawCentered(&sContext[frameBufIdx], "Network Connection to Port2 failed !!!", -1, 240,
			    (42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }
}

/*
** ECAP demo
*/
static void ActionECAPdemo(void)
{
	EcapBkLightVary();
    UpdateUartConsoleHelp();
}

/*
** Action when no touch is detected 
*/
static void ActionIdle(void)
{
    ; /* Perform nothing */
}


/*
** Action for Menu
*/
static void ActionMenu(void)
{
    imageCount = 1;
	updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu introduction icon click
*/
static void ActionMenuIntro(void)
{
    imageCount = CLICK_IDX_INTRO;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
	 
}

/*
** Action for menu webdemo icon click
*/
static void ActionMenuWebDemo(void) 
{ 
    imageCount = CLICK_IDX_CHOICE;
    ActionEnetInit();
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu McASP icon click
*/
static void ActionMenuMcASP(void) 
{
    imageCount = CLICK_IDX_MCASP;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu MMCSD icon click
*/
static void ActionMenuMMCSD(void) 
{
    imageCount = CLICK_IDX_MMCSD;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu Uart icon click
*/
static void ActionMenuUart(void) 
{
    imageCount = CLICK_IDX_UART;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu RTC icon click
*/
static void ActionMenuRTC(void) 
{
    imageCount = CLICK_IDX_RTC;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu timer icon click
*/
static void ActionMenuTimer(void) 
{
    imageCount = CLICK_IDX_TIMER;
    tmr2Flag  = FALSE;
    tmrStepVary = TRUE;
    Timer2Start();
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu ethernet icon click
*/
static void ActionMenuEthernet(void) 
{
    imageCount = CLICK_IDX_ETHERNET;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}


/*
** Action for menu ethernet icon click
*/
static void ActionMenuECAP(void) 
{
    imageCount = CLICK_IDX_ECAP;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu GPIO icon click
*/
static void ActionMenuGPIO(void) 
{
    imageCount = CLICK_IDX_GPIO;
    updatePage(imageCount);
    UpdateUartConsoleHelp();

    while(UARTCharsAvail(SOC_UART_0_REGS))
    {
        UARTGetc();
    }

    clickIdx = 0;
    gpioLedBlink();
}

/*
** Action for menu I2C icon click
*/
static void ActionMenuI2C(void) 
{
    imageCount = CLICK_IDX_I2C;
    I2CDemoFlag = TRUE;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for menu PM icon click
*/
static void ActionMenuPM(void) 
{
    imageCount = CLICK_IDX_PM;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for DVFS icon click
*/
static void ActionMenuDVFS(void)
{
    imageCount = CLICK_IDX_DVFS;
    updatePage(imageCount);
    UpdateUartConsoleHelp();
}

/*
** Action for RTC Time Set. This is a blocking call.
*/
static void ActionTimeSet(void)
{
    RtcTimeCalSet(); 
    imageCount = CLICK_IDX_RTC;
    updatePage(imageCount);
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

    UpdateUartConsoleHelp();
}

/*
** Steps to be taken when DS1 is selected
*/
void ActionDeepSleep1(void)
{
    ConsoleUtilsPrintf("\n\rEntering DS1 Power Saving Mode...");
    PowerSaveModeEnter(ds1Data, SLEEP_MODE_DS1);
    UpdateUartConsoleHelp();
}

/*
** Steps to be taken when DS0 is selected
*/
void ActionDeepSleep0(void)
{
    ConsoleUtilsPrintf("\n\rEntering DS0 Power Saving Mode...");
    PowerSaveModeEnter(ds0Data, SLEEP_MODE_DS0);
    UpdateUartConsoleHelp();
}

/*
** Select wake source
*/
void ActionWakeOnTsc()
{
    /* update wake source */
    wakeSource = WAKE_SOURCE_TSC;
    ConsoleUtilsPrintf("\n\n Touch screen wake selected \r\n");

    /* update the display and console */
    updatePage(imageCount);

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

    /* update the display and console */
    updatePage(imageCount);

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

    /* update the display and console */
    updatePage(imageCount);

    UpdateUartConsoleHelp();
}

/*
** Select wake source
*/
void ActionWakeOnGpio()
{
    /* update wake source */
    wakeSource = WAKE_SOURCE_GPIO;
    ConsoleUtilsPrintf("\n\n GPIO wake selected \r\n");

    /* update the display and console */
    updatePage(imageCount);

    UpdateUartConsoleHelp();
}

/*
** Select RTC wake source
*/
void ActionWakeOnRTC(void)
{
    if((DEVICE_VERSION_2_0 == deviceVersion) ||
       (DEVICE_VERSION_2_1 == deviceVersion))
    {
        /* update wake source */
        wakeSource = WAKE_SOURCE_RTC;

        ConsoleUtilsPrintf("\n\n RTC Alarm wake selected \r\n");

        /* update the display */
        updatePage(imageCount);

        UpdateUartConsoleHelp();
    }
    else
    {
        ConsoleUtilsPrintf("\nRTC Alarm wake not supported in this SoC ver.\n");
        ActionWakeOnTsc();
    }
}

/*
** Toggle colors on LCD
*/
static void toggleColors(void)
{
	static unsigned int colour[2] = {ClrBlue, ClrRed};
	static unsigned int index = 0;
    static tRectangle sRect1 = {220,210,248,238};
	
    GrContextForegroundSet(&sContext[!frameBufIdx], colour[index]);
    GrRectFill(&sContext[!frameBufIdx], &sRect1);

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
	
	index = !index;
}

/****************************** End of file *********************************/
  
