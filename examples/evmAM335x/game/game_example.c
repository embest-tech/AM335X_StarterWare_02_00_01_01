/**
 * \file  game_example.c
 *
 * \brief This application demonstrates a game in which a blob-like character
 *        tries to find its way out of a maze.
 *
 *        Application Configuration:
 *
 *            Modules Used:
 *                Graphics Library
 *                LCD Controller
 *                TouchScreen Controller
 *                McASP1
 *                I2C1
 *
 *            Configurable Parameters:
 *                None.
 *
 *            Hard-coded configuration of other parameters:
 *                pixel clock frequency - 23MHz
 *                Functional mode       - Raster
 *                Display mode          - TFT
 *                LCD frame resolution  - 800 x 480
 *                Input data format     - 24-BPP Unpacked
 *
 *        Application UseCase:
 *        1. Graphics library will be used to draw the maze and other objects
 *           of the game on the LCD.
 *        2. Object in the game can be moved to up/down/left/right using the
 *           touch screen.
 *
 *        Running the example:
 *        1. On running the application, the starting slide of the game will be
 *           displayed on the LCD.
 *        2. The game is started by pressing the select button on the LCD.
 *        3. During game play, the select button will fire a bullet in the
 *           direction the character is currently facing.
 *        4. The navigation push buttons on the left side of the LCD will cause
 *           the character to walk in the corresponding direction.
 *        5. Score is accumulated for shooting the stars and for finding the
 *           exit to the maze and will be displayed on the screen at the end of
 *           the game.
 *
 *        Limitations:
 *        None.
 */

/* Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
 *
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
 * This file is updated to make it work for StarterWare. */

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>EK-LM3S2965 Quickstart Application (qs_ek-lm3s2965)</h1>
//!
//! A game in which a blob-like character tries to find its way out of a maze.
//! The character starts in the middle of the maze and must find the exit,
//! which will always be located at one of the four corners of the maze.  Once
//! the exit to the maze is located, the character is placed into the middle of
//! a new maze and must find the exit to that maze; this repeats endlessly.
//!
//! The game is started by pressing the select push button on the right side
//! of the board.  During game play, the select push button will fire a bullet
//! in the direction the character is currently facing, and the navigation push
//! buttons on the left side of the board will cause the character to walk in
//! the corresponding direction.
//!
//! Populating the maze are a hundred spinning stars that mindlessly attack the
//! character.  Contact with one of these stars results in the game ending, but
//! the stars go away when shot.
//!
//! Score is accumulated for shooting the stars and for finding the exit to the
//! maze.  The game lasts for only one character, and the score is displayed on
//! the screen at the end of the game.
//!
//! If the CAN device board is attached and is running the can_device_qs
//! application, the volume of the music and sound effects can be adjusted over
//! CAN with the two push buttons on the target board.  The LED on the CAN
//! device board will track the state of the LED on the main board via CAN
//! messages.  The operation of the game will not be affected by the absence of
//! the CAN device board.
//!
//! Since the OLED display on the evaluation board has burn-in characteristics
//! similar to a CRT, the application also contains a screen saver.  The screen
//! saver will only become active if two minutes have passed without the user
//! push button being pressed while waiting to start the game (that is, it will
//! never come on during game play).  Qix-style bouncing lines are drawn on the
//! display by the screen saver.
//!
//! After two minutes of running the screen saver, the display will be turned
//! off and the user LED will blink.  Either mode of screen saver (bouncing
//! lines or blank display) will be exited by pressing the select push button.
//! The select push button will then need to be pressed again to start the
//! game.
//
//*****************************************************************************

#include "debug.h"
#include "globals.h"
#include "images.h"
#include "random.h"
#include "screen_saver.h"
#include "raster.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "soc_AM335x.h"
#include "gameTimer.h"
#include "game_example.h"
#include "dmtimer.h"
#include "grlib.h"

#ifdef _TMS320C6X
#include "dspcache.h"
#else
#include "cache.h"
#endif

#include "gameToneLoop.h"
#include "gameAic31.h"
#include "mmu.h"

/*******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LCD_SIZE             (800*480*4)
#define LCD_CLK              192000000

#define PALETTE_SIZE        32
#define LCD_WIDTH             800
#define LCD_HEIGHT             480

#define PALETTE_OFFSET        4
#define FRAME_BUFFER_0        0
#define FRAME_BUFFER_1        1
#define TIME_ELAPSED_INST    20

#define SIZE_IMAGE_ARRAY    384008u

#define START_ADDR_DDR      (0x80000000)
#define START_ADDR_DEV      (0x44000000)
#define START_ADDR_OCMC     (0x40300000)
#define NUM_SECTIONS_DDR    (512)
#define NUM_SECTIONS_DEV    (960)
#define NUM_SECTIONS_OCMC   (1)
#define PIXEL_24_BPP_PACKED        (0x0)
#define PIXEL_24_BPP_UNPACKED    (0x1)

/*******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void (*I2CpFunc)(void);

/*******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

//*****************************************************************************
//
//  Memory that is used as the local frame buffer. 16 BPP image.
//    Two buffers are used.
//
//*****************************************************************************
#ifdef __TMS470__
#pragma DATA_ALIGN(g_pucBuffer0, 4)
unsigned char g_pucBuffer0[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#pragma DATA_ALIGN(g_pucBuffer1, 4)
unsigned char g_pucBuffer1[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];

#elif defined _TMS320C6X
#pragma DATA_ALIGN(g_pucBuffer0, 4)
unsigned char g_pucBuffer0[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#pragma DATA_ALIGN(g_pucBuffer1, 4)
unsigned char g_pucBuffer1[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
unsigned char g_pucBuffer0[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#pragma data_alignment=4
unsigned char g_pucBuffer1[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];

#else
unsigned char g_pucBuffer0[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)] __attribute__ ((aligned (4)));
unsigned char g_pucBuffer1[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)] __attribute__ ((aligned (4)));
#endif


//*****************************************************************************
//
//  The graphics library display structure.
//
//*****************************************************************************
tDisplay g_s35_800x480x24Display0;
tDisplay g_s35_800x480x24Display1;

//*****************************************************************************
//
//  Display context which is passed to the graphics library.
//
//*****************************************************************************
tContext sContext0;
tContext sContext1;

//*****************************************************************************
//
//    32 byte palette for 24bpp LCD display.
//
//*****************************************************************************
unsigned int palette_32b[PALETTE_SIZE/sizeof(unsigned int)] =
            {0x4000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u};


//*****************************************************************************
//
//    Array to hold the time elapsed for TIME_ELAPSED_INST iterations.
//
//*****************************************************************************
unsigned int frameTransferRate[TIME_ELAPSED_INST];
//*****************************************************************************
//
//    Index used to fill the frameTransferRate array.
//
//*****************************************************************************
unsigned int trIndex = 0;

//*****************************************************************************
//
// A set of flags used to track the state of the application.
//
//*****************************************************************************
unsigned int g_ulFlags;

//*****************************************************************************
//
// The speed of the processor clock, which is therefore the speed of the clock
// that is fed to the peripherals.
//
//*****************************************************************************
unsigned int g_ulSystemClock;

//*****************************************************************************
//
// Storage for a local frame buffer.
//
//*****************************************************************************
unsigned char g_pucFrame[(GAME_W*GAME_H/2)+6+(16*3)];

//*****************************************************************************
//
// The debounced state of the five push buttons.  The bit positions correspond
// to:
//
//     0x00    - All
//     0x0b - Left
//     0x07 - Right
//     0x0E - Up
//     0x0D - Down
//       0x0A    - Select/Fire
//
//*****************************************************************************
unsigned char g_ucSwitches = 0x1f;

/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/*******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
void SetupIntc(void);
void SetUpLCD(void);
static void LCDIsr(void);
void Isr(void);
void ConfigureAINTCIntI2C(void);
void I2CSetup(void);
static void PeripheralsSetup(void);
void ConfigureFrameBuffer(void);
static void MMUConfigAndEnable(void);

#ifdef _TMS320C6X
void CleanDSPCache_LCD (void);
#endif


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


//*****************************************************************************
//
// Delay for a multiple of the system tick clock rate.
//
//*****************************************************************************
void Delay(unsigned int ulCount)
{
    volatile unsigned int count = ulCount;
    //
    // Loop while there are more clock ticks to wait for.
    //
    while(count--);
}


//*****************************************************************************
//
// The main code for the application.  It sets up the peripherals, displays the
// splash screens, and then manages the interaction between the game and the
// screen saver.
//
//*****************************************************************************
int
main(void)
{
    tRectangle sRect;
#ifndef _TMS320C6X
    // unsigned int index;
#endif

    SetupIntc();

#ifdef _TMS320C6X
    CacheEnableMAR((unsigned int)0xC0000000, (unsigned int)0x20000000);
    CacheEnable(L1PCFG_L1PMODE_32K | L1DCFG_L1DMODE_32K | L2CFG_L2MODE_256K);
#else
    /*    Enable MMU    */
    MMUConfigAndEnable();

    /* Enable cache and branch prediction */
    CacheEnable(CACHE_ALL);

#endif

    SetUpLCD();

    ConfigureFrameBuffer();

    GrOffScreen24BPPInit(&g_s35_800x480x24Display0, (unsigned char *)g_pucBuffer0, LCD_WIDTH, LCD_HEIGHT);
    GrOffScreen24BPPInit(&g_s35_800x480x24Display1, (unsigned char *)g_pucBuffer1, LCD_WIDTH, LCD_HEIGHT);

    // Initialize a drawing context.
    GrContextInit(&sContext0, &g_s35_800x480x24Display0);
    GrContextInit(&sContext1, &g_s35_800x480x24Display1);

    /* Enable End of frame0/frame1 interrupt */
    RasterIntEnable(SOC_LCDC_0_REGS, RASTER_END_OF_FRAME0_INT |
                                     RASTER_END_OF_FRAME1_INT);

    /* enable raster */
    RasterEnable(SOC_LCDC_0_REGS);

    PeripheralsSetup();

    AudioCodecInit();
    ToneLoopInit();
    AudioTxActivate();

    // TS init
    InitTouchScreen();

    GrContextForegroundSet(&sContext0, ClrYellowGreen);
    GrContextForegroundSet(&sContext1, ClrYellowGreen);
    sRect.sXMin = GAME_X - 3;
    sRect.sYMin = GAME_Y - 3;
    sRect.sXMax = GAME_X + GAME_W + 3;
    sRect.sYMax = GAME_Y + GAME_H + 3;
    GrRectFill(&sContext0, &sRect);
    GrRectFill(&sContext1, &sRect);

    sRect.sXMin = keys[4].x_min;
    sRect.sYMin = keys[4].y_min;
    sRect.sXMax = keys[4].x_max;
    sRect.sYMax = keys[4].y_max;
    GrRectFill(&sContext0, &sRect);
    GrRectFill(&sContext1, &sRect);

    sRect.sXMin = keys[0].x_min;    // 600, 300
    sRect.sYMin = keys[0].y_min;
    sRect.sXMax = keys[1].x_max;    // 750, 450
    sRect.sYMax = keys[1].y_max;
    GrRectFill(&sContext0, &sRect);
    GrRectFill(&sContext1, &sRect);

    sRect.sXMin = keys[2].x_min;
    sRect.sYMin = keys[2].y_min;
    sRect.sXMax = keys[3].x_max;
    sRect.sYMax = keys[3].y_max;
    GrRectFill(&sContext0, &sRect);
    GrRectFill(&sContext1, &sRect);

    // Draw String - Play/Fire
    GrContextForegroundSet(&sContext0, ClrRed);
    GrContextFontSet(&sContext0, &g_sFontCm22b);
    GrStringDrawCentered(&sContext0, "Play/Fire", -1, 120, 325, 0);
    GrContextForegroundSet(&sContext1, ClrRed);
    GrContextFontSet(&sContext1, &g_sFontCm22b);
    GrStringDrawCentered(&sContext1, "Play/Fire", -1, 120, 325, 0);

    //Delay(0xFFFFF);

    // Configure and start timer2
    Timer2Config();
    Timer2Start();

    // Loop forever.
    while(1)
    {
        // Display the main screen.
        if(MainScreen())
        {
            // The button was pressed, so start the game.
            PlayGame();
        }
        else
        {
            // The button was not pressed during the timeout period, so start
            // the screen saver.
            ScreenSaver();
        }
    }
}

/*
** Configures raster to display image
*/
void SetUpLCD(void)
{
    /* Enable clock for LCD Module */
    LCDModuleClkConfig();

    LCDPinMuxSetup();

    /*
    **Clock for DMA,LIDD and for Core(which encompasses
    ** Raster Active Matrix and Passive Matrix logic)
    ** enabled.
    */
    RasterClocksEnable(SOC_LCDC_0_REGS);

    /* disable raster */
    RasterDisable(SOC_LCDC_0_REGS);

    /* configure the pclk */
    RasterClkConfig(SOC_LCDC_0_REGS, 23040000, LCD_CLK);
    //RasterClkConfig(SOC_LCDC_0_REGS, 7833600, 150000000);

    /* configuring DMA of LCD controller */
    RasterDMAConfig(SOC_LCDC_0_REGS, RASTER_DOUBLE_FRAME_BUFFER,
                    RASTER_BURST_SIZE_16, RASTER_FIFO_THRESHOLD_8,
                    RASTER_BIG_ENDIAN_DISABLE);

    /* configuring modes(ex:tft or stn,color or monochrome etc) for raster controller */
    RasterModeConfig(SOC_LCDC_0_REGS, RASTER_DISPLAY_MODE_TFT_UNPACKED,
                     RASTER_PALETTE_DATA, RASTER_COLOR, RASTER_RIGHT_ALIGNED);


     /* Configuring the polarity of timing parameters of raster controller */
    RasterTiming2Configure(SOC_LCDC_0_REGS, RASTER_FRAME_CLOCK_LOW |
                                            RASTER_LINE_CLOCK_LOW  |
                                            RASTER_PIXEL_CLOCK_HIGH|
                                            RASTER_SYNC_EDGE_RISING|
                                            RASTER_SYNC_CTRL_ACTIVE|
                                            RASTER_AC_BIAS_HIGH, 0, 255);

    /* Configuring horizontal timing parameter */
    RasterHparamConfig(SOC_LCDC_0_REGS, 800, 48, 40, 40);

    /* Configuring vertical timing parameters */
    RasterVparamConfig(SOC_LCDC_0_REGS, 480, 3, 13, 29);


    RasterFIFODMADelayConfig(SOC_LCDC_0_REGS, 128);
}

/*
** configures arm interrupt controller to generate raster interrupt
*/
void SetupIntc(void)
{
    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_LCDCINT, LCDIsr);

    IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the System Interrupts for AINTC    */
    IntSystemEnable(SYS_INT_LCDCINT);

    LCDBackLightEnable();

    UPDNPinControl();

    TouchIntRegister();
}

/*
** For each end of frame interrupt base and ceiling is reconfigured
*/
static void LCDIsr(void)
{
    unsigned int  status;

    status = RasterIntStatus(SOC_LCDC_0_REGS,RASTER_END_OF_FRAME0_INT_STAT |
                                             RASTER_END_OF_FRAME1_INT_STAT );

    status = RasterClearGetIntStatus(SOC_LCDC_0_REGS, status);

    /* Read the timer counter, to see how much time has elapsed */
    // if(trIndex < TIME_ELAPSED_INST)
        // frameTransferRate[trIndex++] = DMTimerCounterGet(SOC_DMTIMER_2_REGS);

    if (status & RASTER_END_OF_FRAME0_INT_STAT)
    {
        flagA = 1;
    }

    if(status & RASTER_END_OF_FRAME1_INT_STAT)
    {
        flagB = 1;
    }

    /* since the frame buffers are constant no need to re-configure again.*/
}


void ConfigureFrameBuffer(void)
{
    unsigned int index = 0;
    unsigned char *dest0, *dest1, *src;

    /* configuring the base ceiling */
    RasterDMAFBConfig(SOC_LCDC_0_REGS,
                      (unsigned int)(g_pucBuffer0+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer0+PALETTE_OFFSET) +
                      (SIZE_IMAGE_ARRAY*4) - 1, FRAME_BUFFER_0);

    RasterDMAFBConfig(SOC_LCDC_0_REGS,
                      (unsigned int)(g_pucBuffer0+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer0+PALETTE_OFFSET) +
                      (SIZE_IMAGE_ARRAY*4) - 1, FRAME_BUFFER_1);

    src = (unsigned char *) palette_32b;
    dest0 = (unsigned char *) (g_pucBuffer0+PALETTE_OFFSET);
    dest1 = (unsigned char *) (g_pucBuffer1+PALETTE_OFFSET);

    // Copy palette info into buffer
    for( index = PALETTE_OFFSET; index < (PALETTE_SIZE+PALETTE_OFFSET); index++)
    {
        *dest0++ = *src;
        *dest1++ = *src++;
    }
}


/*
** Enable all the peripherals in use
*/
static void PeripheralsSetup(void)
{
    McASP1PinMuxSetup();
    I2C1ModuleClkConfig();
    I2CPinMuxSetup(1);
    McASP1ModuleClkConfig();
    EDMAModuleClkConfig();
}

#ifdef _TMS320C6X
void CleanDSPCache_LCD (void)
{
    CacheWB((unsigned int)g_pucBuffer0, sizeof(g_pucBuffer0));
    CacheWB((unsigned int)g_pucBuffer1, sizeof(g_pucBuffer1));
}
#endif
