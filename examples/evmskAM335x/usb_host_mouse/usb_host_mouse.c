//*****************************************************************************
//
// usb_host_mouse.c - main application code for the host mouse example.
//
// This is part of AM 335X StarterWare   Firmware Package. modified and resued from
// revision 6288 of the EK-LM3S3748 Firmware Package.
//
//*****************************************************************************

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

#include "hw_types.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "raster.h"
#include "grlib.h"
#include "usblib.h"
#include "usbhid.h"
#include "usbhost.h"
#include "usbhhid.h"
#include "usbhhidmouse.h"
#include "delay.h"
#include "cache.h"
#include "mmu.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB HID Mouse Host (usb_host_mouse)</h1>
//!
//! This example application demonstrates how to support a USB mouse
//! attached to the evaluation kit board.  The display will show if a mouse
//! is currently connected and the current state of the buttons on the
//! on the bottom status area of the screen.  The main drawing area will show
//! a mouse cursor that can be moved around in the main area of the screen.
//! If the left mouse button is held while moving the mouse, the cursor will
//! draw on the screen.  A side effect of the application not being able to
//! read the current state of the screen is that the cursor will erase
//! anything it moves over while the left mouse button is not pressed.
//
//*****************************************************************************

//*****************************************************************************
//
// The USB controller instance
//
//*****************************************************************************
#define USB_INSTANCE 1

//*****************************************************************************
//
// The size of the host controller's memory pool in bytes.
//
//*****************************************************************************
#define HCD_MEMORY_SIZE         128

//*****************************************************************************
//
// The memory pool to provide to the Host controller driver.
//
//*****************************************************************************
unsigned char g_pHCDPool[HCD_MEMORY_SIZE];

//*****************************************************************************
//
// The size of the mouse device interface's memory pool in bytes.
//
//*****************************************************************************
#define MOUSE_MEMORY_SIZE       128

//*****************************************************************************
//
// The memory pool to provide to the mouse device.
//
//*****************************************************************************
unsigned char g_pucMouseBuffer[MOUSE_MEMORY_SIZE];

//*****************************************************************************
//
// Declare the USB Events driver interface.
//
//*****************************************************************************
DECLARE_EVENT_DRIVER(g_sUSBEventDriver, 0, 0, USBHCDEvents);

//*****************************************************************************
//
// The global that holds all of the host drivers in use in the application.
// In this case, only the Mouse class is loaded.
//
//*****************************************************************************
static tUSBHostClassDriver const * const g_ppHostClassDrivers[] =
{
    &g_USBHIDClassDriver
    ,&g_sUSBEventDriver
};

//*****************************************************************************
//
// This global holds the number of class drivers in the g_ppHostClassDrivers
// list.
//
//*****************************************************************************
static const unsigned int g_ulNumHostClassDrivers =
    sizeof(g_ppHostClassDrivers) / sizeof(tUSBHostClassDriver *);

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
tContext g_sContext;

#define LCD_SIZE (480*272*4)
#define PALETTE_SIZE 32
#define LCD_WIDTH 480
#define LCD_HEIGHT 272
#define PALETTE_OFFSET 4
#define FRAME_BUFFER_0  0
#define FRAME_BUFFER_1  1
#define PIXEL_24_BPP_PACKED     (0x0)
#define PIXEL_24_BPP_UNPACKED   (0x1)

#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)


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

// Memory that is used as the local frame buffer.
#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#elif defined __TMS470__ || defined _TMS320C6X
#pragma DATA_ALIGN(g_pucBuffer, 4);
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#else
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)] __attribute__ ((aligned (4)));
#endif

// The graphics library display structure.
tDisplay g_s35_480x272x24Display;

// 32 byte Palette.
unsigned int palette_32b[PALETTE_SIZE/sizeof(unsigned int)] =
            {0x4000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u};
static void MMUConfigAndEnable(void);

//*****************************************************************************
//
// The global value used to store the mouse instance value.
//
//*****************************************************************************
static unsigned int g_ulMouseInstance;

//*****************************************************************************
//
// The global values used to store the mouse state.
//
//*****************************************************************************
static unsigned int g_ulButtons;
static tRectangle g_sCursor;

//*****************************************************************************
//
// This enumerated type is used to hold the states of the mouse.
//
//*****************************************************************************
enum
{
    //
    // No device is present.
    //
    STATE_NO_DEVICE,

    //
    // Mouse has been detected and needs to be initialized in the main
    // loop.
    //
    STATE_MOUSE_INIT,

    //
    // Mouse is connected and waiting for events.
    //
    STATE_MOUSE_CONNECTED,

    //
    // An unsupported device has been attached.
    //
    STATE_UNKNOWN_DEVICE,

    //
    // A power fault has occured.
    //
    STATE_POWER_FAULT
}
eUSBState;

//*****************************************************************************
//
// These defines are used to define the screen constraints to the application.
//
//*****************************************************************************
#define DISPLAY_BANNER_HEIGHT   23
#define DISPLAY_BANNER_BG       ClrDarkBlue
#define DISPLAY_BANNER_FG       ClrWhite
#define DISPLAY_MOUSE_BG        ClrBlack
#define DISPLAY_MOUSE_FG        ClrWhite
#define DISPLAY_MOUSE_SIZE      2

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned int ulLine)
{
}
#endif

//*****************************************************************************
//
// This function updates the cursor position based on deltas received from
// the mouse device.
//
// \param iXDelta is the signed movement in the X direction.
// \param iYDelta is the signed movement in the Y direction.
//
// This function is called by the mouse handler code when it detects a change
// in the position of the mouse.  It will take the inputs and force them
// to be constrained to the display area of the screen.  If the left mouse
// button is pressed then the mouse will draw on the screen and if it is not
// it will move around normally.  A side effect of not being able to read the
// current state of the screen is that the cursor will erase anything it moves
// over while the left mouse button is not pressed.
//
// \return None.
//
//*****************************************************************************
void
UpdateCursor(int iXDelta, int iYDelta)
{
    int iTemp;

    //
    // If the left button is not pressed then erase the previous cursor
    // position.
    //
    if((g_ulButtons & 1) == 0)
    {
        //
        // Erase the previous cursor.
        //
        GrContextForegroundSet(&g_sContext, DISPLAY_MOUSE_BG);
        GrRectFill(&g_sContext, &g_sCursor);
    }

    //
    // Need to do signed math so use the temporary signed value.
    //
    iTemp = g_sCursor.sXMin;

    //
    // Update the X position without going off the screen.
    //
    if(((int)g_sCursor.sXMin + iXDelta + DISPLAY_MOUSE_SIZE) <
       GrContextDpyWidthGet(&g_sContext))
    {
        //
        // Update the X cursor position.
        //
        iTemp += iXDelta;

        //
        // Don't let the cursor go off the left of the screen either.
        //
        if(iTemp < 0)
        {
            iTemp = 0;
        }
    }

    //
    // Update the X position.
    //
    g_sCursor.sXMin = iTemp;
    g_sCursor.sXMax = iTemp + DISPLAY_MOUSE_SIZE;

    //
    // Need to do signed math so use the temporary signed value.
    //
    iTemp = g_sCursor.sYMin;

    //
    // Update the Y position without going off the screen.
    //
    if(((int)g_sCursor.sYMin + iYDelta) < (GrContextDpyHeightGet(&g_sContext) -
       DISPLAY_BANNER_HEIGHT - (DISPLAY_MOUSE_SIZE+1)))
    {
        //
        // Update the Y cursor position.
        //
        iTemp += iYDelta;

        //
        // Don't let the cursor overwrite the status area of the screen.
        //
        if(iTemp < DISPLAY_BANNER_HEIGHT + 1)
        {
            iTemp = DISPLAY_BANNER_HEIGHT + 1;
        }
    }

    //
    // Update the Y position.
    //
    g_sCursor.sYMin = iTemp;
    g_sCursor.sYMax = iTemp + DISPLAY_MOUSE_SIZE;

    //
    // Draw the new cursor.
    //
    GrContextForegroundSet(&g_sContext, DISPLAY_MOUSE_FG);
    GrRectFill(&g_sContext, &g_sCursor);
}

//*****************************************************************************
//
// This function will update the small mouse button indicators in the status
// bar area of the screen.  This can be called on its own or it will be called
// whenever UpdateStatus() is called as well.
//
//*****************************************************************************
void
UpdateButtons(void)
{
    tRectangle sRect;
    int iButton;

    //
    // Initialize the button indicator.
    //
    sRect.sXMin = GrContextDpyWidthGet(&g_sContext) - 35;
    sRect.sYMin = GrContextDpyHeightGet(&g_sContext) - 18;
    sRect.sXMax = sRect.sXMin + 8;
    sRect.sYMax = sRect.sYMin + 14;

    //
    // Check all three buttons.
    //
    for(iButton = 0; iButton < 3; iButton++)
    {
        //
        // Draw the button indicator red if pressed and black if not pressed.
        //
        if(g_ulButtons & (1 << iButton))
        {
            GrContextForegroundSet(&g_sContext, ClrRed);
        }
        else
        {
            GrContextForegroundSet(&g_sContext, ClrBlack);
        }

        //
        // Draw the back of the  button indicator.
        //
        GrRectFill(&g_sContext, &sRect);

        //
        // Draw the border on the button indicator.
        //
        GrContextForegroundSet(&g_sContext, ClrWhite);
        GrRectDraw(&g_sContext, &sRect);

        //
        // Move to the next button indicator position.
        //
        sRect.sXMin += 10;
        sRect.sXMax += 10;
    }
}

//*****************************************************************************
//
// This function updates the status area of the screen.  It uses the current
// state of the application to print the status bar.
//
//*****************************************************************************
void
UpdateStatus(void)
{
    tRectangle sRect;
    static int USBState = -1;

    if(USBState != eUSBState)
    {
        //
        // Fill the bottom rows of the screen with blue to create the status area.
        //
        sRect.sXMin = 0;
        sRect.sYMin = GrContextDpyHeightGet(&g_sContext) -
                      DISPLAY_BANNER_HEIGHT - 1;
        sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
        sRect.sYMax = sRect.sYMin + DISPLAY_BANNER_HEIGHT;

        //
        // Draw the background of the banner.
        //
        GrContextForegroundSet(&g_sContext, DISPLAY_BANNER_BG);
        GrRectFill(&g_sContext, &sRect);

        //
        // Put a white box around the banner.
        //
        GrContextForegroundSet(&g_sContext, DISPLAY_BANNER_FG);
        GrRectDraw(&g_sContext, &sRect);

        //
        // Put the application name in the middle of the banner.
        //
        GrContextFontSet(&g_sContext, &g_sFontCm20);

        //
        // Update the status on the screen.
        //
        if(eUSBState == STATE_NO_DEVICE)
        {
            //
            // mouse is currently disconnected.
            //
            GrStringDraw(&g_sContext, "No Device", -1, 4, sRect.sYMin + 4, 0);
        }
        else if(eUSBState == STATE_MOUSE_CONNECTED)
        {
            //
            // mouse is connected.
            //
            GrStringDraw(&g_sContext, "Connected", -1, 4, sRect.sYMin + 4, 0);
        }
        else if(eUSBState == STATE_UNKNOWN_DEVICE)
        {
            //
            // mouse is connected.
            //
            GrStringDraw(&g_sContext, "Unknown Device", -1, 4, sRect.sYMin + 4, 0);
        }
        else if(eUSBState == STATE_POWER_FAULT)
        {
            //
            // mouse is connected.
            //
            GrStringDraw(&g_sContext, "Power Fault", -1, 4, sRect.sYMin + 4, 0);
        }

        USBState = eUSBState;
    }

    UpdateButtons();
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
}


/*
** Configures raster to display image
*/
static void SetUpLCD(void)
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

    /* Disable raster */
    RasterDisable(SOC_LCDC_0_REGS);

    /* Configure the pclk */
    RasterClkConfig(SOC_LCDC_0_REGS, 23040000, 192000000);

    /* Configuring DMA of LCD controller */
    RasterDMAConfig(SOC_LCDC_0_REGS, RASTER_DOUBLE_FRAME_BUFFER,
                    RASTER_BURST_SIZE_16, RASTER_FIFO_THRESHOLD_8,
                    RASTER_BIG_ENDIAN_DISABLE);

    /* Configuring modes(ex:tft or stn,color or monochrome etc) for raster controller */
    RasterModeConfig(SOC_LCDC_0_REGS, RASTER_DISPLAY_MODE_TFT_UNPACKED,
                     RASTER_PALETTE_DATA, RASTER_COLOR, RASTER_RIGHT_ALIGNED);


     /* Configuring the polarity of timing parameters of raster controller */
    RasterTiming2Configure(SOC_LCDC_0_REGS, RASTER_FRAME_CLOCK_LOW |
                                            RASTER_LINE_CLOCK_LOW  |
                                            RASTER_PIXEL_CLOCK_HIGH|
                                            RASTER_SYNC_EDGE_RISING|
                                            RASTER_SYNC_CTRL_ACTIVE|
                                            RASTER_AC_BIAS_HIGH     , 0, 255);

    /* Configuring horizontal timing parameter */
    RasterHparamConfig(SOC_LCDC_0_REGS, 480, 4, 8, 43);

    /* Configuring vertical timing parameters */
    RasterVparamConfig(SOC_LCDC_0_REGS, 272, 10, 4, 12);

    RasterFIFODMADelayConfig(SOC_LCDC_0_REGS, 128);

}


static void LCDAINTCConfigure(void)
{
    /* Registering the Interrupt Service Routine(ISR). */
        IntRegister(SYS_INT_LCDCINT, LCDIsr);

        /* Setting the priority for the system interrupt in AINTC. */
        IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

        /* Enabling the system interrupt in AINTC. */
        IntSystemEnable(SYS_INT_LCDCINT);
}

static void USBAINTCConfigure(int usbInstance)
{

    if(usbInstance)
    {
        /* Registering the Interrupt Service Routine(ISR). */
        IntRegister(SYS_INT_USB1, USB1HostIntHandler);

        /* Setting the priority for the system interrupt in AINTC. */
        IntPrioritySet(SYS_INT_USB1, 0, AINTC_HOSTINT_ROUTE_IRQ);

        /* Enabling the system interrupt in AINTC. */
        IntSystemEnable(SYS_INT_USB1);
    }
    else
    {
        /* Registering the Interrupt Service Routine(ISR). */
        IntRegister(SYS_INT_USB0, USB0HostIntHandler);

        /* Setting the priority for the system interrupt in AINTC. */
        IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

        /* Enabling the system interrupt in AINTC. */
        IntSystemEnable(SYS_INT_USB0);
    }
}

static void USBInterruptEnable(int usbInstance)
{
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Configuring AINTC to receive USB interrupts. */
    USBAINTCConfigure(usbInstance);

    LCDAINTCConfigure();
}





//*****************************************************************************
//
// This is the generic callback from host stack.
//
// \param pvData is actually a pointer to a tEventInfo structure.
//
// This function will be called to inform the application when a USB event has
// occured that is outside those releated to the mouse device.  At this
// point this is used to detect unsupported devices being inserted and removed.
// It is also used to inform the application when a power fault has occured.
// This function is required when the g_USBGenericEventDriver is included in
// the host controller driver array that is passed in to the
// USBHCDRegisterDrivers() function.
//
// \return None.
//
//*****************************************************************************
void
USBHCDEvents(void *pvData)
{
    tEventInfo *pEventInfo;

    //
    // Cast this pointer to its actual type.
    //
    pEventInfo = (tEventInfo *)pvData;

    switch(pEventInfo->ulEvent)
    {
        //
        // New mouse detected.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // An unknown device was detected.
            //
            eUSBState = STATE_UNKNOWN_DEVICE;

            UpdateStatus();
            break;
        }

        //
        // Keyboard has been unplugged.
        //
        case USB_EVENT_DISCONNECTED:
        {
            //
            // Unknown device has been removed.
            //
            eUSBState = STATE_NO_DEVICE;

            UpdateStatus();
            break;
        }

        default:
        {
            break;
        }
    }
}

//*****************************************************************************
//
// This is the callback from the USB HID mouse handler.
//
// \param pvCBData is ignored by this function.
// \param ulEvent is one of the valid events for a mouse device.
// \param ulMsgParam is defined by the event that occurs.
// \param pvMsgData is a pointer to data that is defined by the event that
// occurs.
//
// This function will be called to inform the application when a mouse has
// been plugged in or removed and anytime mouse movement or button pressed
// is detected.
//
// \return This function will return 0.
//
//*****************************************************************************
unsigned int
MouseCallback(void *pvCBData, unsigned int ulEvent, unsigned int ulMsgParam,
              void *pvMsgData)
{
    switch(ulEvent)
    {
        //
        // New mouse detected.
        //
        case USB_EVENT_CONNECTED:
        {

            //
            // Proceed to the STATE_MOUSE_INIT state so that the main loop can
            // finish initialized the mouse since USBHMouseInit() cannot be
            // called from within a callback.
            //
            eUSBState = STATE_MOUSE_INIT;

            break;
        }

        //
        // Mouse has been unplugged.
        //
        case USB_EVENT_DISCONNECTED:
        {
            //
            // Change the state so that the main loop knows that the mouse is
            // no longer present.
            //
            eUSBState = STATE_NO_DEVICE;

            //
            // Reset the button state.
            //
            g_ulButtons = 0;

            break;
        }

        //
        // Mouse button press detected.
        //
        case USBH_EVENT_HID_MS_PRESS:
        {
            //
            // Save the new button that was pressed.
            //
            g_ulButtons |= ulMsgParam;

            break;
        }

        //
        // Mouse button release detected.
        //
        case USBH_EVENT_HID_MS_REL:
        {
            //
            // Remove the button from the pressed state.
            //
            g_ulButtons &= ~ulMsgParam;

            break;
        }

        //
        // Mouse X movement detected.
        //
        case USBH_EVENT_HID_MS_X:
        {
            //
            // Update the cursor on the screen.
            //
            UpdateCursor((signed char)ulMsgParam, 0);

            break;
        }

        //
        // Mouse Y movement detected.
        //
        case USBH_EVENT_HID_MS_Y:
        {
            //
            // Update the cursor on the screen.
            //
            UpdateCursor(0, (signed char)ulMsgParam);

            break;
        }
    }

    //
    // Update the status area of the screen.
    //
    UpdateStatus();

    return(0);
}
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

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    tRectangle sRect;

    unsigned int i;
    unsigned char *src, *dest;

      MMUConfigAndEnable();

    //
    //configures arm interrupt controller to generate raster interrupt
    //
    USBInterruptEnable(USB_INSTANCE);

    //
    //LCD back light setup
    //
    LCDBackLightEnable();

    //
    //UPD Pin setup
    //
    UPDNPinControl();

    //
    //Delay timer setup
    //
    DelayTimerSetup();

    //
    //Configures raster to display image
    //
    SetUpLCD();

    /* configuring the base ceiling */

    RasterDMAFBConfig(SOC_LCDC_0_REGS,
                      (unsigned int)(g_pucBuffer+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer+PALETTE_OFFSET) + sizeof(g_pucBuffer) - 2 -
                      PALETTE_OFFSET, FRAME_BUFFER_0);

    RasterDMAFBConfig(SOC_LCDC_0_REGS,
                      (unsigned int)(g_pucBuffer+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer+PALETTE_OFFSET) + sizeof(g_pucBuffer) - 2 -
                      PALETTE_OFFSET, FRAME_BUFFER_1);

    src = (unsigned char *) palette_32b;
    dest = (unsigned char *) (g_pucBuffer+PALETTE_OFFSET);

    // Copy palette info into buffer
    for( i = PALETTE_OFFSET; i < (PALETTE_SIZE+PALETTE_OFFSET); i++)
    {
        *dest++ = *src++;
    }

    GrOffScreen24BPPInit(&g_s35_480x272x24Display, g_pucBuffer, LCD_WIDTH, LCD_HEIGHT);

    // Initialize a drawing context.
    GrContextInit(&g_sContext, &g_s35_480x272x24Display);

    /* enable End of frame interrupt */
    RasterEndOfFrameIntEnable(SOC_LCDC_0_REGS);

    /* enable raster */
    RasterEnable(SOC_LCDC_0_REGS);

    //
    // Fill the top 15 rows of the screen with blue to create the banner.
    //
    sRect.sXMin = 0;
    sRect.sYMin = 0;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = DISPLAY_BANNER_HEIGHT;

    //
    // Set the banner background.
    //
    GrContextForegroundSet(&g_sContext, DISPLAY_BANNER_BG);
    GrRectFill(&g_sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&g_sContext, DISPLAY_BANNER_FG);
    GrRectDraw(&g_sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&g_sContext, &g_sFontCm20);
    GrStringDrawCentered(&g_sContext, "usb_host_mouse", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 10, 0);


    //
    // Register the host class drivers.
    //
    USBHCDRegisterDrivers(USB_INSTANCE, g_ppHostClassDrivers, g_ulNumHostClassDrivers);

    //
    // Initialized the cursor.
    //
    g_ulButtons = 0;
    g_sCursor.sXMin = GrContextDpyWidthGet(&g_sContext) / 2;
    g_sCursor.sXMax = g_sCursor.sXMin + DISPLAY_MOUSE_SIZE;
    g_sCursor.sYMin = GrContextDpyHeightGet(&g_sContext) / 2;
    g_sCursor.sYMax = g_sCursor.sYMin + DISPLAY_MOUSE_SIZE;

    //
    // Update the status on the screen.
    //
    UpdateStatus();

    //
    //  Update the cursor once to display it.
    //
    UpdateCursor(0, 0);

    //
    // Open an instance of the mouse driver.  The mouse does not need
    // to be present at this time, this just saves a place for it and allows
    // the applications to be notified when a mouse is present.
    //
    g_ulMouseInstance =
        USBHMouseOpen(USB_INSTANCE, MouseCallback, g_pucMouseBuffer, MOUSE_MEMORY_SIZE);

    //
    // Initialize the power configuration. This sets the power enable signal
    // to be active high and does not enable the power fault.
    //
    USBHCDPowerConfigInit(USB_INSTANCE, USBHCD_VBUS_AUTO_HIGH);

    //
    // Initialize the host controller stack.
    //
    USBHCDInit(USB_INSTANCE, g_pHCDPool, HCD_MEMORY_SIZE);

    //
    // Call the main loop for the Host controller driver.
    //
    USBHCDMain(USB_INSTANCE, g_ulMouseInstance);

    //
    // The main loop for the application.
    //
    while(1)
    {
        switch(eUSBState)
        {
            //
            // This state is entered when the mouse is first detected.
            //
            case STATE_MOUSE_INIT:
            {
                //
                // Initialize the newly connected mouse.
                //
                USBHMouseInit(g_ulMouseInstance);

                //
                // Proceed to the mouse connected state.
                //
                eUSBState = STATE_MOUSE_CONNECTED;

                //
                // Update the status on the screen.
                //
                UpdateStatus();

                break;
            }
            case STATE_MOUSE_CONNECTED:
            {
                //
                // Nothing is currently done in the main loop when the mouse
                // is connected.
                //
                break;
            }
            case STATE_NO_DEVICE:
            {
                //
                // The mouse is not connected so nothing needs to be done here.
                //
                break;
            }
            default:
            {
                break;
            }
        }

        //
        // Periodically call the main loop for the Host controller driver.
        //
        USBHCDMain(USB_INSTANCE, g_ulMouseInstance);
    }
}
