//*****************************************************************************
//
// usb_dev_mouse.c - Main routines for the enumeration example.
//
// This is part of AM1808 Sitaraware firmware package, modified and reused from revision 6288
// of the DK-LM3S9B96 Firmware Package.
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


#include "raster.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "pushbutton.h"
#include "checkbox.h"
#include "radiobutton.h"
#include "container.h"
#include "slider.h"
#include "debug.h"
#include "usblib.h"
#include "usbhid.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdhid.h"
#include "usbdhidmouse.h"
#include "usb_mouse_structs.h"
#include "touch.h"
#include "delay.h"
#include "cache.h"
#include "mmu.h"

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
****************************************************************************/

#define LCD_SIZE (800*480*4)
#define PALETTE_SIZE 32
#define LCD_WIDTH 800
#define LCD_HEIGHT 480
#define PALETTE_OFFSET  4
#define PIXEL_24_BPP_PACKED     (0x0)
#define PIXEL_24_BPP_UNPACKED   (0x1)
#define FRAME_BUFFER_0  0
#define FRAME_BUFFER_1  1
#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)

static void MMUConfigAndEnable(void);
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

//Graphics context structure
tContext g_sContext;

// Memory that is used as the local frame buffer.
#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#elif defined __TMS470__ || defined _TMS320C6X
#pragma DATA_ALIGN(g_pucBuffer, 4);
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#else
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)]__attribute__ ((aligned (4)));
#endif

// The graphics library display structure.
tDisplay g_s35_800x480x24Display;

// 32 byte Palette.
unsigned int palette_32b[PALETTE_SIZE/sizeof(unsigned int)] =
            {0x4000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u};

//*****************************************************************************
//
// Holds the current state of the push button - pressed or not.
//
//*****************************************************************************
static volatile unsigned char g_ucButtons;

//*****************************************************************************
//
// This structure defines the area of the display that is devoted to a
// mouse button.  Touchscreen input in this area is translated into press and
// release messages for the given button.
//
//*****************************************************************************
typedef struct
{
    const char *pszLabel;
    unsigned short usX;
    unsigned short usWidth;
    unsigned char  ucReportFlag;
}
tMouseButtonArea;

//******************************************************************************
//
// The height of the mouse button bar at the bottom of the display and the
// number of buttons it contains.
//
//******************************************************************************
#define BUTTON_HEIGHT 50
#define NUM_MOUSE_BUTTONS 3

//*******************************************************************************
//
// Numer of pressure reads that going to perform.  This is required to avoid the debouncing effect
//
//*******************************************************************************
#define DEBOUNCE_READ 50

//*******************************************************************************
//
// Definitions of the positions and labels for each of the three mouse buttons.
//
//******************************************************************************
static tMouseButtonArea g_sMouseButtons[NUM_MOUSE_BUTTONS] =
{
    { "Left Button", 160,   160, MOUSE_REPORT_BUTTON_1 },
    { "Center Button", 320, 161, MOUSE_REPORT_BUTTON_3 },
    { "Right Button", 481, 160, MOUSE_REPORT_BUTTON_2 }
};

//*****************************************************************************
//
// Holds command bits used to signal the main loop to perform various tasks.
//
//*****************************************************************************
volatile unsigned int g_ulCommands;

//*****************************************************************************
//
// Holds the touch screen press status
//
//*****************************************************************************
extern volatile unsigned int IsTSPress;
extern volatile int touchIntCount;
unsigned int g_iTouch;
unsigned int g_move;
unsigned short g_released;
//*****************************************************************************
//
// Holds the current press position for the touchscreen.
//
//*****************************************************************************
static int g_lScreenX;
static int g_lScreenY;

//*****************************************************************************
//
// Holds the previous press position for the touchscreen.
//
//*****************************************************************************
static volatile int g_lScreenStartX;
static volatile int g_lScreenStartY;


//*****************************************************************************
//
// A flag used to indicate whether or not we are currently connected to the USB
// host.
//
//*****************************************************************************
volatile tBoolean bConnected;

//*****************************************************************************
//
// This enumeration holds the various states that the mouse can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    MOUSE_STATE_UNCONFIGURED,

    //
    // No keys to send and not waiting on data.
    //
    MOUSE_STATE_IDLE,

    //
    // Waiting on data to be sent out.
    //
    MOUSE_STATE_SENDING
}
g_eMouseState = MOUSE_STATE_UNCONFIGURED;


//*****************************************************************************
//
// This function updates the color STN display to show button state.
//
// This function is called from ButtonHandler to update the display showing
// the state of each of the buttons.
//
// Returns None.
//
//*****************************************************************************
void
UpdateDisplay(unsigned char ucButtons, tBoolean bRedraw)
{
    unsigned int ulLoop;
    tRectangle sRect, sRectOutline;
    static unsigned char ucLastButtons;

    //
    // Initialize the Y coordinates of the button rectangle.
    //
    sRectOutline.sYMin = GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT;
    sRectOutline.sYMax = GrContextDpyHeightGet(&g_sContext) - 1;
    sRect.sYMin = sRectOutline.sYMin + 1;
    sRect.sYMax = sRectOutline.sYMax - 1;

    //
    // Set the font we use for the button text.
    //
    GrContextFontSet(&g_sContext, &g_sFontCmss18);

    //
    // Loop through each of the mouse buttons, drawing each in turn.
    //
    for(ulLoop = 0; ulLoop < NUM_MOUSE_BUTTONS; ulLoop++)
    {
        //
        // Draw the outline if we are redrawing the whole button area.
        //
        if(bRedraw)
        {
            GrContextForegroundSet(&g_sContext, ClrWhite);

            sRectOutline.sXMin = g_sMouseButtons[ulLoop].usX;
            sRectOutline.sXMax = (sRectOutline.sXMin +
                                 g_sMouseButtons[ulLoop].usWidth) - 1;

            GrRectDraw(&g_sContext, &sRectOutline);
        }

        //
        // Has the button state changed since we last drew it or are we
        // drawing the buttons unconditionally?
        //
        if(((g_ucButtons & g_sMouseButtons[ulLoop].ucReportFlag) !=
           (ucLastButtons & g_sMouseButtons[ulLoop].ucReportFlag)) || bRedraw)
        {
            //
            // Set the appropriate button color depending upon whether the
            // button is pressed or not.
            //
            GrContextForegroundSet(&g_sContext, ((g_ucButtons &
                                   g_sMouseButtons[ulLoop].ucReportFlag) ?
                                   ClrRed : ClrGreen));

            sRect.sXMin = g_sMouseButtons[ulLoop].usX + 1;
            sRect.sXMax = (sRect.sXMin + g_sMouseButtons[ulLoop].usWidth) - 3;
            GrRectFill(&g_sContext, &sRect);

            //
            // Draw the button text.
            //
            GrContextForegroundSet(&g_sContext, ClrWhite);
            GrStringDrawCentered(&g_sContext, g_sMouseButtons[ulLoop].pszLabel,
                                 -1, (sRect.sXMin + sRect.sXMax) / 2,
                                 (sRect.sYMin + sRect.sYMax) / 2, 0);
        }
    }

    //
    // Remember the button state we just drew.
    //
    ucLastButtons = ucButtons;
}


//*****************************************************************************
//
// This function handles updates due to touchscreen input.
//
// This function is called periodically from the main loop to check the
// touchscreen state and, if necessary, send a HID report back to the host
// system.
//
// Returns Returns \b true on success or \b false if an error is detected.
//
//*****************************************************************************
static void
TouchHandler(void)
{
    short lDeltaX = 0;
    short lDeltaY = 0;
    unsigned int ulLoop;
    static unsigned char ucButtons = 0;
    static unsigned int eventCount =0;

    //
    //If there is a touch event, consider this as a first touch
    //
    if(g_iTouch)
    {
        //
        //Reset the delta
        //
        lDeltaX =0;
        lDeltaY =0;
        eventCount++;

        //
        //Wait for some more event to confirm a movement
        //
        if(eventCount >= 20)
        {
            //
            //There is a movement, so reset the touch flag
            //
            g_iTouch= 0;
            //
            //Set the movement flag
            //
            g_move = 1;
            //
            //reset the event couter
            //
            eventCount = 0;
        }
        //
        //Wait for 10ms and see if still there is a movement
        //
        delay(10);
    }
    else
    {
        //
        // If there is movement
        //
        if(g_move)
        {
            //
            //Wait for some more time to confirm
            //
            delay(50);
            //
            //Reset the deltas
            //
            lDeltaX =0;
            lDeltaY =0;
            //
            //Reset the movement flag
            //
            g_move = 0;
        }
        //
        //If still not released the touch
        //
        else if(!TouchReleaseDetect())
        {
            //
            //get the current x and y
            //
            lDeltaY = g_lScreenY;
            lDeltaX = g_lScreenX;
            //
            //Calculate the delta from the previous value
            //
            lDeltaX -= g_lScreenStartX;
            lDeltaY -= g_lScreenStartY;

            delay(20);
            //
            //check is delta is more than 15, then reset to a lesser value
            //
            if(lDeltaX > 15)
                lDeltaX = 10;
            if(lDeltaX < -15)
                lDeltaX = -10;
            if(lDeltaY > 15)
                lDeltaY = 5;
            if(lDeltaY < -15)
                lDeltaY = -5;
            //
            //if there is continious touch at the same point dont allow the cursor to move
            //
            if(lDeltaX < 3 && lDeltaX > -3)
                lDeltaX =0;
            if(lDeltaY < 3 && lDeltaY > -3)
                lDeltaY =0;
            //
            //Reset the touch flag
            //
            g_iTouch = 0;
        }

    }

    //
    // Is the press within the button area?  If so, determine which
    // button has been pressed.
    //
    if(g_lScreenY >= (GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT))
    {
        lDeltaX =0;
        lDeltaY =0;
        //
        // Run through the list of buttons to determine which one was
        // pressed.
        //
        if(!g_released)
        {
            for(ulLoop = 0; ulLoop < NUM_MOUSE_BUTTONS; ulLoop++)
            {
                if((g_lScreenX >= g_sMouseButtons[ulLoop].usX) &&
                   (g_lScreenX < (g_sMouseButtons[ulLoop].usX +
                    g_sMouseButtons[ulLoop].usWidth)))
                {
                    g_ucButtons |= g_sMouseButtons[ulLoop].ucReportFlag;
                    break;
                }
            }
        }
        else
        {
            g_released = 0;
        }
    }

    //
    //Store the current x and y, we need it to calculate the delta
    //
    g_lScreenStartX = g_lScreenX;
    g_lScreenStartY = g_lScreenY;

    //
    // Was there any movement or change in button state?
    //
    if(lDeltaX || lDeltaY || (ucButtons != g_ucButtons))
    {
        //
        // Yes - send a report back to the host after clipping the deltas
        // to the maximum we can support.
        //
        lDeltaX = (lDeltaX > 127) ? 127 : lDeltaX;
        lDeltaX = (lDeltaX < -128) ? -128 : lDeltaX;
        lDeltaY = (lDeltaY > 127) ? 127 : lDeltaY;
        lDeltaY = (lDeltaY < -128) ? -128 : lDeltaY;

        //
        // Remember the current button state.
        //
        ucButtons = g_ucButtons;

        //
        // Send the report back to the host.
        //
        USBDHIDMouseStateChange((void *)&g_sMouseDevice,
                                (char)lDeltaX, (char)lDeltaY,
                                ucButtons);

    }

    //
    // Update the button portion of the display.
    //
    UpdateDisplay(ucButtons, false);
}

//*****************************************************************************
//
//  This is the callback from the USB device HID mouse class driver.
//
// \param pvCBData is ignored by this function.
// \param ulEvent is one of the valid events for a mouse device.
// \param ulMsgParam is defined by the event that occurs.
// \param pvMsgData is a pointer to data that is defined by the event that
// occurs.
//
// This function will be called to inform the application when a change occurs
// during operation as a HID class USB mouse device.
//
// \return This function will return 0.
//
//*****************************************************************************
unsigned int
MouseHandler(void *pvCBData, unsigned int ulEvent,
             unsigned int ulMsgData, void *pvMsgData)
{
    switch(ulEvent)
    {
        //
        // The USB host has connected to and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            bConnected = true;
            break;
        }

        //
        // The USB host has disconnected from the device.
        //
        case USB_EVENT_DISCONNECTED:
        {
            bConnected = false;
            g_eMouseState = MOUSE_STATE_UNCONFIGURED;
            break;
        }

        //
        // A report was sent to the host. We are free to send another.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            break;
        }

    }
    return(0);
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
    RasterHparamConfig(SOC_LCDC_0_REGS, 800, 48, 40, 40);

    /* Configuring vertical timing parameters */
    RasterVparamConfig(SOC_LCDC_0_REGS, 480, 3, 13, 29);

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

//*****************************************************************************
//
// Sets up the AINTC Interrupt
//
//*****************************************************************************

static void USB0AINTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_USB0);
}

static void USBInterruptEnable(void)
{
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Configuring AINTC to receive USB interrupts. */
    USB0AINTCConfigure();

    LCDAINTCConfigure();
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


int main(void)
{
    tRectangle sRect;
    unsigned int i = 0;
    unsigned char *src = (unsigned char *) palette_32b;
    unsigned char *dest = (unsigned char *) (g_pucBuffer+4);

    MMUConfigAndEnable();

    //
    //Enable USB module clock
    //
    USB0ModuleClkConfig();
    //
    //Enable DM timer 3 module clock
    //
    DMTimer3ModuleClkConfig();
    //
    //Enbale touch screen module colock
    //
    TSCADCModuleClkConfig();
    //
    //Enable touch screen ADC pinmux
    //
    TSCADCPinMuxSetUp();
    //
    //Enbale USB interrupts
    //
    USBInterruptEnable();
    //
    //Switch ON LCD back light
    //
    LCDBackLightEnable();
    //
    //UDP Pin control
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
    //
    //Register touch scren interrupt
    //
    TouchIntRegister();

    IntSystemEnable(SYS_INT_TINT3);
    IntPrioritySet(SYS_INT_TINT3, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

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

    GrOffScreen24BPPInit(&g_s35_800x480x24Display, g_pucBuffer, LCD_WIDTH, LCD_HEIGHT);

    /* Initialize a drawing context. */
    GrContextInit(&g_sContext, &g_s35_800x480x24Display);

    /* enable End of frame interrupt */
    RasterEndOfFrameIntEnable(SOC_LCDC_0_REGS);

    /* enable raster */
    RasterEnable(SOC_LCDC_0_REGS);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.sXMin = 0;
    sRect.sYMin = 0;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = 23;
    GrContextForegroundSet(&g_sContext, ClrDarkBlue);
    GrRectFill(&g_sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrRectDraw(&g_sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&g_sContext, &g_sFontCm20);
    GrStringDrawCentered(&g_sContext, "usb-dev-mouse", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 10, 0);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.sXMin = 0;
    sRect.sYMin = 25;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT - 2;
    GrContextForegroundSet(&g_sContext, ClrDarkGray);
    GrRectFill(&g_sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&g_sContext, ClrRed);
    GrRectDraw(&g_sContext, &sRect);

    //
    // Draw the buttons in their initial (unpressed)state.
    //
    UpdateDisplay(g_ucButtons, true);

    //
    //Initialize touch screen
    //
    TouchInit();
    //
    //Touch screen Interrupt enbale
    //
    TouchIntEnable();
    //
    //Touch Screen Enable
    //
    TouchEnable();
    //
    // Initialize the mouse
    //
    USBDHIDMouseInit(0, (tUSBDHIDMouseDevice *)&g_sMouseDevice);

    //
    // Drop into the main loop.
    //
    while(1)
    {
        //
        // Wait for USB configuration to complete.
        //
        while(!bConnected)
        {
        }

        //
        //Wait till someone touches the screen
        //
        while(!g_iTouch)
        {
            g_iTouch = TouchDetect();
        }

        //
        //Loop here as long as someone moving the finger/stylus on the touch screen
        //
        do
        {
            //
            //If so, read the x and Y vlaue and give it to touch handler
            //
            TouchCoOrdGet(&g_lScreenX, &g_lScreenY);
            //
            //Call touch handler
            //
            TouchHandler();

        }while(TouchDetect());
        //
        //Touch is released
        //
        g_released = 1;
        //
        //Reset the button status
        //
        g_ucButtons = 0;
        //
        //Call the touch handler to update the release status
        //
        TouchHandler();
        //
        //Reset the touch flag
        //
        g_iTouch = 0;
    }

}

