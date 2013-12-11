//****************************************************************************
//
// usb_dev_chidcdc.c - Main routines for the composite two CDC serial example.
//
//****************************************************************************
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
#include "ustdlib.h"
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "hw_types.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "raster.h"
#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "pushbutton.h"
#include "checkbox.h"
#include "radiobutton.h"
#include "container.h"
#include "slider.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usb.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "hw_usb.h"
#include "delay.h"
#include "usbhid.h"
#include "usbdhid.h"
#include "usbdcomp.h"
#include "usbdhidmouse.h"
#include "usb_structs.h"
#include "touch.h"
#include "debug.h"
#include "mmu.h"


/****************************************************************************
*
* USB composite HID Mouse and CDC serial Device (usb_dev_chidcdc)
*
* This example application turns the evaluation board into a composite USB
* mouse supporting the Human Interface Device class and a CDC serial device
* The mouse pointer will move in a square pattern for the duration of the
* time it is plugged in.  The The serial port application supports the
* USB Communication Device Class, Abstract Control Model to redirect UART0
* traffic to and from the USB host system.  File usb_dev_serial_win2k.inf may
* be used to install the example as a virtual COM port on a Windows2000
* system.  For WindowsXP or Vista, usb_dev_serial.inf should be used.
****************************************************************************/


/****************************************************************************
*
* Holds command bits used to signal the main loop to perform various tasks.
*
*****************************************************************************/

volatile unsigned int g_ulFlags;

/*****************************************************************************
*
*  The memory allocated to hold the composite descriptor that is created by
*  the call to USBDCompositeInit().
*
****************************************************************************/

unsigned char g_pucDescriptorData[DESCRIPTOR_DATA_SIZE];

/******************************************************************************
*
*  Graphics context used to show text on the color STN display.
*
*******************************************************************************/

tContext g_sContext;

/******************************************************************************
*
*  page tables start must be aligned in 16K boundary
*
*******************************************************************************/

#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/******************************************************************************
*
*  Memory that is used as the local frame buffer.
*
*******************************************************************************/

#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#elif defined __TMS470__ || defined _TMS320C6X
#pragma DATA_ALIGN(g_pucBuffer, 4);
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
#else
unsigned char g_pucBuffer[GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)] __attribute__ ((aligned (4)));
#endif

/******************************************************************************
*
*  The graphics library display structure.
*
*******************************************************************************/

tDisplay g_s35_800x480x24Display;

/******************************************************************************
*
*  32 byte Palette.
*
*******************************************************************************/

unsigned int palette_32b[PALETTE_SIZE/sizeof(unsigned int)] =
            {0x4000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u};


/*****************************************************************************
*
*  External mouse definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDHIDMouseDevice g_sMouseDevice;

/*****************************************************************************
*
*  External CDC serial definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDCDCDevice g_sCDCDevice;

/*****************************************************************************
*
*  External composite device definitions used by the composite main routine.
*
******************************************************************************/

extern tUSBDCompositeDevice g_sCompDevice;
extern tCompositeEntry g_psCompDevices[];

/******************************************************************************
*
*  External button variable which holds the current state of the push button
*  - pressed or not.
*
*******************************************************************************/

extern volatile unsigned char g_ucButtons;

/*****************************************************************************
*
*  External status variable used by the composite main routine.
*
******************************************************************************/

extern char *g_pcStatus;

/*****************************************************************************
*
*  External USB buffer pointer used by the composite main routine.
*
******************************************************************************/

extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;

/******************************************************************************
*                                                                             *
* \brief  ISR for LCD.\n                                                      *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void LCDIsr(void)
{
    unsigned int  status;

    status = RasterIntStatus(SOC_LCDC_0_REGS,RASTER_END_OF_FRAME0_INT_STAT |
                                             RASTER_END_OF_FRAME1_INT_STAT );

    status = RasterClearGetIntStatus(SOC_LCDC_0_REGS, status);
}

/******************************************************************************
*                                                                             *
* \brief  This function Configure the AINTC controller for UART.\n            *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void UARTAINTCConfigure(void)
{
    /* Registering the Interrupt Service Routine(ISR). */
        IntRegister(SYS_INT_UART0INT, USBUARTIntHandler);

        /* Setting the priority for the system interrupt in AINTC. */
        IntPrioritySet(SYS_INT_UART0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

        /* Enabling the system interrupt in AINTC. */
        IntSystemEnable(SYS_INT_UART0INT);
}

/******************************************************************************
*                                                                             *
* \brief  This function Configures raster to display image.\n                 *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
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

/******************************************************************************
*                                                                             *
* \brief  This function Configures the AINTC controller for LCD.\n            *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void LCDAINTCConfigure(void)
{
    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_LCDCINT, LCDIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_LCDCINT);
}

/******************************************************************************
*                                                                             *
* \brief  This function Configures the AINTC controller for USB.\n            *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void USB0AINTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
//    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_USB0);
}

/******************************************************************************
*                                                                             *
* \brief  This function Enables the interrupts for USB.\n                     *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void USBInterruptEnable(void)
{

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Configuring AINTC to receive USB0 interrupts. */
    USB0AINTCConfigure();

    LCDAINTCConfigure();

    UARTAINTCConfigure();
}

/******************************************************************************
*                                                                             *
* \brief  Function to setup MMU. This function Maps three regions (1. DDR     *
*         2. OCMC and 3. Device memory) and enables MMU.                      *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
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
                        MMU_MEMTYPE_NORMAL_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                     MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_SHAREABLE(MMU_CACHE_WT_NOWA,
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

/******************************************************************************
*                                                                             *
* \brief  Function to init the LCD.\n                                         *
*                                                                             *
* \param none.                                                                *
*                                                                             *
* \return  none.                                                              *
*                                                                             *
******************************************************************************/
void LCDInit()
{
    unsigned char *src, *dest;
    unsigned int i;

    /* Configures raster to display image  */

//  SetUpLCD();

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

}


/******************************************************************************
*                                                                             *
* \brief  Generic event handler for the composite device.\n                   *
*                                                                             *
* \param pvCBData    : Is the event callback pointer provided during          *
*                       USBDCompositeInit                                     *
*                                                                             *
* \param ulEvent     : Identifies the event we are being called back for.     *
*                                                                             *
* \param ulMsgValue  : Is an event-specific value.                            *
*                                                                             *
* \param pvMsgData   : Is an event-specific pointer.                          *
*                                                                             *
* \return  The return value is event-specific.                                *
*                                                                             *
******************************************************************************/
unsigned int
EventHandler(void *pvCBData, unsigned int ulEvent,
              unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ulIntsOff;


    /* Which event are we being asked to process? */

    switch(ulEvent)
    {

        /* We are connected to a host and communication is now possible. */

        case USB_EVENT_CONNECTED:


            /* Flush our buffers. */


            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            ulIntsOff = IntDisable();
            g_pcStatus = "Host connected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;


        /* The host has disconnected. */

        case USB_EVENT_DISCONNECTED:

            ulIntsOff = IntDisable();
            g_pcStatus = "Host disconnected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;
        default:
            break;
    }

    return(0);
}

/*****************************************************************************
*
*  This is the main loop that runs the application.
*
*****************************************************************************/
int
main(void)
{
    tRectangle sRect;

    MMUConfigAndEnable();

    /* Enable USB module clock */

    USB0ModuleClkConfig();

    /* Enable DM timer 3 module clock */

    DMTimer3ModuleClkConfig();

    /* Enbale touch screen module colock */

    TSCADCModuleClkConfig();

    /* Enable touch screen ADC pinmux */

    TSCADCPinMuxSetUp();

    /* configures arm interrupt controller to generate raster interrupt  */

    USBInterruptEnable();

    /* LCD Back light setup  */

    LCDBackLightEnable();

    /* UPD Pin setup */

    UPDNPinControl();

    /* Delay timer setup */

    DelayTimerSetup();

    /* Configures raster to display image  */

    SetUpLCD();

    /* Register touch scren interrupt */

    TouchIntRegister();

    IntSystemEnable(SYS_INT_TINT3);
    IntPrioritySet(SYS_INT_TINT3, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);


    /* Configures raster to display image  and Copy palette info into buffer */
    LCDInit();

    GrOffScreen24BPPInit(&g_s35_800x480x24Display, g_pucBuffer, LCD_WIDTH, LCD_HEIGHT);

    /* Initialize a drawing context. */
    GrContextInit(&g_sContext, &g_s35_800x480x24Display);

    /* enable End of frame interrupt */
    RasterEndOfFrameIntEnable(SOC_LCDC_0_REGS);

    /* enable raster */
    RasterEnable(SOC_LCDC_0_REGS);


    /* Fill the top 24 rows of the screen with blue to create the banner. */

    sRect.sXMin = 0;
    sRect.sYMin = 0;
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = (MAX_ROW_NUM - 1);
    GrContextForegroundSet(&g_sContext, ClrDarkBlue);
    GrRectFill(&g_sContext, &sRect);

    /* Put a white box around the banner. */

    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrRectDraw(&g_sContext, &sRect);


    /* Put the application name in the middle of the banner. */

    GrContextFontSet(&g_sContext, &g_sFontCm20);
    GrStringDrawCentered(&g_sContext, "usb-dev-composite", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 10, 0);

    sRect.sXMin = 0;
    sRect.sYMin = (MAX_ROW_NUM + 1);
    sRect.sXMax = GrContextDpyWidthGet(&g_sContext) - 1;
    sRect.sYMax = GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT - 2;
    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(&g_sContext, &sRect);



    /* Put a white box around the banner. */

    GrContextForegroundSet(&g_sContext, ClrRed);
    GrRectDraw(&g_sContext, &sRect);


    /* Draw the buttons in their initial (unpressed)state. */

    UpdateDisplay(g_ucButtons, true);

    /*  Show the various static text elements on the color STN display. */

    GrContextFontSet(&g_sContext, TEXT_FONT);
    GrStringDraw(&g_sContext, "Tx bytes:", -1, CDC_STR_X_POSITION,
                  CDC_STR_Y_POSITION, false);
    GrStringDraw(&g_sContext, "Tx buffer:", -1, CDC_STR_X_POSITION,
                 (CDC_STR_Y_POSITION + CDC_STR_Y_DIFF), false);
    GrStringDraw(&g_sContext, "Rx bytes:", -1, CDC_STR_X_POSITION,
                  (CDC_STR_Y_POSITION + (CDC_STR_Y_DIFF * 3)), false);
    GrStringDraw(&g_sContext, "Rx buffer:", -1, CDC_STR_X_POSITION,
                  (CDC_STR_Y_POSITION + (CDC_STR_Y_DIFF * 4)), false);
    DrawBufferMeter(&g_sContext, BUFFER_METER_X_POS, BUFFER_METER_Y_POS);
    DrawBufferMeter(&g_sContext, BUFFER_METER_X_POS,
                    (BUFFER_METER_Y_POS + CDC_BUF_METER_Y_DIFF));

    /* Tell the user what we are up to. */

    DisplayStatus(&g_sContext, " Waiting for host... ");

    /* Initialize touch screen */

    TouchInit();

    /* Touch screen Interrupt enbale */

    TouchIntEnable();

    /* Touch Screen Enable */

    TouchEnable();


    /* Pass the USB library our device information, initialize the USB
       controller and connect the device to the bus.
    */

    g_psCompDevices[0].pvInstance =
        USBDHIDMouseCompositeInit(0, (tUSBDHIDMouseDevice *)&g_sMouseDevice);
    g_psCompDevices[1].pvInstance =
        USBDCDCCompositeInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);


    /* Pass the device information to the USB library and place the device
       on the bus.
    */

    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_DATA_SIZE,
                      g_pucDescriptorData);


    /* Initialize the mouse and serial devices. */
    SerialInit();

    /* Drop into the main loop. */

    while(1)
    {

        /* Allow the main serial routine to run. */

        SerialMain();

        /* Allow the main mouse routine to run. */

        MouseMain();
    }
}
