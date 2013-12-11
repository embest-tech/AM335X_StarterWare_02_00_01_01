//*****************************************************************************
//
// usb_dev_mouse.c - Main routines for the enumeration example.
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
#include "touch.h"
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
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usb_structs.h"
#include "delay.h"
#include "cp15.h"
#include "mmu.h"

/*****************************************************************************
*
* Holds the current state of the push button - pressed or not.
*
*****************************************************************************/

volatile unsigned char g_ucButtons;

/*****************************************************************************
*
* Flag used by Timer ISR.
*
*****************************************************************************/

unsigned int timerIsrFlag = FALSE;

/*****************************************************************************
*
* This structure defines the area of the display that is devoted to a
* mouse button.  Touchscreen input in this area is translated into press and
* release messages for the given button.
*
*****************************************************************************/

typedef struct
{
    const char *pszLabel;
    unsigned short usX;
    unsigned short usWidth;
    unsigned char  ucReportFlag;
}
tMouseButtonArea;

/*****************************************************************************
*
* Definitions of the positions and labels for each of the three mouse buttons.
*
*****************************************************************************/

static tMouseButtonArea g_sMouseButtons[NUM_MOUSE_BUTTONS] =
{
    { "Left Button", 160,   160, MOUSE_REPORT_BUTTON_1 },
    { "Center Button", 320, 161, MOUSE_REPORT_BUTTON_3 },
    { "Right Button", 481, 160, MOUSE_REPORT_BUTTON_2 }
};

/*****************************************************************************
*
* Holds command bits used to signal the main loop to perform various tasks.
*
*****************************************************************************/

volatile unsigned int g_ulCommands;

/*****************************************************************************
*
*  Holds the touch screen press status.
*
*****************************************************************************/

unsigned int g_move;
unsigned short g_released;

/*****************************************************************************
*
* Holds the previous press position for the touchscreen.
*
*****************************************************************************/

static volatile int g_lScreenStartX;
static volatile int g_lScreenStartY;

/*****************************************************************************
*
* Holds the current press position for the touchscreen.
*
*****************************************************************************/

static int g_lScreenX;
static int g_lScreenY;
unsigned short int g_iTouch = 0;

/******************************************************************************
*
*  External Graphics context used to show text on the color STN display.
*
*******************************************************************************/

extern tContext g_sContext;

/******************************************************************************
*
*  External HID Mouse definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDHIDMouseDevice g_sMouseDevice;

/****************************************************************************
*
* External flag variable which holds command bits used to signal the main loop
* to perform various tasks.
*
*****************************************************************************/

extern unsigned int g_ulFlags;

/*****************************************************************************
*
*  Status variable used by the composite main routine to display the status of
*  the device.
*
******************************************************************************/

extern char *g_pcStatus;

/*****************************************************************************
*
* Holds the touch screen press status
*
*****************************************************************************/
extern volatile unsigned int IsTSPress;
extern volatile int touchIntCount;

/*****************************************************************************
*
*  I2c function pointer prototype.
*
******************************************************************************/

extern void (*I2CpFunc)(void);

/*****************************************************************************
*
* This enumeration holds the various states that the mouse can be in during
* normal operation.
*
*****************************************************************************/

volatile enum
{
    /* Unconfigured. */

    MOUSE_STATE_UNCONFIGURED,

    /* No keys to send and not waiting on data. */

    MOUSE_STATE_IDLE,

    /* Waiting on data to be sent out. */

    MOUSE_STATE_SENDING
}
g_eMouseState = MOUSE_STATE_UNCONFIGURED;

/*****************************************************************************
*
* Values for I2c clock.
*
*****************************************************************************/

#define     I2C_INPUT_CLK           (24000000)
#define     I2C_SCALED_CLK          (8000000)
#define     I2C_OUTPUT_CLK          (100000)

/******************************************************************************
*                                                                             *
* \brief  This function updates the color STN display to show button state.   *
*                                                                             *
*         This function is called from ButtonHandler to update the            *
*         display showing the state of each of the buttons.                   *
*                                                                             *
* \param ucButtons Button state.\n                                            *
*                                                                             *
* \param bRedraw   Flag to tell whether to redraw or not.\n                   *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void UpdateDisplay(unsigned char ucButtons, tBoolean bRedraw)
{
    unsigned int ulLoop;
    tRectangle sRect, sRectOutline;
    static unsigned char ucLastButtons;

    /* Initialize the Y coordinates of the button rectangle. */

    sRectOutline.sYMin = GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT;
    sRectOutline.sYMax = GrContextDpyHeightGet(&g_sContext) - 1;
    sRect.sYMin = sRectOutline.sYMin + 1;
    sRect.sYMax = sRectOutline.sYMax - 1;

    /* Set the font we use for the button text. */

    GrContextFontSet(&g_sContext, &g_sFontCmss18);

    /* Loop through each of the mouse buttons, drawing each in turn. */

    for(ulLoop = 0; ulLoop < NUM_MOUSE_BUTTONS; ulLoop++)
    {
        /* Draw the outline if we are redrawing the whole button area. */

        if(bRedraw)
        {
            GrContextForegroundSet(&g_sContext, ClrWhite);

            sRectOutline.sXMin = g_sMouseButtons[ulLoop].usX;
            sRectOutline.sXMax = (sRectOutline.sXMin +
                                 g_sMouseButtons[ulLoop].usWidth) - 1;

            GrRectDraw(&g_sContext, &sRectOutline);
        }

        /* Has the button state changed since we last drew it or are we
           drawing the buttons unconditionally?
        */
        if(((g_ucButtons & g_sMouseButtons[ulLoop].ucReportFlag) !=
           (ucLastButtons & g_sMouseButtons[ulLoop].ucReportFlag)) || bRedraw)
        {
            /* Set the appropriate button color depending upon whether the
             * button is pressed or not.
            */
            GrContextForegroundSet(&g_sContext, ((g_ucButtons &
                                   g_sMouseButtons[ulLoop].ucReportFlag) ?
                                   ClrRed : ClrGreen));

            sRect.sXMin = g_sMouseButtons[ulLoop].usX + 1;
            sRect.sXMax = (sRect.sXMin + g_sMouseButtons[ulLoop].usWidth) - 3;
            GrRectFill(&g_sContext, &sRect);

            /* Draw the button text. */

            GrContextForegroundSet(&g_sContext, ClrWhite);
            GrStringDrawCentered(&g_sContext, g_sMouseButtons[ulLoop].pszLabel,
                                 -1, (sRect.sXMin + sRect.sXMax) / 2,
                                 (sRect.sYMin + sRect.sYMax) / 2, 0);
        }
    }

    /* Remember the button state we just drew. */

    ucLastButtons = ucButtons;
}

/******************************************************************************
*                                                                             *
* \brief  This function handles updates due to touchscreen input.             *
*                                                                             *
*         This function is called periodically from the main loop to          *
*         check the touchscreen state and, if necessary, send a HID report    *
*         back to the host system.                                            *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void
TouchHandler(void)
{
    short lDeltaX = 0;
    short lDeltaY = 0;
    unsigned int ulLoop;
    static unsigned char ucButtons = 0;
    static unsigned int eventCount =0;

    /* If there is a touch event, consider this as a first touch */
    if(g_iTouch)
    {

        /* Reset the delta */

        lDeltaX =0;
        lDeltaY =0;
        eventCount++;

        /* Wait for some more event to confirm a movement */

        if(eventCount >= 20)
        {
            /* There is a movement, so reset the touch flag */

            g_iTouch= 0;

            /* Set the movement flag */

            g_move = 1;

            /* reset the event couter */

            eventCount = 0;
        }

        /* Wait for 10ms and see if still there is a movement */

        delay(10);
    }
    else
    {
        /* If there is movement */

        if(g_move)
        {
            /* Wait for some more time to confirm */
            delay(50);

            /* Reset the deltas */

            lDeltaX =0;
            lDeltaY =0;

            /* Reset the movement flag */

            g_move = 0;
        }
        /* If still not released the touch */
        else if(!TouchReleaseDetect())
        {
            /* get the current x and y */

            lDeltaY = g_lScreenY;
            lDeltaX = g_lScreenX;

            /* Calculate the delta from the previous value */

            lDeltaX -= g_lScreenStartX;
            lDeltaY -= g_lScreenStartY;

            delay(20);

            /* check is delta is more than 15, then reset to a lesser value */

            if(lDeltaX > 15)
                lDeltaX = 10;
            if(lDeltaX < -15)
                lDeltaX = -10;
            if(lDeltaY > 15)
                lDeltaY = 5;
            if(lDeltaY < -15)
                lDeltaY = -5;

            /* if there is continious touch at the same point dont
               allow the cursor to move.
            */
            if(lDeltaX < 3 && lDeltaX > -3)
                lDeltaX =0;
            if(lDeltaY < 3 && lDeltaY > -3)
                lDeltaY =0;

            /* Reset the touch flag */

            g_iTouch = 0;
        }

    }

    /* Is the press within the button area?  If so, determine which
       button has been pressed.
    */
    if(g_lScreenY >= (GrContextDpyHeightGet(&g_sContext) - BUTTON_HEIGHT))
    {
        lDeltaX =0;
        lDeltaY =0;
        /* Run through the list of buttons to determine which one was
           pressed.
        */
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
    /* Store the current x and y, we need it to calculate the delta */

    g_lScreenStartX = g_lScreenX;
    g_lScreenStartY = g_lScreenY;

    /* Was there any movement or change in button state? */

    if(lDeltaX || lDeltaY || (ucButtons != g_ucButtons))
    {
        /* Yes - send a report back to the host after clipping the deltas
           to the maximum we can support.
        */
        lDeltaX = (lDeltaX > 127) ? 127 : lDeltaX;
        lDeltaX = (lDeltaX < -128) ? -128 : lDeltaX;
        lDeltaY = (lDeltaY > 127) ? 127 : lDeltaY;
        lDeltaY = (lDeltaY < -128) ? -128 : lDeltaY;

        /* Remember the current button state. */

        ucButtons = g_ucButtons;

        /* Send the report back to the host. */

        USBDHIDMouseStateChange((void *)&g_sMouseDevice,
                                (char)lDeltaX, (char)lDeltaY,
                                ucButtons);

    }
    /* Update the button portion of the display. */

    UpdateDisplay(ucButtons, false);
}

/******************************************************************************
*                                                                             *
* \brief  This is the callback from the USB device HID mouse class driver.\n  *
*         This function will be called to inform the application when a change*
*         occurs during operation as a HID class USB mouse device.            *
*                                                                             *
* \param pvCBData is the client-supplied callback pointer for this channel.   *
*                                                                             *
* \param ulEvent identifies the event we are being notified about.            *
*                                                                             *
* \param ulMsgValue is an event-specific value.                               *
*                                                                             *
* \param pvMsgData is an event-specific pointer.                              *
*                                                                             *
* \return The return value is event-specific.                                 *
*                                                                             *
******************************************************************************/
unsigned int
MouseHandler(void *pvCBData, unsigned int ulEvent,
             unsigned int ulMsgData, void *pvMsgData)
{
    unsigned char ulIntsOff;

    switch(ulEvent)
    {

        /* The USB host has connected to and configured the device. */

        case USB_EVENT_CONNECTED:
        {
            g_eMouseState = MOUSE_STATE_IDLE;

            /* Flush our buffers. */

            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            ulIntsOff = IntDisable();
            g_pcStatus = "Host connected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);

            break;
        }

        /* The USB host has disconnected from the device. */

        case USB_EVENT_DISCONNECTED:
        {
            g_eMouseState = MOUSE_STATE_UNCONFIGURED;

            ulIntsOff = IntDisable();
            g_pcStatus = "Host disconnected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);

            break;
        }

        /* A report was sent to the host. We are free to send another. */

        case USB_EVENT_TX_COMPLETE:
        {
            g_eMouseState = MOUSE_STATE_IDLE;
            break;
        }

    }
    return(0);
}

/******************************************************************************
*                                                                             *
* \brief Main loop for Mouse handling function.                               *
*                                                                             *
* \param none.                                                                *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void MouseMain(void)
{
    volatile unsigned int count = 0xF;

    /* Wait till someone touches the screen */

    while((!g_iTouch) && (count--))
    {
        g_iTouch = TouchDetect();
    }

    if(g_iTouch)
    {
        count = 0xF;
        /*
         * Loop here as long as someone moving the finger/stylus on the
         *  touch screen
        */
        do
        {
            /* If so, read the x and Y vlaue and give it to touch handler */

            TouchCoOrdGet(&g_lScreenX, &g_lScreenY);
            TouchHandler();

        }while((TouchDetect()) && (count--));
        g_iTouch =0;
    }
    /* Touch is released */
    g_released = 1;
    /* Reset the button status */
    g_ucButtons = 0;
    /* Call the touch handler to update the release status */
    TouchHandler();
    /* Reset the touch flag */
    g_iTouch = 0;
}
