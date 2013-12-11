//*****************************************************************************
//
// usb_dev_serial.c - Routines for the TwO USB CDC serial example.
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


#include "ustdlib.h"
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "hw_types.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
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
#include "usb_structs.h"
#include "delay.h"
#include "cp15.h"
#include "mmu.h"

/*****************************************************************************
*
* Variables tracking transmit and receive counts.
*
*****************************************************************************/

volatile unsigned int g_ulUARTTxCount1 = 0;
volatile unsigned int g_ulUARTRxCount1 = 0;
volatile unsigned int g_ulUARTTxCount2 = 0;
volatile unsigned int g_ulUARTRxCount2 = 0;

/*****************************************************************************
*
*  Status variable used by the composite main routine to display the status of
*  the device.
*
******************************************************************************/

char *g_pcStatus;

/******************************************************************************
*
*  External Graphics context used to show text on the color STN display.
*
*******************************************************************************/

extern tContext g_sContext;

/****************************************************************************
*
* External flag variable which holds command bits used to signal the main loop
* to perform various tasks.
*
*****************************************************************************/

extern unsigned int g_ulFlags;

/******************************************************************************
*                                                                             *
* \brief  Handles CDC driver notifications related to control and setup of    *
*         the device.\n                                                       *
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
unsigned int ControlHandler(void *pvCBData, unsigned int ulEvent,
                            unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ulIntsOff;


    /* Which event are we being asked to process? */
    switch(ulEvent)
    {

        /* We are connected to a host and communication is now possible. */

        case USB_EVENT_CONNECTED:

            /* Flush our buffers. */

            USBBufferFlush(&g_sTxBuffer1);
            USBBufferFlush(&g_sRxBuffer1);
            USBBufferFlush(&g_sTxBuffer2);
            USBBufferFlush(&g_sRxBuffer2);
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


        /* We don't expect to receive any other events.  Ignore any that show
           up in a release build or hang in a debug build.
        */
        default:
            break;
    }

    return(0);
}

/******************************************************************************
*                                                                             *
* \brief  Handles CDC driver notifications related to the transmit1 channel   *
*         (data to the USB host).\n                                           *
*                                                                             *
*         This function is called by the CDC driver to notify us of any       *
*         events related to operation of the transmit data channel            *
*         (the IN channel carrying data to the USB host)                      *
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
unsigned int TxHandler1(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData)
{

    /* Which event have we been sent? */

    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:

            /* Since we are using the USBBuffer, we don't need to do anything
             here.
            */
            break;

        /* We don't expect to receive any other events.  Ignore any that show
           up in a release build or hang in a debug build.
        */
        default:
            break;
    }

    return(0);
}

/******************************************************************************
*                                                                             *
* \brief  Handles CDC driver notifications related to the transmit2 channel   *
*         (data to the USB host).                                             *
*                                                                             *
*         This function is called by the CDC driver to notify us of any       *
*         events related to operation of the transmit data channel            *
*         (the IN channel carrying data to the USB host)                      *
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
unsigned int TxHandler2(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData)
{

    /* Which event have we been sent? */

    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:

            /* Since we are using the USBBuffer, we don't need to do anything
               here.
            */
            break;


        /* We don't expect to receive any other events.  Ignore any that show
           up in a release build or hang in a debug build.
        */
        default:
            break;
    }
    return(0);
}

/******************************************************************************
*                                                                             *
* \brief  Handles CDC driver notifications related to the receive1  channel   *
*         (data from the USB host).                                           *
*                                                                             *
*        This function is called by the CDC driver to notify us of any events *
*        related to operation of the receive data channel (the OUT channel    *
*        carrying data from the USB host)                                     *
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
unsigned int RxHandler1(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ucChar;


    /* Which event are we being sent? */

    switch(ulEvent)
    {

        /* A new packet has been received. */

        case USB_EVENT_RX_AVAILABLE:
        {

            /* Feed the characters into the txbuffer of other device */
            while(USBBufferRead((tUSBBuffer *)&g_sRxBuffer1, &ucChar, 1))
            {
                USBBufferWrite((tUSBBuffer *)&g_sTxBuffer2,
                               (unsigned char *)&ucChar, 1);
                g_ulUARTTxCount2++;
                g_ulUARTRxCount1++;
            }
            break;
        }


        /* We are being asked how much unprocessed data we have still to
           process. We return 0 if the UART is currently idle or 1 if it is
           in the process of transmitting something. The actual number of
           bytes in the UART FIFO is not important here, merely whether or
           not everything previously sent to us has been transmitted.
        */
        case USB_EVENT_DATA_REMAINING:
        {
            return(0);
        }

        /* We are being asked to provide a buffer into which the next packet
           can be read. We do not support this mode of receiving data so let
           the driver know by returning 0. The CDC driver should not be sending
           this message but this is included just for illustration and
           completeness.
        */
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        /* We don't expect to receive any other events.  Ignore any that show
           up in a release build or hang in a debug build.
        */
        default:
            break;
    }

    return(0);
}

/******************************************************************************
*                                                                             *
* \brief  Handles CDC driver notifications related to the receive2  channel   *
*         (data from the USB host).                                           *
*                                                                             *
*        This function is called by the CDC driver to notify us of any events *
*        related to operation of the receive data channel (the OUT channel    *
*        carrying data from the USB host)                                     *
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
unsigned int RxHandler2(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ucChar;


    /* Which event are we being sent? */

    switch(ulEvent)
    {

        /* A new packet has been received. */

        case USB_EVENT_RX_AVAILABLE:
        {
            /* Feed the characters into the txbuffer of other device */
            while(USBBufferRead((tUSBBuffer *)&g_sRxBuffer2, &ucChar, 1))
            {
                USBBufferWrite((tUSBBuffer *)&g_sTxBuffer1,
                               (unsigned char *)&ucChar, 1);
                g_ulUARTTxCount1++;
                g_ulUARTRxCount2++;
            }
            break;
        }

        /* We are being asked how much unprocessed data we have still to
           process. We return 0 if the UART is currently idle or 1 if it is
           in the process of transmitting something. The actual number of
           bytes in the UART FIFO is not important here, merely whether or
           not everything previously sent to us has been transmitted.
        */
        case USB_EVENT_DATA_REMAINING:
        {
            return(0);
        }

        /* We are being asked to provide a buffer into which the next packet
           can be read. We do not support this mode of receiving data so let
           the driver know by returning 0. The CDC driver should not be sending
           this message but this is included just for illustration and
           completeness.
        */
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        /* We don't expect to receive any other events.  Ignore any that show
           up in a release build or hang in a debug build.
        */
        default:
            break;
    }

    return(0);
}

/******************************************************************************
*                                                                             *
* \brief  Shows the status string on the color STN display.                   *
*                                                                             *
* \param psContext is a pointer to the graphics context representing the      *
*        display.                                                             *
*                                                                             *
* \param pcStatus is a pointer to the string to be shown.                     *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void DisplayStatus(tContext *psContext, char *pcStatus)
{
    tRectangle rectLine;
    int lY;


    /* Calculate the Y coordinate of the top left of the character cell
       for our line of text.
    */
    lY = (GrContextDpyHeightGet(psContext) / 8) -
         (GrFontHeightGet(TEXT_FONT) / 2);


    /* Determine the bounding rectangle for this line of text. We add 4 pixels
       to the height just to ensure that we clear a couple of pixels above and
       below the line of text.
    */
    rectLine.sXMin = 0;
    rectLine.sXMax = GrContextDpyWidthGet(psContext) - 1;
    rectLine.sYMin = lY;
    rectLine.sYMax = lY + GrFontHeightGet(TEXT_FONT) + 3;


    /* Clear the line with black. */

    GrContextForegroundSet(&g_sContext, ClrBlack);
    GrRectFill(psContext, &rectLine);


    /* Draw the new status string */

    GrContextForegroundSet(&g_sContext, ClrWhite);
    GrStringDrawCentered(psContext, pcStatus, -1,
                         GrContextDpyWidthGet(psContext) / 2,
                         GrContextDpyHeightGet(psContext) / 8 , false);
}

/******************************************************************************
*                                                                             *
* \brief Draw a horizontal meter at a given position on the display and fill  *
*         fill it with green.                                                 *
*                                                                             *
* \param psContext is a pointer to the graphics context representing the      *
*        display.                                                             *
*                                                                             *
* \param lX    X - Cordinate.                                                 *
*                                                                             *
* \param lY    Y - Cordinate.                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void DrawBufferMeter(tContext *psContext, int lX, int lY)
{
    tRectangle sRect;
    int lCorrectedY;


    /* Correct the Y coordinate so that the meter is centered on the same line
       as the text caption to its left.
    */
    lCorrectedY = lY - (BUFFER_METER_HEIGHT - TEXT_HEIGHT) ;


    /* Determine the bounding rectangle of the meter. */

    sRect.sXMin = lX;
    sRect.sXMax = lX + BUFFER_METER_WIDTH - 1;
    sRect.sYMin = lCorrectedY;
    sRect.sYMax = lCorrectedY + BUFFER_METER_HEIGHT - 1;


    /* Fill the meter with green to indicate empty */

    GrContextForegroundSet(psContext, ClrGreen);
    GrRectFill(psContext, &sRect);


    /* Put a white box around the meter. */

    GrContextForegroundSet(psContext, ClrWhite);
    GrRectDraw(psContext, &sRect);
}

/******************************************************************************
*                                                                             *
* \brief Draw green and red blocks within a graphical meter on the display to *
*        indicate percentage fullness of some quantity (transmit and receive  *
*        buffers in this case.                                                *
*                                                                             *
* \param psContext is a pointer to the graphics context representing the      *
*        display.                                                             *
*                                                                             *
* \param ulFullPercent is a percentage value to indicate how much to fill.    *
*                                                                             *
* \param lX    X - Cordinate.                                                 *
*                                                                             *
* \param lY    Y - Cordinate.                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void UpdateBufferMeter(tContext *psContext, unsigned int ulFullPercent, int lX,
                       int lY)
{
    tRectangle sRect;
    int lCorrectedY;
    int lXBreak;


    /* Correct the Y coordinate so that the meter is centered on the same line
       as the text caption to its left and so that we avoid the meter's 1 pixel
       white border.
    */
    lCorrectedY = lY - ((BUFFER_METER_HEIGHT - TEXT_HEIGHT) / 2) + 1;


    /*  Determine where the break point between full (red) and empty (green)
        sections occurs.
    */
    lXBreak = (lX + 1) + (ulFullPercent * (BUFFER_METER_WIDTH - 2)) / 100;


    /* Determine the bounding rectangle of the full section. */

    sRect.sXMin = lX + 1;
    sRect.sXMax = lXBreak;
    sRect.sYMin = lCorrectedY;
    sRect.sYMax = lCorrectedY + BUFFER_METER_HEIGHT - 3;


    /* Fill the full section with red (if there is anything to draw) */

    if(ulFullPercent)
    {
        GrContextForegroundSet(psContext, ClrRed);
        GrRectFill(psContext, &sRect);
    }


    /* Fill the empty section with green. */

    sRect.sXMin = lXBreak;
    sRect.sXMax = lX + BUFFER_METER_WIDTH - 2;
    if(sRect.sXMax > sRect.sXMin)
    {
        GrContextForegroundSet(psContext, ClrGreen);
        GrRectFill(psContext, &sRect);
    }


    /* Revert to white for text drawing which may occur later. */

    GrContextForegroundSet(psContext, ClrWhite);

}

/******************************************************************************
*                                                                             *
* \brief Serial initilalization routine.                                      *
*                                                                             *
* \param none.                                                                *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void SerialInit(void)
{

    DelayTimerSetup();

    USBBufferInit((tUSBBuffer *)&g_sTxBuffer1);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer1);
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer2);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer2);
}

/******************************************************************************
*                                                                             *
* \brief Main loop for serial handling function.                              *
*                                                                             *
* \param none.                                                                *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void SerialMain(void)
{
    static unsigned int ulTxCount1 = 0;
    static unsigned int ulRxCount1 = 0;
    static unsigned int ulTxCount2 = 0;
    static unsigned int ulRxCount2 = 0;
    unsigned char Intstatus;
    char pcBuffer[16];
    unsigned int ulFullness;


    /* Have we been asked to update the status display? */

    if(g_ulFlags & COMMAND_STATUS_UPDATE)
    {

        /* Clear the command flag */

        Intstatus = IntDisable();
        g_ulFlags &= ~COMMAND_STATUS_UPDATE;
        IntEnable(Intstatus);
        GrContextFontSet(&g_sContext, TEXT_FONT);
        DisplayStatus(&g_sContext, g_pcStatus);
    }


    /* Has there been any transmit traffic since we last checked? */

    if(ulTxCount1 != g_ulUARTTxCount1)
    {

        /* Take a snapshot of the latest transmit count. */

        ulTxCount1 = g_ulUARTTxCount1;


        /* Update the display of bytes transmitted by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulTxCount1);
        GrStringDraw(&g_sContext, pcBuffer, -1, CDC1_BUF_METER_X_POS,
                      (CDC1_STR_Y_POSITION + CDC_STR_Y_DIFF), true);


        /* Update the RX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's receive buffer is the UART's
           transmit buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sRxBuffer1) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, CDC1_BUF_METER_X_POS,
                           CDC1_BUF_METER_Y_POS);
    }

    /* Has there been any transmit traffic since we last checked? */

    if(ulTxCount2 != g_ulUARTTxCount2)
    {

        /* Take a snapshot of the latest transmit count. */

        ulTxCount2 = g_ulUARTTxCount2;


        /* Update the display of bytes transmitted by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulTxCount2);
        GrStringDraw(&g_sContext, pcBuffer, -1, CDC2_BUF_METER_X_POS,
                      (CDC1_STR_Y_POSITION + CDC_STR_Y_DIFF), true);


        /* Update the RX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's receive buffer is the UART's
           transmit buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sRxBuffer2) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, CDC2_BUF_METER_X_POS,
                           CDC2_BUF_METER_Y_POS);
    }


    /* Has there been any receive traffic since we last checked? */

    if(ulRxCount1 != g_ulUARTRxCount1)
    {

        /* Take a snapshot of the latest receive count. */

        ulRxCount1 = g_ulUARTRxCount1;


        /* Update the display of bytes received by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulRxCount1);
        GrStringDraw(&g_sContext, pcBuffer, -1, CDC1_BUF_METER_X_POS,
                      (CDC1_STR_Y_POSITION + (CDC_STR_Y_DIFF * 4)), true);


        /* Update the TX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's transmit buffer is the UART's
           receive buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sTxBuffer1) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, CDC1_BUF_METER_X_POS,
                      (CDC2_BUF_METER_Y_POS + CDC_BUF_METER_Y_DIFF));
    }

    /* Has there been any receive traffic since we last checked? */

    if(ulRxCount2 != g_ulUARTRxCount2)
    {

        /* Take a snapshot of the latest receive count. */

        ulRxCount2 = g_ulUARTRxCount2;


        /* Update the display of bytes received by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulRxCount2);
        GrStringDraw(&g_sContext, pcBuffer, -1, CDC2_BUF_METER_X_POS,
                     (CDC1_STR_Y_POSITION + (CDC_STR_Y_DIFF * 4)), true);


        /* Update the TX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's transmit buffer is the UART's
           receive buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sTxBuffer2) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, CDC2_BUF_METER_X_POS,
                          (CDC2_BUF_METER_Y_POS + CDC_BUF_METER_Y_DIFF));
    }
}

