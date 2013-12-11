//*****************************************************************************
//
// usb_dev_serial.c - Main routines for the USB CDC serial example.
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
#include "usb_structs.h"
#include "delay.h"
#include "cp15.h"
#include "mmu.h"

/*****************************************************************************
*
* Variables tracking transmit and receive counts.
*
*****************************************************************************/

volatile unsigned int g_ulUARTTxCount = 0;
volatile unsigned int g_ulUARTRxCount = 0;

/*****************************************************************************
*
* Flag indicating whether or not we are currently sending a Break condition.
*
*****************************************************************************/
static tBoolean g_bSendingBreak = false;

/*****************************************************************************
*
*  Status variable used by the composite main routine to display the status of
*  the device.
*
******************************************************************************/

char *g_pcStatus;

/*****************************************************************************
*
* Defines required to redirect UART0 via USB.
*
*****************************************************************************/

#define USB_UART_BASE           SOC_UART_0_REGS

/*****************************************************************************
*
* Default line coding settings for the redirected UART.
*
*****************************************************************************/

#define DEFAULT_BIT_RATE        (115200)
#define BAUD_RATE_115200        (115200)

/*****************************************************************************
*
* UART module i/p clock.
*
*****************************************************************************/

#define UART_MODULE_INPUT_CLK     (48000000)

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



/*****************************************************************************
*
* Internal function prototypes.
*
*****************************************************************************/

static void USBUARTPrimeTransmit(unsigned int ulBase);
static void CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, int lErrors);
static void SetControlLineState(unsigned short usState);
static tBoolean SetLineCoding(tLineCoding *psLineCoding);
static void GetLineCoding(tLineCoding *psLineCoding);
static void SendBreak(tBoolean bSend);

/******************************************************************************
**
** This function is called whenever serial data is received from the UART.
** It is passed the accumulated error flags from each character received in
** this interrupt and determines from them whether or not an interrupt
** notification to the host is required.
**
** If a notification is required and the control interrupt endpoint is idle,
** we send the notification immediately.  If the endpoint is not idle, we
** accumulate the errors in a global variable which will be checked on
** completion of the previous notification and used to send a second one
** if necessary.
*****************************************************************************/

static void CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, int lErrors)
{
    unsigned short usSerialState;


    /* Clear our USB serial state.  Since we are faking the handshakes, always
       set the TXCARRIER (DSR) and RXCARRIER (DCD) bits.
    */
    usSerialState = USB_CDC_SERIAL_STATE_TXCARRIER |
                    USB_CDC_SERIAL_STATE_RXCARRIER;


    /* Are any error bits set? */

    if(lErrors)
    {

        /* At least one error is being notified so translate from our hardware
           error bits into the correct state markers for the USB notification.
        */
        if(lErrors & UART_OVERRUN_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_OVERRUN;
        }

        if(lErrors & UART_PARITY_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_PARITY;
        }

        if(lErrors & UART_FRAMING_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_FRAMING;
        }

        if(lErrors & UART_BREAK_DETECTED_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_BREAK;
        }

       /* Call the CDC driver to notify the state change. */
        USBDCDCSerialStateChange((void *)psDevice, usSerialState);
    }
}

/******************************************************************************
*                                                                             *
* \brief  A wrapper function performing FIFO configurations.\n                *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void UartFIFOConfigure(void)
{
    unsigned int fifoConfig = 0;

    /* Setting the TX and RX FIFO Trigger levels as 1. No DMA enabled. */
    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  1,
                                  1,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_0_ENABLE);

    /* Configuring the FIFO settings. */
    UARTFIFOConfig(SOC_UART_0_REGS, fifoConfig);
}

/******************************************************************************
*                                                                             *
* \brief  A wrapper function performing Baud Rate settings.\n                 *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void UartBaudRateSet(void)
{
    unsigned int divisorValue = 0;

    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         BAUD_RATE_115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(SOC_UART_0_REGS, divisorValue);
}

/******************************************************************************
*                                                                             *
* \brief  Read as many characters from the UART FIFO as we can and move the   *
*         into the CDC transmit buffer.\n                                     *
*                                                                             *
* \param none.                                                                *
*                                                                             *
* \return UART error flags read during data reception.                        *
*                                                                             *
******************************************************************************/
static int ReadUARTData(void)
{
    int lChar, lErrors;
    unsigned char ucChar;
    unsigned int ulSpace;


    /* Clear our error indicator. */

    lErrors = 0;


    /* How much space do we have in the buffer? */

    ulSpace = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);


    /* Read data from the UART FIFO until there is none left or we run
       out of space in our receive buffer.
    */
    while(ulSpace && UARTCharsAvail(USB_UART_BASE))
    {

        /* Read a character from the UART FIFO into the ring buffer if no
           errors are reported.
        */
        lChar = UARTCharGetNonBlocking(USB_UART_BASE);


        /* If the character did not contain any error notifications,
           copy it to the output buffer.
        */
        if(!(lChar & ~0xFF))
        {
            ucChar = (unsigned char)(lChar & 0xFF);
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                           (unsigned char *)&ucChar, 1);


            /* Decrement the number of bytes we know the buffer can accept. */

            ulSpace--;
        }
        else
        {

           /* Update our error accumulator. */

           lErrors |= lChar;
        }

        /* Update our count of bytes received via the UART. */

        g_ulUARTRxCount++;
    }

    /* Pass back the accumulated error indicators. */

    return(lErrors);
}

/******************************************************************************
*                                                                             *
* \brief  Take as many bytes from the transmit buffer as we have space for and*
*         move them into the USB UART's transmit FIFO. \n                     *
*                                                                             *
* \param ulBase  Base address of the UART instance.\n                         *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void USBUARTPrimeTransmit(unsigned int ulBase)
{
    unsigned int ulRead;
    unsigned char ucChar;


    /* If we are currently sending a break condition, don't receive any
       more data. We will resume transmission once the break is turned off.
    */
    if(g_bSendingBreak)
    {
        return;
    }


    /* If there is space in the UART FIFO, try to read some characters
       from the receive buffer to fill it again.
    */
    while(UARTSpaceAvail(ulBase))
    {

        /* Get a character from the buffer. */

        ulRead = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);


        /* Did we get a character? */

        if(ulRead)
        {

            /* Place the character in the UART transmit FIFO. */

            UARTCharPutNonBlocking(ulBase, ucChar);


            /* Update our count of bytes transmitted via the UART. */

            g_ulUARTTxCount++;
        }
        else
        {
            /* We ran out of characters so exit the function. */

            return;
        }
    }
}

/******************************************************************************
*                                                                             *
* \brief  UART error handler.\n                                               *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void UARTHandleError(void)
{
    while((UARTRxErrorGet(USB_UART_BASE)))
    {
        /* Read a byte from the RBR if RBR has data.*/
       UARTCharGetNonBlocking(USB_UART_BASE);
    }
}

/******************************************************************************
*                                                                             *
* \brief  Interrupt handler for the UART which we are redirecting via USB.\n  *
*                                                                             *
* \param none.\n                                                              *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
void USBUARTIntHandler(void)
{
    unsigned int ulInts;
    int lErrors;

    /* Get and clear the current interrupt source(s) */

    ulInts = UARTIntIdentityGet(SOC_UART_0_REGS);



        /* Check if the cause is receiver line error condition.*/
        if( UART_INTID_RX_LINE_STAT_ERROR == ulInts || UART_INTID_CHAR_TIMEOUT == ulInts)
        {
            UARTHandleError();
        }

        /* Are we being interrupted because the TX FIFO has space available? */

        if(ulInts == UART_INTID_TX_THRES_REACH)
        {
            /* Move as many bytes as we can into the transmit FIFO. */

            USBUARTPrimeTransmit(USB_UART_BASE);

            /* If the output buffer is empty, turn off the transmit interrupt.*/

            if(!USBBufferDataAvailable(&g_sRxBuffer))
            {
                UARTIntDisable(USB_UART_BASE, UART_INTID_TX_THRES_REACH);
            }
        }

        /* Handle receive interrupts. */

        if(ulInts  == UART_INTID_RX_THRES_REACH)
        {
            /* Read the UART's characters into the buffer. */

            lErrors = ReadUARTData();

            /* Check to see if we need to notify the host of any errors we just
               detected.
            */
            CheckForSerialStateChange(&g_sCDCDevice, lErrors);
        }
}

/******************************************************************************
*                                                                             *
* \brief  Set the state of the RS232 RTS and DTR signals.\n                   *
*                                                                             *
* \param usState  State of hte GPIO line.\n                                   *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void SetControlLineState(unsigned short usState)
{

    /* TODO: If configured with GPIOs controlling the handshake lines,
       set them appropriately depending upon the flags passed in the wValue
       field of the request structure passed.
    */
}

/******************************************************************************
*                                                                             *
* \brief  Set the communication parameters to use on the UART.\n              *
*                                                                             *
* \param psLineCoding  pointer tLineCoding struct.\n                          *
*                                                                             *
* \return Status of set communication params.\n                               *
*                                                                             *
******************************************************************************/
static tBoolean SetLineCoding(tLineCoding *psLineCoding)
{
    unsigned int ulConfig, ulParity;
    tBoolean bRetcode;


    /* Assume everything is OK until we detect any problem. */

    bRetcode = true;


    /* Word length.  For invalid values, the default is to set 8 bits per
       character and return an error.
    */
    switch(psLineCoding->ucDatabits)
    {
        case 5:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_5;
            break;
        }

        case 6:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_6;
            break;
        }

        case 7:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_7;
            break;
        }

        case 8:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_8;
            break;
        }

        default:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_8;
            bRetcode = false;
            break;
        }
    }


    /* Parity.  For any invalid values, we set no parity and return an error. */

    switch(psLineCoding->ucParity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ulParity = UART_PARITY_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ulParity = UART_ODD_PARITY;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ulParity = UART_EVEN_PARITY;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
            ulParity = UART_PARITY_REPR_1;
            break;
        }

        case USB_CDC_PARITY_SPACE:
        {
            ulParity = UART_PARITY_REPR_0;
            break;
        }

        default:
        {
            ulParity =  UART_PARITY_NONE;
            bRetcode = false;
            break;
        }
    }


    /* Stop bits.  Our hardware only supports 1 or 2 stop bits whereas CDC
       allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
       invalid or unsupported value of ucStop, we set up for 1 stop bit but
       return an error in case the caller needs to Stall or otherwise report
       this back to the host.
    */
    switch(psLineCoding->ucStop)
    {

        /* One stop bit requested. */

        case USB_CDC_STOP_BITS_1:
        {
            ulConfig |= UART_FRAME_NUM_STB_1;
            break;
        }


        /* Two stop bits requested. */

        case USB_CDC_STOP_BITS_2:
        {
            ulConfig |= UART_FRAME_NUM_STB_1_5_2;
            break;
        }


        /* Other cases are either invalid values of ucStop or values that we
           cannot support so set 1 stop bit but return an error.
        */
        default:
        {
            ulConfig = UART_FRAME_NUM_STB_1;
            bRetcode |= false;
            break;
        }
    }


    /* Set the UART mode appropriately. */
    UARTLineCharacConfig(SOC_UART_0_REGS, ulConfig, ulParity);


    /* Let the caller know if we had a problem or not. */

    return(bRetcode);
}

/******************************************************************************
*                                                                             *
* \brief  Get the communication parameters in use on the UART.\n              *
*                                                                             *
* \param psLineCoding  pointer tLineCoding struct.\n                          *
*                                                                             *
* \return Communication params.\n                                             *
*                                                                             *
******************************************************************************/
static void GetLineCoding(tLineCoding *psLineCoding)
{

    /* Get the current line coding set in the UART. */

    psLineCoding->ulRate = DEFAULT_BIT_RATE;
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;

}

/******************************************************************************
*                                                                             *
* \brief  This function sets or clears a break condition on the redirected    *
*         UART RX line.  A break is started when the function is called with  *
*         bSend set to true and persists until the function is called again   *
*         with \e bSend set to \b false.\n                                    *
*                                                                             *
* \return none.\n                                                             *
*                                                                             *
******************************************************************************/
static void SendBreak(tBoolean bSend)
{

    /* Are we being asked to start or stop the break condition? */

    if(!bSend)
    {

        /* Remove the break condition on the line. */

        UARTBreakCtl(USB_UART_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {

        /* Start sending a break condition on the line. */

        UARTBreakCtl(USB_UART_BASE, true);
        g_bSendingBreak = true;
    }
}

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


        /* Return the current serial communication parameters. */

        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;


        /* Set the current serial communication parameters. */

        case USBD_CDC_EVENT_SET_LINE_CODING:
            SetLineCoding(pvMsgData);
            break;

        /* Set the current serial communication parameters. */

        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            SetControlLineState((unsigned short)ulMsgValue);
            break;

        /* Send a break condition on the serial line.*/

        case USBD_CDC_EVENT_SEND_BREAK:
            SendBreak(true);
            break;

        /* Clear the break condition on the serial line. */

        case USBD_CDC_EVENT_CLEAR_BREAK:
            SendBreak(false);
            break;

        /* Ignore SUSPEND and RESUME for now. */

        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
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
* \brief  Handles CDC driver notifications related to the transmit  channel   *
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
unsigned int
TxHandler(void *pvCBData, unsigned int ulEvent, unsigned int ulMsgValue,
          void *pvMsgData)
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
* \brief  Handles CDC driver notifications related to the receive   channel   *
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
unsigned int RxHandler(void *pvCBData, unsigned int ulEvent,
                       unsigned int ulMsgValue, void *pvMsgData)
{

    /* Which event are we being sent? */

    switch(ulEvent)
    {

        /* A new packet has been received. */

        case USB_EVENT_RX_AVAILABLE:
        {

            /* Feed some characters into the UART TX FIFO and enable the
               interrupt so we are told when there is more space.
            */
              USBUARTPrimeTransmit(USB_UART_BASE);
              UARTIntEnable(USB_UART_BASE, UART_INTID_TX_THRES_REACH);
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

        /*
           We are being asked to provide a buffer into which the next packet
           can be read. We do not support this mode of receiving data so let
           the driver know by returning 0. The CDC driver should not be sending
           this message but this is included just for illustration and
           completeness.
        */
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        /*
           We don't expect to receive any other events.  Ignore any that show
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
    unsigned int intFlags = 0;



    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Performing the Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Performing a module reset. */
    UARTModuleReset(SOC_UART_0_REGS);


    /* Performing Baud Rate settings. */
    UartBaudRateSet();

   /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(SOC_UART_0_REGS, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(SOC_UART_0_REGS);

    /* Disabling Break Control. */
    UARTBreakCtl(SOC_UART_0_REGS, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(SOC_UART_0_REGS, UART16x_OPER_MODE);

     /* Performing FIFO configurations. */
    UartFIFOConfigure();

    /* Preparing the 'intFlags' variable to be passed as an argument.*/
    intFlags |= (UART_INT_LINE_STAT | UART_INT_RHR_CTI);

    /* Enable the Interrupts in UART.*/
    UARTIntEnable(SOC_UART_0_REGS, intFlags);

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
    static unsigned int ulTxCount = 0;
    static unsigned int ulRxCount = 0;
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

    if(ulTxCount != g_ulUARTTxCount)
    {
        /* Take a snapshot of the latest transmit count. */

        ulTxCount = g_ulUARTTxCount;

        /* Update the display of bytes transmitted by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulTxCount);
        GrStringDraw(&g_sContext, pcBuffer, -1, BUFFER_METER_X_POS,
                      CDC_STR_Y_POSITION, true);

        /*
           Update the RX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's receive buffer is the UART's
           transmit buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sRxBuffer) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, BUFFER_METER_X_POS,
                          BUFFER_METER_Y_POS);
    }

    /* Has there been any receive traffic since we last checked? */

    if(ulRxCount != g_ulUARTRxCount)
    {
        /* Take a snapshot of the latest receive count. */

        ulRxCount = g_ulUARTRxCount;

        /* Update the display of bytes received by the UART. */

        GrContextFontSet(&g_sContext, TEXT_FONT);
        usnprintf(pcBuffer, 16, "%d ", ulRxCount);
        GrStringDraw(&g_sContext, pcBuffer, -1, BUFFER_METER_X_POS,
                      (CDC_STR_Y_POSITION + CDC_STR_Y_DIFF * 3), true);

        /*
           Update the TX buffer fullness. Remember that the buffers are
           named relative to the USB whereas the status display is from
           the UART's perspective. The USB's transmit buffer is the UART's
           receive buffer.
        */
        ulFullness = ((USBBufferDataAvailable(&g_sTxBuffer) * 100) /
                      UART_BUFFER_SIZE);
        UpdateBufferMeter(&g_sContext, ulFullness, BUFFER_METER_X_POS,
                           (BUFFER_METER_Y_POS + CDC_BUF_METER_Y_DIFF));
    }
}
