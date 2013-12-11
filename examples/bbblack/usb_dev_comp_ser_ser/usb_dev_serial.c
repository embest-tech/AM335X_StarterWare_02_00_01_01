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
#include "beaglebone.h"
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
    unsigned char Intstatus;

    /* Have we been asked to update the status display? */

    if(g_ulFlags & COMMAND_STATUS_UPDATE)
    {

        /* Clear the command flag */

        Intstatus = IntDisable();
        g_ulFlags &= ~COMMAND_STATUS_UPDATE;
        IntEnable(Intstatus);
    }
}

