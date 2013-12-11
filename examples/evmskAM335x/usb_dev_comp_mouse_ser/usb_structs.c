//****************************************************************************
//
// usb_structs.c - Data structures defining the composite HID mouse and CDC
// serial USB device.
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

#include "hw_types.h"
#include "usblib.h"
#include "usbhid.h"
#include "usb-ids.h"
#include "usbcdc.h"
#include "usbdevice.h"
#include "usbdcomp.h"
#include "usbdcdc.h"
#include "usbdhid.h"
#include "usbdhidmouse.h"
#include "grlib.h"
#include "usb_structs.h"

/****************************************************************************
*
* The languages supported by this device.
*
****************************************************************************/
const unsigned char g_pLangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

/*****************************************************************************
*
*  The manufacturer string.
*
*****************************************************************************/
const unsigned char g_pManufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
    't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

/*****************************************************************************
*
*  The product string.
*
******************************************************************************/
const unsigned char g_pProductString[] =
{
    (42 + 1) * 2,
    USB_DTYPE_STRING,
    'C', 0, 'o', 0, 'm', 0, 'p', 0, 'o', 0, 's', 0, 'i', 0, 't', 0,
    'e', 0, ' ', 0, 'H', 0, 'I', 0, 'D', 0, ' ', 0, 'M', 0, 'o', 0,
    'u', 0, 's', 0, 'e', 0, ' ', 0, 'a', 0, 'n', 0, 'd', 0, ' ', 0,
    'C', 0, 'D', 0, 'C', 0, ' ', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0,
    'a', 0, 'l', 0, ' ', 0, 'E', 0, 'x', 0, 'a', 0, 'm', 0, 'p', 0,
    'l', 0, 'e', 0,
};

/*****************************************************************************
*
*  The serial number string.
*
******************************************************************************/
const unsigned char g_pSerialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/*****************************************************************************
*
*  The descriptor string table.
*
******************************************************************************/
const unsigned char * const g_pStringDescriptors[] =
{
    g_pLangDescriptor,
    g_pManufacturerString,
    g_pProductString,
    g_pSerialNumberString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_pStringDescriptors) /               \
                                sizeof(unsigned char *))

/*****************************************************************************
*
*  External USB buffer pointer used by the composite main routine.
*
******************************************************************************/
extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;

/****************************************************************************
*
* The number of individual device class instances comprising this composite
* device.
*
****************************************************************************/

#define   NUM_DEVICES            2

/****************************************************************************
*
* The HID mouse device initialization and customization structures.
*
****************************************************************************/

tHIDMouseInstance g_sMouseInstance;

const tUSBDHIDMouseDevice g_sMouseDevice =
{

    /* Stellaris VID. */

    USB_VID_STELLARIS,

    /* Stellaris HID Mouse PID. */

    USB_PID_MOUSE,

    /* This is in 2mA increments so 500mA. */

    250,

    /* Self powered device. */

    USB_CONF_ATTR_SELF_PWR,

    /* The Mouse handler function. */

    MouseHandler,

    /* Point to the mouse device structure. */

    (void *)&g_sMouseDevice,

    /* The composite device does not use the strings from the class. */

    0,
    0,

    /* The instance data for this mouse device. */

    &g_sMouseInstance
};

/****************************************************************************
*
* The CDC device initialization and customization structures. In this case,
* we are using USBBuffers between the CDC device class driver and the
* application code. The function pointers and callback data values are set
* to insert a buffer in each of the data channels, transmit and receive.
* With the buffer in place, the CDC channel callback is set to the relevant
* channel function and the callback data is set to point to the channel
* instance data. The buffer, in turn, has its callback set to the application
* function and the callback data set to our CDC instance structure.
****************************************************************************/
tCDCSerInstance g_sCDCInstance;

const tUSBDCDCDevice g_sCDCDevice =
{

    /* Stellaris VID. */

    USB_VID_STELLARIS,

    /* Stellaris Virtual Serial Port PID.*/

    USB_PID_SERIAL,

    0,

    /* Self powered device.*/

    USB_CONF_ATTR_SELF_PWR,

    /* Serial device information callback function. */

    ControlHandler,

    /* The CDC Serial device1 information. */

    (void *)&g_sCDCDevice,

    /* Receive buffer. */

    USBBufferEventCallback,
    (void *)&g_sRxBuffer,

    /* Transmit buffer. */

    USBBufferEventCallback,
    (void *)&g_sTxBuffer,

    /* The composite device does not use the strings from the class. */
    0,
    0,

    /* The serial instance data for this device. */
    &g_sCDCInstance
};

/****************************************************************************
*
* Receive buffers.
*
****************************************************************************/
unsigned char g_pcUSBRxBuffer[UART_BUFFER_SIZE];
unsigned char g_pucRxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sRxBuffer =
{
    false,                          // This is a receive buffer.
    RxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pcUSBRxBuffer,                // pcBuffer
    UART_BUFFER_SIZE,               // ulBufferSize
    g_pucRxBufferWorkspace          // pvWorkspace
};

/****************************************************************************
*
* Transmit buffers.
*
****************************************************************************/
unsigned char g_pcUSBTxBuffer[UART_BUFFER_SIZE];
unsigned char g_pucTxBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sTxBuffer =
{
    true,                           // This is a transmit buffer.
    TxHandler,                      // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketWrite,             // pfnTransfer
    USBDCDCTxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pcUSBTxBuffer,                // pcBuffer
    UART_BUFFER_SIZE,               // ulBufferSize
    g_pucTxBufferWorkspace          // pvWorkspace
};

/****************************************************************************
*
* The array of devices supported by this composite device.
*
****************************************************************************/
tCompositeEntry g_psCompDevices[NUM_DEVICES]=
{

    /* HID Mouse Information.*/

    {
        &g_sHIDDeviceInfo,
        0
    },

    /* Serial Device Instance. */

    {
        &g_sCDCSerDeviceInfo,
        0
    }
};

/****************************************************************************
*
* Additional workspaced required by the composite device.
*
****************************************************************************/
unsigned int g_pulCompWorkspace[NUM_DEVICES];

/****************************************************************************
*
* The instance data for this composite device.
*
****************************************************************************/
tCompositeInstance g_CompInstance;

/****************************************************************************
*
* Allocate the Device Data for the top level composite device class.
*
****************************************************************************/
tUSBDCompositeDevice g_sCompDevice =
{

    /* Stellaris VID. */

    USB_VID_STELLARIS,

    /* Stellaris PID for composite serial device. */

    USB_PID_COMP_HID_SER,

    /* This is in 2mA increments so 500mA. */

    500,

    /* Bus powered device. */

    USB_CONF_ATTR_BUS_PWR,

    /* Default composite event handler. */

    EventHandler,

    /* The string table. */

    g_pStringDescriptors,
    NUM_STRING_DESCRIPTORS,


    /* The Composite device array. */

    NUM_DEVICES,

    g_psCompDevices,

    &g_CompInstance
};
