//****************************************************************************
//
// usb_structs.h - Data structures defining the two CDC serial USB device.
//
// Copyright (c) 2010-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 8555 of the EK-LM3S9B92 Firmware Package.
//
//****************************************************************************

#ifndef __USB_STRUCTS_H__
#define __USB_STRUCTS_H__


/*****************************************************************************
*
*  External USB buffer pointer used by the composite main routine.
*
******************************************************************************/

extern const tUSBBuffer g_sTxBuffer1;
extern const tUSBBuffer g_sRxBuffer1;
extern const tUSBBuffer g_sTxBuffer2;
extern const tUSBBuffer g_sRxBuffer2;

/*****************************************************************************
*
*  External CDC serial definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDCDCDevice g_sCDCDevice1;
extern const tUSBDCDCDevice g_sCDCDevice2;



#define COMMAND_STATUS_UPDATE   0x00000002

#define DESCRIPTOR_DATA_SIZE    (COMPOSITE_DCDC_SIZE + COMPOSITE_DCDC_SIZE)


//****************************************************************************
//
// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size
// of a maximum-sized USB packet.
//
//****************************************************************************
#define UART_BUFFER_SIZE         256

#define START_ADDR_DDR           (0x80000000)
#define START_ADDR_DEV           (0x44000000)
#define START_ADDR_OCMC          (0x40300000)
#define NUM_SECTIONS_DDR         (512)
#define NUM_SECTIONS_DEV         (960)
#define NUM_SECTIONS_OCMC        (1)


//****************************************************************************
//
// CDC device callback function prototypes.
//
//****************************************************************************
extern unsigned int RxHandler1(void *pvCBData, unsigned int ulEvent,
                               unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int RxHandler2(void *pvCBData, unsigned int ulEvent,
                               unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int TxHandler1(void *pvCBData, unsigned int ulEvent,
                               unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int TxHandler2(void *pvCBData, unsigned int ulEvent,
                               unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int EventHandler(void *pvCBData, unsigned int ulEvent,
                                 unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int ControlHandler(void *pvCBData, unsigned int ulEvent,
                                   unsigned int ulMsgValue, void *pvMsgData);
extern void SerialInit(void);
extern void SerialMain(void);

#endif
