//*****************************************************************************
//
// usbdata.c - Data definitions related to USB stack.
//
// Copyright (c) 2008-2010 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of AM1808 Sitaraware USB Library and reused from revision 6288 
// of the  Stellaris USB Library.
//
//*****************************************************************************

#include "hw_types.h"
#include "usblib.h"

//*****************************************************************************
//
// Flag to indicate whether or not we have been initialized.
//
//*****************************************************************************
tUSBInstanceObject g_USBInstance[USB_NUM_INSTANCE];

tUSBPerfInfo g_USBPerfInfo[5000];
unsigned int ulPerfInfoCounter=0;
unsigned int fReadEnabled=0;
unsigned int fWriteEnabled=0;
unsigned int fUSBDisconnected = 0;


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
