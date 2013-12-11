//****************************************************************************
//
// usb_structs.h - Data structures defining the composite HID mouse and CDC
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


#ifndef __USB_STRUCTS_H__
#define __USB_STRUCTS_H__

/*****************************************************************************
*
*  External USB buffer pointer used by the composite main routine.
*
******************************************************************************/

extern const tUSBBuffer g_sTxBuffer;
extern const tUSBBuffer g_sRxBuffer;

/*****************************************************************************
*
*  External CDC serial definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDCDCDevice g_sCDCDevice;


/****************************************************************************
*
* Globals used by both classes.
*
****************************************************************************/

#define COMMAND_STATUS_UPDATE   0x00000002

#define DESCRIPTOR_DATA_SIZE    (COMPOSITE_DCDC_SIZE + COMPOSITE_DHID_SIZE)
/*  Input clock to LCD module */
#define LCD_CLK                  150000000
#define PALETTE_SIZE             32
#define LCD_WIDTH                800
#define LCD_HEIGHT               480
#define PALETTE_OFFSET           4
#define PIXEL_24_BPP_PACKED     (0x0)
#define PIXEL_24_BPP_UNPACKED   (0x1)
#define FRAME_BUFFER_0  0
#define FRAME_BUFFER_1  1
#define START_ADDR_DDR           (0x80000000)
#define START_ADDR_DEV           (0x44000000)
#define START_ADDR_OCMC          (0x40300000)
#define NUM_SECTIONS_DDR         (512)
#define NUM_SECTIONS_DEV         (960)
#define NUM_SECTIONS_OCMC        (1)
/* Used to set the mode of PMIC asread pressure mode */
#define MODE_PRESSURE            2
/* Numer of pressure reads that going to perform.  This is required to
   avoid the debouncing effect
*/
#define DEBOUNCE_READ            50


/*****************************************************************************
*
* The height of the mouse button bar at the bottom of the display and the
* number of buttons it contains.
*
*****************************************************************************/
#define BUTTON_HEIGHT              50
#define NUM_MOUSE_BUTTONS          3


/****************************************************************************
*
* The size of the transmit and receive buffers used for the redirected UART.
* This number should be a power of 2 for best performance.  256 is chosen
* pretty much at random though the buffer should be at least twice the size
* of a maximum-sized USB packet.
*
****************************************************************************/
#define UART_BUFFER_SIZE         256

#define TEXT_FONT               &g_sFontCmss22b
#define TEXT_HEIGHT             (GrFontHeightGet(TEXT_FONT))
#define BUFFER_METER_HEIGHT     TEXT_HEIGHT
#define BUFFER_METER_WIDTH      150

#define    MAX_ROW_NUM          24
#define    CDC_STR_X_POSITION   8
#define    CDC_STR_Y_POSITION   80
#define    CDC_STR_Y_DIFF       25
#define    CDC_BUF_METER_Y_DIFF 75

#define    BUFFER_METER_X_POS   150
#define    BUFFER_METER_Y_POS   105

/****************************************************************************
*
* CDC amd mouse device callback function prototypes.
*
****************************************************************************/
extern unsigned int RxHandler(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int TxHandler(void *pvCBData, unsigned int ulEvent,
                        unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int EventHandler(void *pvCBData, unsigned int ulEvent,
                             unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int MouseHandler(void *pvCBData, unsigned int ulEvent,
                                  unsigned int ulMsgData, void *pvMsgData);
extern unsigned int SerialHandler(void *pvCBData, unsigned int ulEvent,
                                   unsigned int ulMsgValue, void *pvMsgData);
extern unsigned int ControlHandler(void *pvCBData, unsigned int ulEvent,
               unsigned int ulMsgValue, void *pvMsgData);
extern void USBUARTIntHandler(void);
extern void DrawBufferMeter(tContext *psContext, int lX, int lY);
extern void DisplayStatus(tContext *psContext, char *pcStatus);
extern void PeripheralsSetup(void);
extern void UpdateDisplay(unsigned char ucButtons, tBoolean bRedraw);
extern void MouseMain(void);

extern void SerialInit(void);
extern void SerialMain(void);

#endif
