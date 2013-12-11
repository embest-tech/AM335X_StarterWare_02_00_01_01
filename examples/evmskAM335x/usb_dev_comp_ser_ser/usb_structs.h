//****************************************************************************
//
// usb_structs.h - Data structures defining the two CDC serial USB device.
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
//Input clock to LCD module
#define LCD_CLK                  150000000
#define LCD_P_CLK                23040000
#define LCD_MODULE_CLK           192000000
#define LCD_NUM_OF_LINE_CLK      255
#define HSYNC_WIDTH             4
#define H_FRONT_PORCH           8
#define H_BACK_PORCH            43
#define LINES_PER_PANEL         272
#define VSYNC_WIDTH             10
#define V_FRONT_PORCH           4
#define V_BACK_PORCH            12
#define LCDC_FIFO_DELAY         128

#define PALETTE_SIZE             32
#define LCD_WIDTH                480
#define LCD_HEIGHT               272
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


//****************************************************************************
//
// The size of the transmit and receive buffers used for the redirected UART.
// This number should be a power of 2 for best performance.  256 is chosen
// pretty much at random though the buffer should be at least twice the size
// of a maximum-sized USB packet.
//
//****************************************************************************
#define UART_BUFFER_SIZE         256

#define TEXT_FONT               &g_sFontCmss22b
#define TEXT_HEIGHT             (GrFontHeightGet(TEXT_FONT))
#define BUFFER_METER_HEIGHT     TEXT_HEIGHT
#define BUFFER_METER_WIDTH      75

#define MAX_ROW_NUM             24
#define CDC1_STR_X_POSITION     8
#define CDC1_STR_Y_POSITION     100
#define CDC2_STR_X_POSITION     250
#define CDC2_STR_Y_POSITION     100
#define CDC1_BUF_METER_X_POS    150
#define CDC1_BUF_METER_Y_POS    155
#define CDC2_BUF_METER_X_POS    392
#define CDC2_BUF_METER_Y_POS    155
#define CDC_STR_Y_DIFF          30
#define CDC_BUF_METER_Y_DIFF    90

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
extern void DrawBufferMeter(tContext *psContext, int lX, int lY);
extern void DisplayStatus(tContext *psContext, char *pcStatus);
extern void SerialInit(void);
extern void SerialMain(void);

#endif
