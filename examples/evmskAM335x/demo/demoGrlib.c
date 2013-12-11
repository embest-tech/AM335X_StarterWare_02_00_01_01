/**
 * \file  demoGrlib.c
 *
 * \brief Demo content using graphics library
 *
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*
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
*/

#include "grlib.h"
#include "widget.h"
#include "pushbutton.h"
#include "slider.h"
#include "demoGrlib.h"
#include "demoMain.h"
#include "demoRaster.h"
#include "demoCfg.h"
#include "cache.h"
#include "demoI2c.h"
#include "delay.h"
#include "device.h"
#include "demoPwrMgmnt.h"
#include "demoDvfs.h"
#include "hw_types.h"

#include "icons.h"

#include <string.h>
#include <stdlib.h>

#define BPP	32

#define SNAKE_CELL_SIZE		(10)
#define SNAKE_MIDDLE_X_MIN	(237)
#define SNAKE_MIDDLE_Y_MIN	(176)
#define SNAKE_MIDDLE_X_MAX	(SNAKE_MIDDLE_X_MIN + SNAKE_CELL_SIZE)
#define SNAKE_MIDDLE_Y_MAX	(SNAKE_MIDDLE_Y_MIN + SNAKE_CELL_SIZE)
#define SNAKE_GAP			(3)
#define SNAKE_MOVE_FACTOR	(1)
#define SNAKE_MAX_DELAY_MS	(180)
#define SNAKE_NO_CELLS		(15)


/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
// Memory that is used as the local frame buffer.
unsigned char g_pucBuffer[2][GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];

unsigned char baseUnCompImage[(LCD_WIDTH*LCD_HEIGHT*(BPP/8))+PALETTE_SIZE];

unsigned int printtemp = 1;

// The graphics library display structure.
tDisplay g_s35_480x272x24Display[2];

tContext sContext[2];
static tRectangle acceleroRect = {90, 112, 390, 246};
static tRectangle rectSnake[SNAKE_NO_CELLS] = 
	{
		{SNAKE_MIDDLE_X_MIN - SNAKE_GAP - SNAKE_CELL_SIZE, 
		 SNAKE_MIDDLE_Y_MIN, 
		 SNAKE_MIDDLE_X_MAX - SNAKE_GAP - SNAKE_CELL_SIZE, 
		 SNAKE_MIDDLE_Y_MAX
		},
		{SNAKE_MIDDLE_X_MIN, SNAKE_MIDDLE_Y_MIN, SNAKE_MIDDLE_X_MAX, SNAKE_MIDDLE_Y_MAX},
		{SNAKE_MIDDLE_X_MIN + SNAKE_GAP + SNAKE_CELL_SIZE, 
		 SNAKE_MIDDLE_Y_MIN, 
		 SNAKE_MIDDLE_X_MAX + SNAKE_GAP + SNAKE_CELL_SIZE, 
		 SNAKE_MIDDLE_Y_MAX
		},
		{SNAKE_MIDDLE_X_MIN + 2*(SNAKE_GAP + SNAKE_CELL_SIZE), 
		 SNAKE_MIDDLE_Y_MIN, 
		 SNAKE_MIDDLE_X_MAX + 2*(SNAKE_GAP + SNAKE_CELL_SIZE), 
		 SNAKE_MIDDLE_Y_MAX
		},
	};

extern volatile unsigned int frameBufIdx;
extern int const xyDefault[4];
extern int const xyNext[4];
extern int const xyPrev[4];
extern int const xyHome[4];
extern int const xyTimeSet[4];
extern int const xyIntro[4];
extern int const xyWebDemo[4];
extern int const xyMcASP[4];
extern int const xyUart[4]; 
extern int const xyRTC[4]; 
extern int const xyTimer[4];
extern int const xyEthernet[4]; 
extern int const xyMMCSD[4]; 
extern int const xyEcapDemo[4]; 
extern int const xyEcapMenu[4]; 
extern int const xyGpioMenu[4]; 
extern int const xyBuzzDemo[4];
extern int const xyI2CMenu[4]; 
extern int const xyPMMenu[4];
extern int const xyPMstandbyDemo[4];
extern int const xyPMds0Demo[4]; 
extern int const xyPMds1Demo[4]; 
extern int const xyPMwksTsc[4]; 
extern int const xyPMwksTmr[4]; 
extern int const xyPMwksUart[4]; 
extern int const xyPMwksGpio[4]; 
extern int const xyPMwksRTC[4];
extern int const xyDVFSMenu[4];
extern int const xyDVFSOpp50[4];
extern int const xyDVFSOpp100[4];
extern int const xyDVFSOpp120[4];
extern int const xyDVFSSrTurbo[4];
extern int const xyDVFSNitro[4];

extern unsigned int wakeSource;

/*******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/*
** Initializes the Graphics library
*/
void GrlibInit(void)
{
	GrOffScreen24BPPInit(&g_s35_480x272x24Display[0], g_pucBuffer[0], LCD_WIDTH, LCD_HEIGHT);
	GrOffScreen24BPPInit(&g_s35_480x272x24Display[1], g_pucBuffer[1], LCD_WIDTH, LCD_HEIGHT);
	
	// Initialize a drawing context.
	GrContextInit(&sContext[0], &g_s35_480x272x24Display[0]);
	GrContextInit(&sContext[1], &g_s35_480x272x24Display[1]);
	
}


/*
** Update the display context with next slide content
*/
void updatePage(unsigned int demoIndex)
{
	tRectangle acceleroRect = {90, 112, 390, 246};
	
	switch(demoIndex)
	{
		case CLICK_IDX_MENU:
			// Draw menu icons
			GrImageDraw(&sContext[frameBufIdx], infoIcon,		xyIntro[0], 	xyIntro[2]);
			GrImageDraw(&sContext[frameBufIdx], webIcon, 		xyWebDemo[0], 	xyWebDemo[2]);
			GrImageDraw(&sContext[frameBufIdx], mcaspIcon, 		xyMcASP[0], 	xyMcASP[2]);
			GrImageDraw(&sContext[frameBufIdx], mmcsdIcon, 		xyMMCSD[0], 	xyMMCSD[2]);
			GrImageDraw(&sContext[frameBufIdx], uartIcon, 		xyUart[0], 	xyUart[2]);
			GrImageDraw(&sContext[frameBufIdx], rtcIcon, 		xyRTC[0], 	xyRTC[2]);
			GrImageDraw(&sContext[frameBufIdx], timerIcon, 		xyTimer[0], 	xyTimer[2]);
			GrImageDraw(&sContext[frameBufIdx], ethernetIcon,	xyEthernet[0],	xyEthernet[2]);  
			GrImageDraw(&sContext[frameBufIdx], eCAPIcon,		xyEcapMenu[0],	xyEcapMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], gpioIcon,		xyGpioMenu[0],	xyGpioMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], i2cIcon,		xyI2CMenu[0],	xyI2CMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], pmIcon,		xyPMMenu[0],	xyPMMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], dvfsIcon,		xyDVFSMenu[0],	xyDVFSMenu[2]);  
			
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
			
		case CLICK_IDX_INTRO:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
		 	GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
		      	GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
		    
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss18b);
			GrStringDrawCentered(&sContext[frameBufIdx], "-  A no OS platform support package for AM335x SoC         ", -1, 260, 42, 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "-  Provides device abstraction libraries for peripherals    ", -1, 233,
											(42 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   and board level example applications                     ", -1, 238, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 
			GrStringDrawCentered(&sContext[frameBufIdx], "-  Ensure the Ethernet port is connected to LAN if Ethernet ", -1, 250, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   shall be used for driving the demo                       ", -1, 238, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], "   Ensure the UART port onboard is connected to the UART    ", -1, 240, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 
			GrStringDrawCentered(&sContext[frameBufIdx], " communication port on the host and a serial communication", -1, 235, 
											(42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);				
			GrStringDrawCentered(&sContext[frameBufIdx], "   application is running (TeraTerm/HyperTerminal/minicom)  ", -1, 225, 
											(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);															
			GrStringDrawCentered(&sContext[frameBufIdx], "    on the host. All the input values are accepted through UART.", -1, 234, 
											(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);															
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_CHOICE:
			GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss16b);
			GrStringDrawCentered(&sContext[frameBufIdx], "The demo can be continued using Touch Screen and/or Ethernet", -1, 240,
											(42 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], "To control via Touch Screen press on next icon", -1, 228,
											(42+ (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);

			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_MCASP:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);
			GrStringDrawCentered(&sContext[frameBufIdx], "McASP (Multi-channel Audio Serial Port)", -1, 240, 42, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- General purpose audio serial port optimized for the ", -1, 260, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "  needs of multichannel audio applications            ", -1, 260, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Supports TDM stream, 125 protocols and DIT          ", -1, 260, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Seperate transmit and receive sections that         ", -1, 255, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], "  can operate synchronously or independently          ", -1, 260, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);		
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], " Ensure that a speaker/headphone is connected to the  ", -1, 260, 
											(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
			GrStringDrawCentered(&sContext[frameBufIdx], " LINE-OUT of EVM to listen to the background music    ", -1, 263, 
											(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_ETHERNET:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "Ethernet", -1, 240, 42, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- The ethernet switch is compliant to IEEE 802.3 standard", -1, 217, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- It has one host port and two slave ports, which are capable", -1, 232, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "of 10/100/1000 Mbps with MII/GMII/RMII/RGMII interfaces", -1, 230, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- The MDIO module implements the 802.3 serial management  ", -1, 234, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);			
			GrStringDrawCentered(&sContext[frameBufIdx], "interface to interrogate and control up to 32 Ethernet PHYs", -1, 240, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], "- LwIP is used for implementing IP stack in StarterWare", -1, 216, 
											(42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);			
			GrStringDrawCentered(&sContext[frameBufIdx], "- The device abstraction layer is glued by implementing the", -1, 225, 
											(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
			GrStringDrawCentered(&sContext[frameBufIdx], "interface layer of lwIP", -1, 118, 
											(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);		
			GrStringDrawCentered(&sContext[frameBufIdx], "- One of the Icons in the home page demonstrates http server", -1, 237, 
											(42 + (10*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);												
														
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_RTC:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
	                GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
		        GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
		        GrImageDraw(&sContext[frameBufIdx], rtcTimeIcon,	xyTimeSet[0], 	xyTimeSet[2]);

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "RTC (Real Time Clock)", -1, 240, 42, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provides a time reference to applications", -1, 181, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- RTC is demonstrated here by setting its registers with user", -1, 253, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   entered date and time, and reading back the same", -1, 218, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_MMCSD:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);			
			GrStringDrawCentered(&sContext[frameBufIdx], "MMC / SD", -1, 240, 42, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- MMCHS host controller has built-in 1024-byte buffer for read", -1, 234, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "or write, two DMA channels, one interrupt line                ", -1, 245, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Support for SDA 3.0 Part A2 programming model               ", -1, 240, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- StarterWare provides support for                            ", -1, 230, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- SD v2.0 standard", -1, 130, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Standard Capacity and High capacity cards", -1, 210, 
											(42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Standard Speed and High Speed cards", -1, 194, 
											(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- DMA  mode of Operation", -1, 158, 
											(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- StarterWare AM335x Bootloader supports booting via MMC/SD", -1, 240, 
											(42 + (10*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_UART:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss18b);
			GrStringDrawCentered(&sContext[frameBufIdx], "UART (Universal Asynchronous Receiver Transmitter)", -1, 240, 42, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Compatible with industry standard 16C750", -1, 191, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Programmable baud rate upto 3.6864 Mbps", -1, 190, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provision for 64 bytes FIFO for transmit and receive", -1, 230, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provision for Modem Control Signals -RTS/CTS", -1, 208, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_TIMER:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);
			GrStringDrawCentered(&sContext[frameBufIdx], "DMTimer", -1, 240, 42, 0);			

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Contains a free running upward counter with auto", -1, 229, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "reload capability", -1, 107, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Timer peripheral can generate periodic interupts", -1, 220, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);													
			GrStringDrawCentered(&sContext[frameBufIdx], "- Observe the time variation interval of color change (below)", -1, 258, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "which is configured using DMTimer", -1, 176, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
		
		case CLICK_IDX_ECAP:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
		        GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
		        GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			GrImageDraw(&sContext[frameBufIdx], eCAPDemoIcon, 	xyEcapDemo[0], 	xyEcapDemo[2]);

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);
			GrStringDrawCentered(&sContext[frameBufIdx], "ECAP", -1, 240, 42, 0);	

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- eCAP module includes the following features", -1, 223, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- 32-bit time base counter                   ", -1, 220, 
											(42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- 4-event time-stamp registers               ", -1, 223, 
											(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Single or continous time-stamps capture mode", -1, 235, 
											(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- eCAP can be configured in PWM mode", -1, 204, 
											(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);																						
			GrStringDrawCentered(&sContext[frameBufIdx], "- LCD brightness is controlled through eCAP", -1, 216, 
											(42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], "- Press the icon below to see eCAP demo", -1, 206, 
											(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;	

                case CLICK_IDX_GPIO:
                    GrImageDraw(&sContext[frameBufIdx], homeIcon,               xyHome[0],      xyHome[2]);
                        GrImageDraw(&sContext[frameBufIdx], prevIcon,           xyPrev[0],      xyPrev[2]);
                        GrImageDraw(&sContext[frameBufIdx], nextIcon,           xyNext[0],      xyNext[2]);

                        GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
                        GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss22b);
                        GrStringDrawCentered(&sContext[frameBufIdx], "GPIO - LED Blink", -1, 240, 42, 0);

                        GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
                        GrStringDrawCentered(&sContext[frameBufIdx], "- Four GPIO modules with 32 pins in each. Can be used for", -1, 240,
                                                                                        (42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
                        GrStringDrawCentered(&sContext[frameBufIdx], "- Data input/output                   ", -1, 165,
                                                                                        (42 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
                        GrStringDrawCentered(&sContext[frameBufIdx], "- Keyboard interface with a de-bouncing cell", -1, 200,
                                                                                        (42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
                        GrStringDrawCentered(&sContext[frameBufIdx], "- Synchronous interrupt and wake-up request generation", -1, 232,
                                                                                        (42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
                        GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);                                                                              
                        GrStringDrawCentered(&sContext[frameBufIdx], "- This demo shows GPIO control for LED blink ", -1, 195,
                                                                                        (42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);

                        toggleFrameBuffer();
                        
                        memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));

                break;

		case CLICK_IDX_PM:
				GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			        GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
				GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
				GrImageDraw(&sContext[frameBufIdx], ds0Icon, 		xyPMds0Demo[0],	xyPMds0Demo[2]);
				GrImageDraw(&sContext[frameBufIdx], ds1Icon,
                                           xyPMds1Demo[0], xyPMds1Demo[2]);
				GrImageDraw(&sContext[frameBufIdx], standbyIcon,
                                            xyPMstandbyDemo[0], xyPMstandbyDemo[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeTscIcon, 		xyPMwksTsc[0],	xyPMwksTsc[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeTimerIcon, 		xyPMwksTmr[0],	xyPMwksTmr[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeUartIcon, 		xyPMwksUart[0],	xyPMwksUart[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeGpioIcon, 		xyPMwksGpio[0],	xyPMwksGpio[2]);

                if(DEVICE_VERSION_2_1 == deviceVersion)
                {
                    GrImageDraw(&sContext[frameBufIdx], wakeRtcIcon,
                                xyPMwksRTC[0],  xyPMwksRTC[2]);
                }
				
				GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
				if(WAKE_SOURCE_TSC == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksTsc[0] + xyPMwksTsc[1] + 5)/2,
                                                    (xyPMwksTsc[2] + xyPMwksTsc[3] - 5)/2, 2);
				}
				else if(WAKE_SOURCE_UART == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksUart[0] + xyPMwksUart[1] + 5)/2,
                                                    (xyPMwksUart[2] + xyPMwksUart[3] - 5)/2, 2);
				}
				else if(WAKE_SOURCE_TMR == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksTmr[0] + xyPMwksTmr[1] + 5)/2,
                                                    (xyPMwksTmr[2] + xyPMwksTmr[3] - 5)/2, 2);
				}
				else if(WAKE_SOURCE_GPIO == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksGpio[0] + xyPMwksGpio[1] + 5)/2, 
                                                    (xyPMwksGpio[2] + xyPMwksGpio[3] - 5)/2, 2);
				}
                else if(WAKE_SOURCE_RTC == wakeSource)
                {
                    GrCircleFill(&sContext[frameBufIdx], (xyPMwksRTC[0] + xyPMwksRTC[1] + 5)/2,
                                 (xyPMwksRTC[2] + xyPMwksRTC[3] - 5)/2, 2);
                }
				
				GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
				GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);
				GrStringDrawCentered(&sContext[frameBufIdx], "Power management Demo", -1, 240, 34, 0);

				GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm16b);
				GrContextFontSet(&sContext[!frameBufIdx], &g_sFontCm16b);
				GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);				
				GrStringDrawCentered(&sContext[frameBufIdx], "Wake Source ", -1, 240, 
				(85 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						
			
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
                	
        case CLICK_IDX_DVFS:
        {
            unsigned int maxOpp = DemoMaxOppGet();

			GrImageDraw(&sContext[frameBufIdx], homeIcon,           xyHome[0],      xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon,           xyPrev[0],      xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon,           xyNext[0],      xyNext[2]);

            if(OPP_50 <= maxOpp)
            {
                GrImageDraw(&sContext[frameBufIdx], opp50Icon, xyDVFSOpp50[0],
                            xyDVFSOpp50[2]);
            }

            if(OPP_100 <= maxOpp)
            {
                GrImageDraw(&sContext[frameBufIdx], opp100Icon, xyDVFSOpp100[0],
                            xyDVFSOpp100[2]);
            }

            if(OPP_120 <= maxOpp)
            {
                GrImageDraw(&sContext[frameBufIdx], opp120Icon, xyDVFSOpp120[0],
                            xyDVFSOpp120[2]);
            }

            if(SR_TURBO <= maxOpp)
            {
                GrImageDraw(&sContext[frameBufIdx], srturboIcon,
                            xyDVFSSrTurbo[0], xyDVFSSrTurbo[2]);
            }

            if(OPP_NITRO <= maxOpp)
            {
                GrImageDraw(&sContext[frameBufIdx], nitroIcon, xyDVFSNitro[0],
                            xyDVFSNitro[2]);
            }

			 GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);

			 if(OPP_50 == mpuOpp)
			 {
		               GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp50[0] + xyDVFSOpp50[1] + 30)/2, (xyDVFSOpp50[2] + xyDVFSOpp50[3] - 30)/2, 2);
			 }
			 else if(OPP_100 == mpuOpp)
			 {
			       GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp100[0] + xyDVFSOpp100[1] + 30)/2, 
                                           (xyDVFSOpp100[2] + xyDVFSOpp100[3] - 30)/2, 2);
			 }
			 else if(OPP_120 == mpuOpp)
			 {
			       GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp120[0] + xyDVFSOpp120[1] + 30)/2,
                                            (xyDVFSOpp120[2] + xyDVFSOpp120[3] - 30)/2, 2);
			 }
			 else if(SR_TURBO == mpuOpp)
			{
			       GrCircleFill(&sContext[frameBufIdx], (xyDVFSSrTurbo[0] + xyDVFSSrTurbo[1] + 30)/2, 
                                           (xyDVFSSrTurbo[2] + xyDVFSSrTurbo[3] - 30)/2, 2);
			}
            else if(OPP_NITRO == mpuOpp)
            {
                GrCircleFill(&sContext[frameBufIdx],
                             (xyDVFSNitro[0] + xyDVFSNitro[1] + 30)/2,
                             (xyDVFSNitro[2] + xyDVFSNitro[3] - 30)/2, 2);
            }

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);

	                GrStringDrawCentered(&sContext[frameBufIdx], 
		                             "Select the required OPP for further execution", -1, 240, 
			                     (75 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);

		        toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
			  break;
        }
		case CLICK_IDX_I2C:
		        GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss20b);
			GrStringDrawCentered(&sContext[frameBufIdx], "I2C - Accelerometer", -1, 240, 34, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
			GrContextFontSet(&sContext[!frameBufIdx], &g_sFontCm14b);
			
			GrStringDrawCentered(&sContext[frameBufIdx], "- Compliant with Philips I2C specification version 2.1", -1, 200, 
											(42 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Accelerometer is interfaced via I2C                 ", -1, 203, 
											(42 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);											
			GrContextForegroundSet(&sContext[frameBufIdx], ClrWhiteSmoke);
			GrRectFill(&sContext[frameBufIdx], &acceleroRect);			
			
		        GrContextForegroundSet(&sContext[frameBufIdx], ClrSlateGray);
			GrRectDraw(&sContext[frameBufIdx], &acceleroRect);
			acceleroRect.sXMin += 1; acceleroRect.sYMin += 1;
			acceleroRect.sXMax -= 1; acceleroRect.sYMax -= 1;
			GrRectDraw(&sContext[frameBufIdx], &acceleroRect);	

			toggleFrameBuffer();
			
			updateI2CDemo();
			
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));

		break;				
		
		default:
		break;
	}
}



void updateI2CDemo(void)
{
	int tiltDir, tiltDeg;
	static unsigned snakeIndex = 0, tempVal = 0;
	
	///////////////////////////
	getAcceleroPos(&tiltDir, &tiltDeg);	
	if((tiltDeg > 20) || (1 == printtemp))
	{
		printtemp = 0;
		
		tempVal = (unsigned int) (SNAKE_MAX_DELAY_MS - (tiltDeg * SNAKE_MOVE_FACTOR));
		delay((tempVal < 120) ? tempVal: 120);
		
		if(	(rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin > acceleroRect.sXMin) &&
			(rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin > acceleroRect.sYMin) &&
			(rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMax < acceleroRect.sXMax) &&
			(rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMax < acceleroRect.sYMax) )
		{
			GrContextForegroundSet(&sContext[!frameBufIdx], ClrWheat);
			GrRectFill(&sContext[!frameBufIdx], &rectSnake[(snakeIndex+0)%SNAKE_NO_CELLS]);			
			GrRectFill(&sContext[!frameBufIdx], &rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS]);		
			GrContextForegroundSet(&sContext[!frameBufIdx], ClrBrown);						
			GrRectFill(&sContext[!frameBufIdx], &rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS]);		
	
			GrContextForegroundSet(&sContext[!frameBufIdx], ClrWhite);
			GrRectFill(&sContext[!frameBufIdx], &rectSnake[(snakeIndex+3)%SNAKE_NO_CELLS]);
                        CacheDataCleanBuff((unsigned int) &g_pucBuffer[0],
                                            GrOffScreen24BPPSize(LCD_WIDTH,
                                            LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
                        CacheDataCleanBuff((unsigned int) &g_pucBuffer[1],
                                           GrOffScreen24BPPSize(LCD_WIDTH,
                                           LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
	
			snakeIndex++;
			snakeIndex = (snakeIndex%SNAKE_NO_CELLS);				
		}
		
		if(LEFT_TILT == tiltDir)
		{
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMin - (SNAKE_GAP + SNAKE_CELL_SIZE);
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMax = rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin + SNAKE_CELL_SIZE;
			
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMin;
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMax = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMax;
		}
		else if(RIGHT_TILT == tiltDir)
		{
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMin + (SNAKE_GAP + SNAKE_CELL_SIZE);
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMax = rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin + SNAKE_CELL_SIZE;
			
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMin;
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMax = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMax;
		}
		else if(BACK_TILT == tiltDir)
		{
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMin - (SNAKE_GAP + SNAKE_CELL_SIZE);
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMax = rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin + SNAKE_CELL_SIZE;

			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMin;
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMax = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMax;			
		}
		else if(FORWARD_TILT == tiltDir)
		{
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sYMin + (SNAKE_GAP + SNAKE_CELL_SIZE);
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMax = rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sYMin + SNAKE_CELL_SIZE;

			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMin = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMin;
			rectSnake[(snakeIndex+2)%SNAKE_NO_CELLS].sXMax = rectSnake[(snakeIndex+1)%SNAKE_NO_CELLS].sXMax;			
		}
		else	
		{}

                CacheDataCleanBuff((unsigned int) &g_pucBuffer[0],
                   GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT,
                   PIXEL_24_BPP_UNPACKED));
                CacheDataCleanBuff((unsigned int) &g_pucBuffer[1],
                   GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT,
                   PIXEL_24_BPP_UNPACKED));
	}
}
