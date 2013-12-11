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

#include "icons.h"

#include <string.h>
#include <stdlib.h>

#define BPP	32

#define SNAKE_CELL_SIZE		(10)
#define SNAKE_MIDDLE_X_MIN	(395)
#define SNAKE_MIDDLE_Y_MIN	(315)
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
tDisplay g_s35_800x480x24Display[2];

tContext sContext[2];
static tRectangle acceleroRect = {150, 200, 650, 440};
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
extern int const xyPMRTCDemo[4];
extern int const xyPMds0Demo[4]; 
extern int const xyPMds1Demo[4]; 
extern int const xyPMstandbyDemo[4];
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
	GrOffScreen24BPPInit(&g_s35_800x480x24Display[0], g_pucBuffer[0], LCD_WIDTH, LCD_HEIGHT);
	GrOffScreen24BPPInit(&g_s35_800x480x24Display[1], g_pucBuffer[1], LCD_WIDTH, LCD_HEIGHT);

	// Initialize a drawing context.
	GrContextInit(&sContext[0], &g_s35_800x480x24Display[0]);
	GrContextInit(&sContext[1], &g_s35_800x480x24Display[1]);

}


/*
** Update the display context with next slide content
*/
void updatePage(unsigned int demoIndex)
{
	tRectangle acceleroRect = {150, 200, 650, 440};
	
	switch(demoIndex)
	{
		case CLICK_IDX_MENU:
			// Draw menu icons
			GrImageDraw(&sContext[frameBufIdx], infoIcon,		xyIntro[0], 	xyIntro[2]);
			GrImageDraw(&sContext[frameBufIdx], webIcon, 		xyWebDemo[0], 	xyWebDemo[2]);
			GrImageDraw(&sContext[frameBufIdx], mcaspIcon, 		xyMcASP[0], 	xyMcASP[2]);
			GrImageDraw(&sContext[frameBufIdx], mmcsdIcon, 		xyMMCSD[0], 	xyMMCSD[2]);
			GrImageDraw(&sContext[frameBufIdx], uartIcon, 		xyUart[0], 		xyUart[2]);
			GrImageDraw(&sContext[frameBufIdx], rtcIcon, 		xyRTC[0], 		xyRTC[2]);
			GrImageDraw(&sContext[frameBufIdx], timerIcon, 		xyTimer[0], 	xyTimer[2]);
			GrImageDraw(&sContext[frameBufIdx], ethernetIcon,	xyEthernet[0],	xyEthernet[2]);  
			GrImageDraw(&sContext[frameBufIdx], eCAPIcon,		xyEcapMenu[0],	xyEcapMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], gpioIcon,		xyGpioMenu[0],	xyGpioMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], i2cIcon,		xyI2CMenu[0],	xyI2CMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], pmIcon,			xyPMMenu[0],	xyPMMenu[2]);  
			GrImageDraw(&sContext[frameBufIdx], dvfsIcon,			xyDVFSMenu[0],	xyDVFSMenu[2]);  
			
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "-  A no OS platform support package for AM335x SoC         ", -1, 400, 75, 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "-  Provides device abstraction libraries for peripherals    ", -1, 380,
											(75 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   and board level example applications                     ", -1, 380, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 
			GrStringDrawCentered(&sContext[frameBufIdx], "-  Ensure the Ethernet port is connected to LAN if Ethernet ", -1, 400, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   shall be used for driving the demo                       ", -1, 380, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 

			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], "   Ensure the UART port onboard is connected to the UART    ", -1, 400, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						 
			GrStringDrawCentered(&sContext[frameBufIdx], " communication port on the host and a serial communication", -1, 400, 
											(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);				
			GrStringDrawCentered(&sContext[frameBufIdx], "   application is running (TeraTerm/HyperTerminal/minicom)  ", -1, 400, 
											(75 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);															
			GrStringDrawCentered(&sContext[frameBufIdx], "    on the host. All the input values are accepted through UART.", -1, 400, 
											(75 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);															

											
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "The demo can be continued using Touch Screen and/or Ethernet", -1, 400,
											(75 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], "To control via Touch Screen press on next icon", -1, 380,
											(75 + (12*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);

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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "McASP (Multi-channel Audio Serial Port)", -1, 400, 75, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- General purpose audio serial port optimized for the needs of", -1, 400, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], " multichannel audio applications                              ", -1, 390, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Supports TDM stream, 125 protocols and DIT                  ", -1, 396, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Seperate transmit and receive sections that can operate     ", -1, 390, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], " synchronously or independently                               ", -1, 390, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);		
											
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
			GrStringDrawCentered(&sContext[frameBufIdx], "Ensure that a speaker/headphone is connected to the	", -1, 400, 
											(75 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
			GrStringDrawCentered(&sContext[frameBufIdx], "LINE-OUT of EVM to listen to the background music", -1, 400, 
											(75 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
											
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "Ethernet", -1, 400, 75, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- The ethernet switch is compliant to IEEE 802.3 standard", -1, 356, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- It has one host port and two slave ports, which are capable", -1, 386, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "of 10/100/1000 Mbps with MII/GMII/RMII/RGMII interfaces", -1, 396, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- The MDIO module implements the 802.3 serial management  ", -1, 390, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);			
			GrStringDrawCentered(&sContext[frameBufIdx], "interface to interrogate and control up to 32 Ethernet PHYs", -1, 394, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], "- LwIP is used for implementing IP stack in StarterWare", -1, 360, 
											(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);			
			GrStringDrawCentered(&sContext[frameBufIdx], "- The device abstraction layer is glued by implementing the", -1, 374, 
											(75 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
			GrStringDrawCentered(&sContext[frameBufIdx], "interface layer of lwIP", -1, 200, 
											(75 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);		
			GrStringDrawCentered(&sContext[frameBufIdx], "- One of the Icons in the home page demonstrates http server", -1, 382, 
											(75 + (10*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);												
														
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "RTC (Real Time Clock)", -1, 400, 75, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provides a time reference to applications", -1, 302, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- RTC is demonstrated here by setting its registers with user", -1, 400, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   entered date and time, and reading back the same", -1, 350, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);			
			GrStringDrawCentered(&sContext[frameBufIdx], "MMC / SD", -1, 400, 75, 0);
			
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- MMCHS host controller has built-in 1024-byte buffer for read", -1, 390, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "or write, two DMA channels, one interrupt line", -1, 320, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Support for SDA 3.0 Part A2 programming model", -1, 324, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- StarterWare provides support for", -1, 240, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- SD v2.0 standard", -1, 216, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Standard Capacity and High capacity cards", -1, 350, 
											(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Standard Speed and High Speed cards", -1, 326, 
											(75 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- DMA  mode of Operation", -1, 262, 
											(75 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- StarterWare AM335x Bootloader supports booting via MMC/SD", -1, 394, 
											(75 + (10*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "UART (Universal Asynchronous Receiver Transmitter)", -1, 400, 75, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Compatible with industry standard 16C750", -1, 302, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Programmable baud rate upto 3.6864 Mbps", -1, 306, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provision for 64 bytes FIFO for transmit and receive", -1, 360, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Provision for Modem Control Signals -RTS/CTS", -1, 334, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "DMTimer", -1, 400, 75, 0);			

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Contains a free running upward counter with auto", -1, 360, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "   reload capability", -1, 180, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Timer peripheral can generate periodic interupts", -1, 350, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);													
			GrStringDrawCentered(&sContext[frameBufIdx], "- Observe the time variation interval of color change (below)", -1, 400, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "which is configured using DMTimer", -1, 286, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);

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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "ECAP", -1, 400, 75, 0);			

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- eCAP module includes the following features", -1, 366, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- 32-bit time base counter                   ", -1, 370, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- 4-event time-stamp registers               ", -1, 374, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Single or continous time-stamps capture mode", -1, 416, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- eCAP can be configured in PWM mode", -1, 340, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);																						
			GrStringDrawCentered(&sContext[frameBufIdx], "- LCD brightness is controlled through eCAP", -1, 360, 
											(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
			GrStringDrawCentered(&sContext[frameBufIdx], "- Press the icon below to see eCAP demo", -1, 340, 
											(75 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;		

		case CLICK_IDX_GPIO:
		    GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
			GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
			GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);
			GrImageDraw(&sContext[frameBufIdx], audioBuzzerDemo, 	xyBuzzDemo[0], 	xyBuzzDemo[2]); // to be changed to GPIO icon

			GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "GPIO - Audio Buzzer", -1, 400, 75, 0);			

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Four GPIO modules with 32 pins in each. Can be used for", -1, 400, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Data input/output                   ", -1, 300, 
											(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Keyboard interface with a de-bouncing cell", -1, 345, 
											(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "-  Synchronous interrupt and wake-up request generation", -1, 388, 
											(75 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);												
			GrStringDrawCentered(&sContext[frameBufIdx], "- This demo shows GPIO control of audio buzzer", -1, 343, 
											(75 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Press the icon below for audio buzzer demo ", -1, 333, 
											(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);											
											
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;		
		
		case CLICK_IDX_PM:
				GrImageDraw(&sContext[frameBufIdx], homeIcon, 		xyHome[0], 	xyHome[2]);
				GrImageDraw(&sContext[frameBufIdx], prevIcon, 		xyPrev[0], 	xyPrev[2]);
				GrImageDraw(&sContext[frameBufIdx], nextIcon, 		xyNext[0], 	xyNext[2]);

                if((DEVICE_VERSION_2_0 == deviceVersion) ||
                   (DEVICE_VERSION_2_1 == deviceVersion))
                {
                    GrImageDraw(&sContext[frameBufIdx], rtconlyIcon,
                                xyPMRTCDemo[0], xyPMRTCDemo[2]);
                }

				GrImageDraw(&sContext[frameBufIdx], ds0Icon, 		xyPMds0Demo[0],	xyPMds0Demo[2]);
				GrImageDraw(&sContext[frameBufIdx], ds1Icon, 		xyPMds1Demo[0],	xyPMds1Demo[2]);
                GrImageDraw(&sContext[frameBufIdx], standbyIcon,
                            xyPMstandbyDemo[0], xyPMstandbyDemo[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeTscIcon, 		xyPMwksTsc[0],	xyPMwksTsc[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeTimerIcon, 		xyPMwksTmr[0],	xyPMwksTmr[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeUartIcon, 		xyPMwksUart[0],	xyPMwksUart[2]);
				GrImageDraw(&sContext[frameBufIdx], wakeGpioIcon, 		xyPMwksGpio[0],	xyPMwksGpio[2]);

                if((DEVICE_VERSION_2_0 == deviceVersion) ||
                   (DEVICE_VERSION_2_1 == deviceVersion))
                {
                    GrImageDraw(&sContext[frameBufIdx], wakeRTCIcon,
                                xyPMwksRTC[0],  xyPMwksRTC[2]);
                }
				
				GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
				if(WAKE_SOURCE_TSC == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksTsc[0] + xyPMwksTsc[1] + 40)/2, (xyPMwksTsc[2] + xyPMwksTsc[3] - 40)/2, 5);
				}
				else if(WAKE_SOURCE_UART == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksUart[0] + xyPMwksUart[1] + 40)/2, (xyPMwksUart[2] + xyPMwksUart[3] - 40)/2, 5);
				}
				else if(WAKE_SOURCE_TMR == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksTmr[0] + xyPMwksTmr[1] + 40)/2, (xyPMwksTmr[2] + xyPMwksTmr[3] - 40)/2, 5);
				}
				else if(WAKE_SOURCE_GPIO == wakeSource)
				{
					GrCircleFill(&sContext[frameBufIdx], (xyPMwksGpio[0] + xyPMwksGpio[1] + 40)/2, (xyPMwksGpio[2] + xyPMwksGpio[3] - 40)/2, 5);
				}
                else if(WAKE_SOURCE_RTC == wakeSource)
                {
                    GrCircleFill(&sContext[frameBufIdx], (xyPMwksRTC[0] + xyPMwksRTC[1] + 40)/2,
                                 (xyPMwksRTC[2] + xyPMwksRTC[3] - 40)/2, 5);
                }
				
				GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
				GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
				GrStringDrawCentered(&sContext[frameBufIdx], "Power management Demo", -1, 400, 60, 0);

				GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
				GrContextFontSet(&sContext[!frameBufIdx], &g_sFontCm22b);
				GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);				
				GrStringDrawCentered(&sContext[frameBufIdx], 
					"Select the Sleep mode", -1, 400, 
						(75 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
				GrStringDrawCentered(&sContext[frameBufIdx], "Wake Source ", -1, 400, 
						(75 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);						
			
			// Toggle frame
			toggleFrameBuffer();
			// Copy base image to FB
			memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));
		break;
                	
        case CLICK_IDX_DVFS:
        {
            unsigned int maxOpp = DemoMaxOppGet();

			GrImageDraw(&sContext[frameBufIdx], homeIcon,               xyHome[0],      xyHome[2]);
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
				  GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp50[0] + xyDVFSOpp50[1] + 60)/2, (xyDVFSOpp50[2] + xyDVFSOpp50[3] - 60)/2, 5);
			 }
			 else if(OPP_100 == mpuOpp)
			 {
					   GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp100[0] + xyDVFSOpp100[1] + 60)/2, (xyDVFSOpp100[2] + xyDVFSOpp100[3] - 60)/2, 5);
			 }
			 else if(OPP_120 == mpuOpp)
			 {
					GrCircleFill(&sContext[frameBufIdx], (xyDVFSOpp120[0] + xyDVFSOpp120[1] + 60)/2, (xyDVFSOpp120[2] + xyDVFSOpp120[3] - 60)/2, 5);
				}
				else if(SR_TURBO == mpuOpp)
				{
						GrCircleFill(&sContext[frameBufIdx], (xyDVFSSrTurbo[0] + xyDVFSSrTurbo[1] + 60)/2, (xyDVFSSrTurbo[2] + xyDVFSSrTurbo[3] - 60)/2, 5);
				}
            else if(OPP_NITRO == mpuOpp)
            {
                GrCircleFill(&sContext[frameBufIdx],
                             (xyDVFSNitro[0] + xyDVFSNitro[1] + 60)/2,
                             (xyDVFSNitro[2] + xyDVFSNitro[3] - 60)/2, 5);
            }

				GrContextForegroundSet(&sContext[frameBufIdx], ClrBlack);
				GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);

	GrStringDrawCentered(&sContext[frameBufIdx], 
		"Select the required OPP for further execution", -1, 400, 
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
			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCmss24b);
			GrStringDrawCentered(&sContext[frameBufIdx], "I2C - Temperature Sensor, Accelerometer", -1, 400, 60, 0);

			GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm22b);
			GrContextFontSet(&sContext[!frameBufIdx], &g_sFontCm22b);
			
			GrStringDrawCentered(&sContext[frameBufIdx], "- Compliant with Philips I2C specification version 2.1", -1, 366, 
											(75 + (1*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			//GrStringDrawCentered(&sContext[frameBufIdx], "- Supports standard mode, fast mode and high speed mode", -1, 370, 
				//							(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			//GrStringDrawCentered(&sContext[frameBufIdx], "- Combined master transmit/receive and receive/transmit modes", -1, 374, 
				//							(75 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			//GrStringDrawCentered(&sContext[frameBufIdx], "- 7-bit and 10-bit device addressing modes", -1, 416, 
				//							(75 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrStringDrawCentered(&sContext[frameBufIdx], "- Temperature sensor and Accelerometer are interfaced via I2C", -1, 416, 
											(75 + (2*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);											
			GrStringDrawCentered(&sContext[frameBufIdx], "Temperature: ", -1, 220, 
							(85 + (3*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
							
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
    unsigned int currTempVal[2] = {1,1};
    static unsigned int snakeIndex = 0;
    static unsigned int prevTempVal[2];
    static char tempMsg[20] = {0};
    static char cleanMsg[20];
    unsigned int tempVal = 0;
    unsigned int i_msg = 0;
    unsigned int i_index;
	int tiltDir, tiltDeg;
	
	updateTemperature(&currTempVal[0], &currTempVal[1]);

    if((currTempVal[0] != prevTempVal[0]) ||
       (currTempVal[1] != prevTempVal[1]) ||
       (1 == printtemp))
	{
		prevTempVal[0] = currTempVal[0];
		prevTempVal[1] = currTempVal[1];		
	
		for(i_index = 0; i_index < 2; i_index++)
		{
			tempVal = currTempVal[i_index];
			if(0 == tempVal)
				tempMsg[i_msg++] = '0';
				
			if(tempVal/1000)
			{
				tempMsg[i_msg++] = (unsigned char)((tempVal/1000) + 48);
				tempVal = tempVal%1000;		
				tempMsg[i_msg++] = (unsigned char)((tempVal/100) + 48);
				tempVal = tempVal%100;
				tempMsg[i_msg++] = (unsigned char)((tempVal/10) + 48);
				tempVal = tempVal%10;
				tempMsg[i_msg++] = (unsigned char)(tempVal + 48);
				tempVal = 0;
			}		
			if(tempVal/100)
			{
				tempMsg[i_msg++] = (unsigned char)((tempVal/100) + 48);
				tempVal = tempVal%100;
				tempMsg[i_msg++] = (unsigned char)((tempVal/10) + 48);
				tempVal = tempVal%10;
				tempMsg[i_msg++] = (unsigned char)(tempVal + 48);
				tempVal = 0;
			}
			if(tempVal/10)
			{
				tempMsg[i_msg++] = (unsigned char)((tempVal/10) + 48);
				tempVal = tempVal%10;
				tempMsg[i_msg++] = (unsigned char)(tempVal + 48);
				tempVal = 0;
			}
			if(tempVal)
			{
				tempMsg[i_msg++] = (unsigned char)(tempVal + 48);
			}

			tempMsg[i_msg++] = '.';
		 }
		 tempMsg[--i_msg] = '\0';
		 strcat(tempMsg, " deg C");
		 
		 GrContextForegroundSet(&sContext[!frameBufIdx], ClrWhiteSmoke);
		 GrStringDrawCentered(&sContext[!frameBufIdx], cleanMsg, -1, 360, 
						 (85 + (3*(3 + GrStringHeightGet(&sContext[!frameBufIdx])))), 0);	 
			
		 GrContextForegroundSet(&sContext[!frameBufIdx], ClrRed);					 
		 GrStringDrawCentered(&sContext[!frameBufIdx], tempMsg, -1, 360, 
						 (85 + (3*(3 + GrStringHeightGet(&sContext[!frameBufIdx])))), 0);

                 CacheDataCleanBuff((unsigned int) &g_pucBuffer[0], 
                                     GrOffScreen24BPPSize(LCD_WIDTH, 
                                     LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
                 CacheDataCleanBuff((unsigned int) &g_pucBuffer[1], 
                                     GrOffScreen24BPPSize(LCD_WIDTH, 
                                     LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
		 strcpy(cleanMsg, tempMsg);
	}
		
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
                               GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT,
                               PIXEL_24_BPP_UNPACKED));
                        CacheDataCleanBuff((unsigned int) &g_pucBuffer[1], 
                               GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT,
                               PIXEL_24_BPP_UNPACKED));
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



