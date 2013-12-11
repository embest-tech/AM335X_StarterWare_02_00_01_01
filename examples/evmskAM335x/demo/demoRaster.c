/**
 * \file   demoRaster.c 
 *
 * \brief  This file contains Raster related functions.
 *        
*/

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

#include "raster.h"
#include "interrupt.h"
#include "demoRaster.h"
#include "demoMain.h"
#include "demoCfg.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "raster.h"
#include "ecap.h"
#include "demoGrlib.h"
#include "grlib.h"
#include "cache.h"

/*******************************************************************************
**                     INTERNAL MACRO DECLARATIONS
*******************************************************************************/
#define SIZE_IMAGE_ARRAY            130568u


/*******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned char g_pucBuffer[2][GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
extern unsigned char baseUnCompImage[];
extern volatile unsigned int frameBufIdx;

/*******************************************************************************
**                     INTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
static void Raster0Isr(void);
void ImageArrExtract(unsigned int const *imagePtr, unsigned int *destPtr);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static volatile unsigned int fbSync = 0;

/*******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Configures raster to display image 
*/
void Raster0Init(void)
{
    RasterClocksEnable(SOC_LCDC_0_REGS);

    /* Disable raster */
    RasterDisable(SOC_LCDC_0_REGS);

    /* Configure the pclk */
    RasterClkConfig(SOC_LCDC_0_REGS, 23040000, 192000000);

    /* Configuring DMA of LCD controller */
    RasterDMAConfig(SOC_LCDC_0_REGS, RASTER_DOUBLE_FRAME_BUFFER,
                    RASTER_BURST_SIZE_16, RASTER_FIFO_THRESHOLD_8,
                    RASTER_BIG_ENDIAN_DISABLE);

    /* Configuring modes(ex:tft or stn,color or monochrome etc) for raster controller */
    RasterModeConfig(SOC_LCDC_0_REGS, RASTER_DISPLAY_MODE_TFT_UNPACKED,
                     RASTER_PALETTE_DATA, RASTER_COLOR, RASTER_RIGHT_ALIGNED);


     /* Configuring the polarity of timing parameters of raster controller */
    RasterTiming2Configure(SOC_LCDC_0_REGS, RASTER_FRAME_CLOCK_LOW |
                                            RASTER_LINE_CLOCK_LOW  |
                                            RASTER_PIXEL_CLOCK_HIGH|
                                            RASTER_SYNC_EDGE_RISING|
                                            RASTER_SYNC_CTRL_ACTIVE|
                                            RASTER_AC_BIAS_HIGH     , 0, 255);

    /* Configuring horizontal timing parameter */
    RasterHparamConfig(SOC_LCDC_0_REGS, 480, 4, 8, 43);

    /* Configuring vertical timing parameters */
    RasterVparamConfig(SOC_LCDC_0_REGS, 272, 10, 4, 12);


    RasterFIFODMADelayConfig(SOC_LCDC_0_REGS, 128);
}

/*
** Displays the banner image. 
*/
void Raster0Start(void)
{
    /* configuring the base ceiling */
    RasterDMAFBConfig(SOC_LCDC_0_REGS, 
                      (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET) + (SIZE_IMAGE_ARRAY*4) - 1,
					  FRAME_BUFFER_0);

    RasterDMAFBConfig(SOC_LCDC_0_REGS, 
                      (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET),
                      (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET) + (SIZE_IMAGE_ARRAY*4) - 1,
					  FRAME_BUFFER_1);
					  
     /* enable raster */
     RasterEnable(SOC_LCDC_0_REGS);
}

/*
**  A wrapper function which enables the End-Of-Frame interrupt of Raster.
*/
void Raster0EOFIntEnable(void)
{
    /* Enable End of frame0/frame1 interrupt */
    RasterIntEnable(SOC_LCDC_0_REGS, RASTER_END_OF_FRAME0_INT |
                                     RASTER_END_OF_FRAME1_INT);

}

/*
** A wrapper function which disables the End-Of-Frame interrupt of Raster.
*/
void Raster0EOFIntDisable(void)
{
    /* Enable End of frame0/frame1 interrupt */
    RasterIntDisable(SOC_LCDC_0_REGS, RASTER_END_OF_FRAME0_INT |
                                     RASTER_END_OF_FRAME1_INT);

}

/*
**  This function registers the ISR, maps a channel and enables the
**  respective interrupt in AINTC for the Raster.
*/
void Raster0IntRegister(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_LCDCINT, Raster0Isr);
}

/*
** Interrupt Service Routine(ISR) for Raster. This function displays an image 
** on the LCD.
*/
static void Raster0Isr(void)
{
    unsigned int  status;
    status = RasterIntStatus(SOC_LCDC_0_REGS,RASTER_END_OF_FRAME0_INT_STAT |
                                             RASTER_END_OF_FRAME1_INT_STAT );

    status = RasterClearGetIntStatus(SOC_LCDC_0_REGS, status);

    if (status & RASTER_END_OF_FRAME0_INT_STAT)
    {
		/* configuring the base ceiling */
		RasterDMAFBConfig(SOC_LCDC_0_REGS, 
						  (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET),
						  (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET) + (SIZE_IMAGE_ARRAY*4) - 1,
						  FRAME_BUFFER_0);
	}
	
    if(status & RASTER_END_OF_FRAME1_INT_STAT)
    {
		RasterDMAFBConfig(SOC_LCDC_0_REGS, 
						  (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET),
						  (unsigned int)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET) + (SIZE_IMAGE_ARRAY*4) - 1,
					  FRAME_BUFFER_1);
	}

	fbSync++;
}

/*
** Extracts an Image to the imageArr array.
*/
void ImageArrExtract(unsigned int const *imagePtr, unsigned int *destPtr)
{
    unsigned int idx = 0;
    unsigned int pixCnt = 0;
    unsigned int pixVal;
    unsigned int cnt;

    while(idx < SIZE_IMAGE_ARRAY)
    {
        pixVal =  *imagePtr;
        if(pixVal & 0xFF000000)
        {
            pixCnt = pixVal >> 24;
        }

        else
        {
            pixCnt = *(imagePtr + 1);
            imagePtr++;
        }

        for(cnt= pixCnt; cnt != 0; cnt--)
        {
           destPtr[idx] =  pixVal & 0xFFFFFF;
           idx++;
        }

        imagePtr++;
    }
}

void toggleFrameBuffer(void)
{
        CacheDataCleanBuff((unsigned int) &g_pucBuffer[0],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
        CacheDataCleanBuff((unsigned int) &g_pucBuffer[1],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));

	frameBufIdx = !frameBufIdx;
	fbSync = 0;
	while(fbSync <2);
}


/***************************** End Of File ************************************/
