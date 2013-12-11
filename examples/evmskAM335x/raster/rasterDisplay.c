/**
 * \file  rasterDisplay.c
 *
 * \brief Sample application for raster. This application receives the
 *        image data from a frame buffer and displays the image on the
 *        LCD panel.
 *
 *        Application Configuration:
 *
 *            Modules Used:
 *                LCD Controller
 *                Interrupt Controller
 *
 *            Configurable Parameters:
 *                None.
 *
 *            Hard-coded configuration of other parameters:
 *                Pixel clock frequency - 23MHz
 *                Functional mode       - Raster
 *                Display mode          - TFT
 *                LCD frame resolution  - 480 x 272
 *                Input data format     - 24-BPP Unpacked
 *
 *        Application Usecase:
 *        1. This application works in double frame buffer mode.
 *        2. The controller receives the data from one of the two frame
 *           buffers alternatively and displays image on the LCD panel.
 *        3. The ISR handles only End-Of-Frame interrupt. The address of
 *           next frame buffer will be updated in each End-Of-Frame interrupt.
 *
 *        Running the example:
 *        1. Raster LCD should be hooked on the board.
 *        2. Load the raster example on the target and execute.
 *        3. When the application is executed, the image will be displayed
 *           on the LCD panel.
 *
 *        Limitations:
 *        1. Packed mode of 24-BPP pixel format is not supported. Only unpacked
 *           mode of 24-BPP format is supported.
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
#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmskAM335x.h"
#include "raster.h"
#include "image.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
#define  LCDC_INSTANCE                SOC_LCDC_0_REGS

#define  START_ADDR_DDR               (0x80000000)

#define  START_ADDR_DEV               (0x44000000)

#define  START_ADDR_OCMC              (0x40300000)

#define  NUM_SECTIONS_DDR             (512)

#define  NUM_SECTIONS_DEV             (960)

#define  NUM_SECTIONS_OCMC            (1)

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif
/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void LCDIsr(void);
static void SetUpLCD(void);
static void LCDAINTCConfigure(void);
/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

/*
** Function to setup MMU. This function Maps three regions ( 1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes 
    ** specified here are, 
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given. 
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

	
int main(void)
{
    MMUConfigAndEnable();

    CacheEnable(CACHE_ALL);

    IntMasterIRQEnable();

    IntAINTCInit();

    LCDAINTCConfigure();

    LCDBackLightEnable();

    SetUpLCD();
  
    /* Configuring the base ceiling */
    RasterDMAFBConfig(LCDC_INSTANCE, 
                      (unsigned int)image1,
                      (unsigned int)image1 + sizeof(image1) - 2,
                      0);

    RasterDMAFBConfig(LCDC_INSTANCE, 
                      (unsigned int)image1,
                      (unsigned int)image1 + sizeof(image1) - 2,
                      1);

    /* Enable End of frame0/frame1 interrupt */
    RasterIntEnable(LCDC_INSTANCE, RASTER_END_OF_FRAME0_INT |
                                     RASTER_END_OF_FRAME1_INT);

    /* Enable raster */
    RasterEnable(LCDC_INSTANCE);

	
    while(1); 
}

/*
** Configures raster to display image 
*/
static void SetUpLCD(void)
{
    /* Enable clock for LCD Module */ 
    LCDModuleClkConfig();

    LCDPinMuxSetup();

    /* 
    **Clock for DMA,LIDD and for Core(which encompasses
    ** Raster Active Matrix and Passive Matrix logic) 
    ** enabled.
    */
    RasterClocksEnable(LCDC_INSTANCE);

    /* Disable raster */
    RasterDisable(LCDC_INSTANCE);
    
    /* Configure the pclk */
    RasterClkConfig(LCDC_INSTANCE, 23040000, 192000000);

    /* Configuring DMA of LCD controller */ 
    RasterDMAConfig(LCDC_INSTANCE, RASTER_DOUBLE_FRAME_BUFFER,
                    RASTER_BURST_SIZE_16, RASTER_FIFO_THRESHOLD_8,
                    RASTER_BIG_ENDIAN_DISABLE);

    /* Configuring modes(ex:tft or stn,color or monochrome etc) for raster controller */
    RasterModeConfig(LCDC_INSTANCE, RASTER_DISPLAY_MODE_TFT_UNPACKED,
                     RASTER_PALETTE_DATA, RASTER_COLOR, RASTER_RIGHT_ALIGNED);


     /* Configuring the polarity of timing parameters of raster controller */
    RasterTiming2Configure(LCDC_INSTANCE, RASTER_FRAME_CLOCK_LOW |
                                            RASTER_LINE_CLOCK_LOW  |
                                            RASTER_PIXEL_CLOCK_HIGH|
                                            RASTER_SYNC_EDGE_RISING|
                                            RASTER_SYNC_CTRL_ACTIVE|
                                            RASTER_AC_BIAS_HIGH     , 0, 255);

    /* Configuring horizontal timing parameter */
    RasterHparamConfig(LCDC_INSTANCE, 480, 4, 8, 43);

    /* Configuring vertical timing parameters */
    RasterVparamConfig(LCDC_INSTANCE, 272, 10, 4, 12);


    RasterFIFODMADelayConfig(LCDC_INSTANCE, 128);

}


/*
** configures arm interrupt controller to generate raster interrupt 
*/
static void LCDAINTCConfigure(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_LCDCINT, LCDIsr);

    IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enable the System Interrupts for AINTC.*/
    IntSystemEnable(SYS_INT_LCDCINT);
}

/*
** For each end of frame interrupt base and ceiling is reconfigured 
*/
static void LCDIsr(void)
{
    unsigned int  status;

    status = RasterIntStatus(LCDC_INSTANCE,RASTER_END_OF_FRAME0_INT_STAT |
                                             RASTER_END_OF_FRAME1_INT_STAT );

    status = RasterClearGetIntStatus(LCDC_INSTANCE, status);   

    if (status & RASTER_END_OF_FRAME0_INT_STAT)
    {
        RasterDMAFBConfig(LCDC_INSTANCE, 
                          (unsigned int)image1,
                          (unsigned int)image1 + sizeof(image1) - 2,
                          0);
    }

    if(status & RASTER_END_OF_FRAME1_INT_STAT)
    {
        RasterDMAFBConfig(LCDC_INSTANCE, 
                          (unsigned int)image1,
                          (unsigned int)image1 + sizeof(image1) - 2,
                          1);
    }
}

/***************************** End Of File ************************************/
