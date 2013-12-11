/**
 * \file bl_platform.h
 *
 * \brief This file exports the APIs used for configuring devices
 *        required during boot
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
 

#ifndef _BL_PLATFORM_H__
#define _BL_PLATFORM_H__

#include "soc_AM335x.h"

#ifdef evmAM335x
    #include "evmAM335x.h"
#elif  (defined beaglebone)
    #include "beaglebone.h"
#elif  (defined evmskAM335x)
    #include "evmskAM335x.h"
#endif    

#if defined(SPI)
#elif defined(MMCSD)
#elif defined(NAND)
  #include "nandlib.h"
#endif


/******************************************************************************
**                        Macro Definitions 
*******************************************************************************/

/* Set of config parameters for AM335x */

    /* Fix the offset, where the boot image lies in the flash */
    /* TODO : Verify size */
#if defined(SPI)

    #define IMAGE_OFFSET                   128 * 1024 /* Will start from 3rd block */
    #define SPI_CHAN                       0x0
    #define SPI_BASE                       SOC_SPI_0_REGS

#elif defined(NAND)

    #define IMAGE_OFFSET                   512 * 1024 /* Will start from 3rd block */
    /* NAND related */
    #define NAND_CS0_BASEADDR              (0x10000000)
    #define NAND_CS0_REGIONSIZE            (GPMC_CS_SIZE_256MB)
    #define NAND_DATA_XFER_MODE            (NAND_XFER_MODE_CPU)
    #define NAND_BUSWIDTH                  (NAND_BUSWIDTH_8BIT)
    #define NAND_CHIP_SELECT               (NAND_CHIP_SELECT_0)
    #define NAND_PAGE_SIZE_IN_BYTES        (NAND_PAGESIZE_2048BYTES)
    #define NAND_BLOCK_SIZE_IN_BYTES       (NAND_BLOCKSIZE_128KB)
    #define NAND_NUMOF_BLK                 (2048)
    #define NAND_MANUFATURER_MICRON_ID     (0x2C)
    #define NAND_DEVICE_ID                 (0xDA)

    #define NAND_CSWROFFTIME               (30)
    #define NAND_CSRDOFFTIME               (31)
    #define NAND_CSONTIME                  (0)

    #define NAND_ADVONTIME                 (0)
    #define NAND_ADVAADMUXONTIME           (0)
    #define NAND_ADVRDOFFTIME              (31)
    #define NAND_ADVWROFFTIME              (31)
    #define NAND_ADVAADMUXRDOFFTIME        (0)
    #define NAND_ADVAADMUXWROFFTIME        (0)

    #define NAND_WEOFFTIME                 (31)
    #define NAND_WEONTIME                  (3)
    #define NAND_OEAADMUXOFFTIME           (31)
    #define NAND_OEOFFTIME                 (31)
    #define NAND_OEAADMUXONTIME            (3)
    #define NAND_OEONTIME                  (1)

    #define NAND_RDCYCLETIME               (31)
    #define NAND_WRCYCLETIME               (31)
    #define NAND_RDACCESSTIME              (28)
    #define NAND_PAGEBURSTACCESSTIME       (0)

    #define NAND_BUSTURNAROUND             (0)
    #define NAND_CYCLE2CYCLEDIFFCSEN       (0)
    #define NAND_CYCLE2CYCLESAMECSEN       (1)
    #define NAND_CYCLE2CYCLEDELAY          (0)
    #define NAND_WRDATAONADMUXBUS          (15)
    #define NAND_WRACCESSTIME              (22)

#elif defined(MMCSD)

    #define MMCSD_BASE                     SOC_MMCHS_0_REGS
    #define MMCSD_DMA_BASE                 SOC_EDMA30CC_0_REGS

    #define MMCSD_IN_FREQ                  96000000 /* 96MHz */
    #define MMCSD_INIT_FREQ                400000   /* 400kHz */

    #define MMCSD_DMA_CHA_TX               24
    #define MMCSD_DMA_CHA_RX               25
    #define MMCSD_DMA_QUE_NUM              0
    #define MMCSD_DMA_REGION_NUM           0
    #define MMCSD_BLK_SIZE                 512
    #define MMCSD_OCR                      (SD_OCR_VDD_3P0_3P1 | SD_OCR_VDD_3P1_3P2)

#endif

#define IMAGE_SIZE                         50 * 1024 /* Max size */
#define DDR_START_ADDR                     0x80000000
#define UART_BASE                          SOC_UART_0_REGS

/* Set of config parameters */

/* 
**Setting the CORE PLL values at OPP100:
** OSCIN = 24MHz, Fdpll = 2GHz
** HSDM4 = 200MHz, HSDM5 = 250MHz
** HSDM6 = 500MHz
*/
#define COREPLL_M                          1000
#define COREPLL_N                          23
#define COREPLL_HSD_M4                     10
#define COREPLL_HSD_M5                     8
#define COREPLL_HSD_M6                     4

/* Setting the  PER PLL values at OPP100:
** OSCIN = 24MHz, Fdpll = 960MHz
** CLKLDO = 960MHz, CLKOUT = 192MHz
*/
#define PERPLL_M                           960
#define PERPLL_N                           23
#define PERPLL_M2                          5

 /* Setting the Display CLKOUT at 300MHz independently
 ** This is required for full clock 150MHz for LCD
 ** OSCIN = 24MHz, Fdpll = 300MHz
 ** CLKOUT = 150MHz
 */
#define DISPLL_M                           25
#define DISPLL_N                           3
#define DISPLL_M2                          1

/* 
**Setting the DDR2 frequency to 266MHz
*/
#define DDRPLL_M_DDR2                     (266)
#define DDRPLL_M_DDR3                     (303)
#define DDRPLL_N		           23
#define DDRPLL_M2		           1

/*
** MACROS to configure SEL bit filed in VDD1_OP_REG of PMIC.
** Refer the datasheet of PMIC for voltage values.
*/
#if (defined beaglebone)

#define     PMIC_VOLT_SEL_0950MV      DCDC_VOLT_SEL_0950MV
#define     PMIC_VOLT_SEL_1100MV      DCDC_VOLT_SEL_1100MV
#define     PMIC_VOLT_SEL_1200MV      DCDC_VOLT_SEL_1200MV
#define     PMIC_VOLT_SEL_1260MV      DCDC_VOLT_SEL_1275MV
#define     PMIC_VOLT_SEL_1325MV      (0x11u)

#elif defined (evmAM335x) || defined (evmskAM335x)

#define     PMIC_VOLT_SEL_0950MV      (0x1fu)
#define     PMIC_VOLT_SEL_1100MV      (0x2bu)
#define     PMIC_VOLT_SEL_1200MV      (0x33u)
#define     PMIC_VOLT_SEL_1260MV      (0x38u)
#define     PMIC_VOLT_SEL_1325MV      (0x3du)

#else

    #error Unsupported EVM !!

#endif

/*
** Structure for OPP Configuration
*/
typedef struct oppConfig
{
    unsigned int pllMult;
    unsigned int pmicVolt;
} tOPPConfig;

/******************************************************************************
**                    External Function Declararions 
*******************************************************************************/

extern void BlPlatformConfigPostBoot( void );
extern void BlPlatformConfig(void);
#if defined(SPI)
    extern void BlPlatformSPISetup(void);
    extern void BL_PLATFORM_SPISetup(void);
    extern unsigned int BlPlatformSPIImageCopy();
#elif defined(MMCSD)
    extern void BlPlatformMMCSDSetup(void);
    extern void BL_PLATFORM_MMCSDSetup(void);
    extern unsigned int BlPlatformMMCSDImageCopy();
#elif defined(NAND)
    extern void BlPlatformNANDSetup(void);
    extern unsigned int BlPlatformNANDImageCopy(NandInfo_t *nandInfo);
    extern void BlPlatformNANDSetup(void);
    extern void BlPlatformNANDInfoInit(NandInfo_t *nandInfo);

#endif

#endif /* _BL_PLATFORM_H__ */

