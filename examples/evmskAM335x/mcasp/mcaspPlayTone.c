/**
 * \file  mcaspPlayTone.c
 *
 * \brief Sample application for McASP. This application plays a tone 
 *        which is part of the application.
 *        Application Configuration:
 *
 *            Modules Used:
 *                McASP1
 *                UART0
 *                EDMA0
 *                I2C0
 *
 *            Configurable parameters:
 *                None.
 *
 *            Hard-coded configuration of other parameters:
 *                1) Sampling Rate - 44100
 *                2) Word width - 16
 *                3) Slot Width - 16
 *
 *        Application Use Case:
 *            A stored audio tone will be ouput on LINE OUT of the EVM
 *
 *        Running the example:
 *            1) Before execution plug a 3.5mm audio jack which is connected
 *               to a headphone or speakers into the audio LINE OUT of the EVM
 *            2) On execution of the example a stored audio tone will be
 *               output on the LINE OUT
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

#include "consoleUtils.h"
#include "evmskAM335x.h"
#include "edma_event.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "codecif.h"
#include "toneRaw.h"
#include <string.h>
#include "mcasp.h"
#include "aic31.h"
#include "cache.h"
#include "hsi2c.h"
#include "edma.h"
#include "mmu.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
******************************************************************************/
/* I2S Time Slot Mask. */
#define I2S_SLOTS_L_R                 (0x03u)

/* McASP Serializer for Transmit */
#define MCASP_XSER_TX                 (2u)

/*
** Configurable parameters for the Audio Tone
*/
#define TONE_WORD_SIZE                (16u)
#define TONE_SLOT_SIZE                (16u)
#define TONE_SAMPLE_RATE              (44100u)

/* Number of linked parameter set used per tx/rx */
#define NUM_PAR                       (2u)

/* Number of channels, L & R */
#define NUM_I2S_CHANNELS              (2u)

/* AIC3106 codec address */
#define I2C_SLAVE_CODEC_AIC31         (0x1Bu)

/* Specify where the parameter set starting is */
#define PAR_ID_START                  (70u)

/*
** Below Macros are calculated based on the above inputs
*/
#define PAR_RX_START                  (PAR_ID_START)
#define PAR_TX_START                  (PAR_RX_START + NUM_PAR)
#define BYTES_PER_SAMPLE              ((TONE_WORD_SIZE >> 3) \
                                      * NUM_I2S_CHANNELS)

/*
** Definitions which are not configurable
*/
#define SIZE_PARAMSET                 (32u)
#define OPT_FIFO_WIDTH                (0x02 << 8u)


/* Number of samples in loop buffer */
#define NUM_SAMPLES_LOOP_BUF          (10u)

/* Macros used for MMU Configuration. */
#define START_ADDR_DDR                (0x80000000)
#define NUM_SECTIONS_DDR              (512)
#define START_ADDR_OCMC               (0x40300000)
#define NUM_SECTIONS_OCMC             (1)
#define START_ADDR_DEV                (0x44000000)
#define NUM_SECTIONS_DEV              (960)

/* Definitions for sample tone */
#define TONE_START_ADDR               ((unsigned int)toneRaw)
#define TONE_NUM_BYTES                (sizeof(toneRaw))
#define TONE_END_ADDR                 (TONE_START_ADDR + TONE_NUM_BYTES - 1)
#define PARAM1_NUM_SAMPLES_L          ((unsigned int)(TONE_NUM_BYTES  \
                                       / (TONE_WORD_SIZE >> 3)))
#define PARAM1_BCNT                   (65000)
#define PARAM1_CCNT                   ((unsigned int) \
                                       (PARAM1_NUM_SAMPLES_L / PARAM1_BCNT))
#define PARAM2_START_ADDR             (TONE_START_ADDR + (PARAM1_CCNT * \
                                       (TONE_WORD_SIZE >> 3) * PARAM1_BCNT))
#define PARAM2_BCNT                   (((TONE_END_ADDR - PARAM2_START_ADDR) \
                                        / (TONE_WORD_SIZE >> 3)) + 1)

/* McASP Instance related macros. */
#define MCASP_INST_BASE               (SOC_MCASP_1_CTRL_REGS)
#define MCASP_DATA_REGS               (SOC_MCASP_1_DATA_REGS)
#define MCASP_FIFO_REGS               (SOC_MCASP_1_FIFO_REGS)
#define MCASP_TX_INT                  (SYS_INT_MCATXINT1)

/* EDMA Related Macros. */
#define EDMA_INST_BASE                (SOC_EDMA30CC_0_REGS)
#define EDMA_CHANNEL_IN_USE           (EDMA3_CHA_MCASP1_TX)
#define EDMA_COMPLTN_INT_NUM          (SYS_INT_EDMACOMPINT)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void McASPTxDMAComplHandler(void);
static void McASPI2STwoChanConfig(void);
static void AudioTxActivate(void);
static void EDMA3CCComplIsr(void);
static void AudioCodecInit(void);
static void McASPIntSetup(void);
static void EDMA3IntSetup(void);
static void ToneLoopInit(void);
static void McASPTxIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
******************************************************************************/
/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/*
** The offset of the paRAM ID sent, from starting of the paRAM set.
*/
static volatile unsigned short parOffSent = 0;
static unsigned char loopBuf[NUM_SAMPLES_LOOP_BUF * BYTES_PER_SAMPLE] = {0};

static struct EDMA3CCPaRAMEntry dmaPar[3] = {
       {
           (unsigned int)((0x02 << 8u)),
           (unsigned int)TONE_START_ADDR,
           (unsigned short)(TONE_WORD_SIZE >> 3),
           (unsigned short)PARAM1_BCNT,
           (unsigned int) MCASP_DATA_REGS,
           (short) (TONE_WORD_SIZE >> 3),
           (short)0x00,
           (unsigned short)(32u * 70u),
           (unsigned short)PARAM1_BCNT,
           (short)(TONE_WORD_SIZE >> 3),
           (short)0x00,
           (unsigned short)PARAM1_CCNT
       },
       {
           (unsigned int)((0x02 << 8u)),
           (unsigned int)(PARAM2_START_ADDR),
           (unsigned short)(TONE_WORD_SIZE >> 3),
           0,
           (unsigned int) MCASP_DATA_REGS,
           (short)(TONE_WORD_SIZE >> 3),
           (short)0x00,
           (unsigned short)(32u * 71u),
           (unsigned short)0,
           (short)0x00,
           (short)0x00,
           (unsigned short)(1u)
       },
       {
           (unsigned int)((0x02 << 8u)),
           (unsigned int)TONE_START_ADDR,
           (unsigned short)(TONE_WORD_SIZE >> 3),
           (unsigned short)PARAM1_BCNT,
           (unsigned int) MCASP_DATA_REGS,
           (short) (TONE_WORD_SIZE >> 3),
           (short)0x00,
           (unsigned short)(32u * 70u),
           (unsigned short)PARAM1_BCNT,
           (short)(TONE_WORD_SIZE >> 3),
           (short)0x00,
           (unsigned short)PARAM1_CCNT
       }
};

/******************************************************************************
**                      INTERNAL CONSTANT DEFINITIONS
******************************************************************************/
/*
** Default paRAM for Transmit section. This will be transmitting from
** a loop buffer.
*/
static struct EDMA3CCPaRAMEntry const txDefaultPar =
       {
           (unsigned int)(OPT_FIFO_WIDTH), /* Opt field */
           (unsigned int)loopBuf, /* source address */
           (unsigned short)(BYTES_PER_SAMPLE), /* aCnt */
           (unsigned short)(NUM_SAMPLES_LOOP_BUF), /* bCnt */
           (unsigned int) MCASP_DATA_REGS, /* dest address */
           (short) (BYTES_PER_SAMPLE), /* source bIdx */
           (short)(0), /* dest bIdx */
           (unsigned short)(PAR_TX_START * SIZE_PARAMSET), /* link address */
           (unsigned short)(0), /* bCnt reload value */
           (short)(0), /* source cIdx */
           (short)(0), /* dest cIdx */
           (unsigned short)1 /* cCnt */
       };

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/
/*
** Assigns loop job for a parameter set
*/
static void ParamTxLoopJobSet(unsigned short parId)
{
    EDMA3CCPaRAMEntry paramSet;

    memcpy(&paramSet, &txDefaultPar, SIZE_PARAMSET - 2);

    /* link the paRAM to itself */
    paramSet.linkAddr = parId * SIZE_PARAMSET;

    EDMA3SetPaRAM(EDMA_INST_BASE, parId, &paramSet);
}

/*
** This function will setup the MMU. The function maps three regions -
** 1. DDR
** 2. OCMC RAM
** 3. Device memory
** The function also enables the MMU.
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
    /* Setup the MMU and do necessary MMU configurations. */
    MMUConfigAndEnable();

    /* Enable all levels of CACHE. */
    CacheEnable(CACHE_ALL);

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    ConsoleUtilsPrintf("McASP Example Application. ");
    ConsoleUtilsPrintf("Please connect headphone/speaker to the LINE OUT");
    ConsoleUtilsPrintf(" of the EVM to listen to the audio tone.");

    /* Enable the module clock for I2C0 Instance. */
    I2C0ModuleClkConfig();

    /* Set up the pin mux for I2C0 instance. */
    I2CPinMuxSetup(0);

    /* Enable the module clock for McASP1 Instance. */
    McASP1ModuleClkConfig();

    /* Enable pin-mux for McASP1 instance. */
    McASP1PinMuxSetup();

    /* Enable the EDMA module clocks. */
    EDMAModuleClkConfig();

    /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

    /* Enable EDMA Interrupt. */
    EDMA3IntSetup(); 

    /* Enable McASP Interrupt. */
    McASPIntSetup();
  
    /* Initialize the I2C interface for the codec AIC31 */
    I2CCodecIfInit(I2C_INST_BASE, I2C_SLAVE_CODEC_AIC31);

    /* Enable the interrupts generation at global level */ 
    IntMasterIRQEnable();

    /* Configure the Codec for I2S mode */
    AudioCodecInit();

    /* Initialize the looping of tone. */
    ToneLoopInit(); 

    /* Start playing tone. */
    AudioTxActivate();

    while(1);
}

/*
** Sets up the interrupts for EDMA in AINTC
*/
static void EDMA3IntSetup(void)
{
    IntRegister(EDMA_COMPLTN_INT_NUM, EDMA3CCComplIsr);

    IntPrioritySet(EDMA_COMPLTN_INT_NUM, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(EDMA_COMPLTN_INT_NUM);
}

/*
** Sets up the interrupts for McASP in AINTC
*/
static void McASPIntSetup(void)
{
    IntRegister(MCASP_TX_INT, McASPTxIsr);

    IntPrioritySet(MCASP_TX_INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(MCASP_TX_INT);

}

/*
** This function will be called once transmit DMA is completed
*/
static void McASPTxDMAComplHandler(void)
{
    ParamTxLoopJobSet((unsigned short)(PAR_TX_START + parOffSent));

    parOffSent = (parOffSent + 1) % NUM_PAR;
}

/*
** EDMA transfer completion ISR
*/
static void EDMA3CCComplIsr(void)
{
    /* Check if transmit DMA completed */
    if(EDMA3GetIntrStatus(EDMA_INST_BASE) & (1 << EDMA_CHANNEL_IN_USE))
    {
        /* Clear the interrupt status for the first channel */
        EDMA3ClrIntr(EDMA_INST_BASE, EDMA_CHANNEL_IN_USE);
        McASPTxDMAComplHandler();
    }
}

/*
** ISR for McASP TX section
*/
static void McASPTxIsr(void)
{
    ; /* Perform any error handling here.*/
}

/*
** Function to initialize the audio codec
*/
static void AudioCodecInit(void)
{
    volatile unsigned int delay = 0xFFF;

    /* Initialize the I2C interface for the codec AIC31 */
    I2CMasterSlaveAddrSet(I2C_INST_BASE, I2C_SLAVE_CODEC_AIC31);

    AIC31Reset(I2C_INST_BASE);
    while(delay--);

    /* Configure the data format and sampling rate */
    AIC31DataConfig(I2C_INST_BASE, AIC31_DATATYPE_I2S, TONE_SLOT_SIZE, 0);
    AIC31SampleRateConfig(I2C_INST_BASE, AIC31_MODE_BOTH, TONE_SAMPLE_RATE);

    /* Initialize both ADC and DAC */
    AIC31ADCInit(I2C_INST_BASE);
    AIC31DACInit(I2C_INST_BASE);
}

/*
** Function to initialize the looping of tone.
*/
static void ToneLoopInit(void)
{
    EDMA3Init(EDMA_INST_BASE, 0);

    EDMA3RequestChannel(EDMA_INST_BASE, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA_CHANNEL_IN_USE, EDMA_CHANNEL_IN_USE, 0);

    /*
    ** To maintain portability, between GCC and TMS470, we initialize the
    ** BCNT for dmaPar[1] here. Else if the initialization is done above,
    ** TMS470 throws up error: expression must have a constant value
    */
    dmaPar[1].bCnt = PARAM2_BCNT;

    /* Initialize the DMA parameters */
    EDMA3SetPaRAM(EDMA_INST_BASE, EDMA_CHANNEL_IN_USE,
                 (struct EDMA3CCPaRAMEntry *)(&(dmaPar[0])));
    EDMA3SetPaRAM(EDMA_INST_BASE, 70,
                 (struct EDMA3CCPaRAMEntry *)(&(dmaPar[1])));
    EDMA3SetPaRAM(EDMA_INST_BASE, 71,
                 (struct EDMA3CCPaRAMEntry *)(&(dmaPar[2])));

    /* Configure the McASP for I2S with two channels */
    McASPI2STwoChanConfig();
}

/*
** Activates the data transmission/reception
** The DMA parameters shall be ready before calling this function.
*/
static void AudioTxActivate(void)
{
    /* Start the clocks */
    McASPTxClkStart(MCASP_INST_BASE, MCASP_TX_CLK_EXTERNAL);

    /* Enable EDMA for the transfer */
    EDMA3EnableTransfer(EDMA_INST_BASE, EDMA_CHANNEL_IN_USE,
                        EDMA3_TRIG_MODE_EVENT);

    /* Activate the  serializers */
    McASPTxSerActivate(MCASP_INST_BASE);

    /* Activate the state machines */
    McASPTxEnable(MCASP_INST_BASE);
}

/*
** Configures the McASP Transmit Section for 2 channels in I2S mode.
*/
static void McASPI2STwoChanConfig(void)
{
    McASPTxReset(MCASP_INST_BASE);

    /* Enable the FIFOs for DMA transfer */
    McASPWriteFifoEnable(MCASP_FIFO_REGS, 1, 1);

    /* Set I2S format in the transmitter/receiver format units */
    McASPTxFmtI2SSet(MCASP_INST_BASE, TONE_WORD_SIZE, TONE_SLOT_SIZE,
                     MCASP_TX_MODE_DMA);

    McASPTxFrameSyncCfg(MCASP_INST_BASE, 2, MCASP_TX_FS_WIDTH_WORD,
                        MCASP_TX_FS_EXT_BEGIN_ON_RIS_EDGE);

    /* configure the clock for transmitter */
    McASPTxClkCfg(MCASP_INST_BASE, MCASP_TX_CLK_EXTERNAL, 0, 0);
    McASPTxClkPolaritySet(MCASP_INST_BASE, MCASP_TX_CLK_POL_FALL_EDGE);
    McASPTxClkCheckConfig(MCASP_INST_BASE, MCASP_TX_CLKCHCK_DIV32,
                          0x00, 0xFF);

    /* Enable the transmitter/receiver slots. I2S uses 2 slots */
    McASPTxTimeSlotSet(MCASP_INST_BASE, I2S_SLOTS_L_R);

    /* Set the serializer as transmitter */
    McASPSerializerTxSet(MCASP_INST_BASE, MCASP_XSER_TX);

    /*
    ** Configure the McASP pins
    ** Input - Frame Sync, Clock and Serializer 12
    ** Output - Serializer 11 is connected to the input of the codec
    */
    McASPPinMcASPSet(MCASP_INST_BASE, 0xFFFFFFFF);
    McASPPinDirOutputSet(MCASP_INST_BASE, MCASP_PIN_AXR(MCASP_XSER_TX));
    McASPPinDirInputSet(MCASP_INST_BASE, MCASP_PIN_AFSX
                                               | MCASP_PIN_ACLKX);
}
