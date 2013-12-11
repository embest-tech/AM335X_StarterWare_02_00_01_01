/**
 * \file  mcaspPlayBk.c
 *
 * \brief McASP example application
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

#include "edma_event.h" 
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "codecif.h"
#include "mcasp.h"
#include "aic31.h"
#include "edma.h"
#include "uartStdio.h"

#include <string.h>

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
******************************************************************************/
/*
** Values which are configurable
*/
/* Slot size to send/receive data */
#define SLOT_SIZE                             (16u)


#define I2S_SLOTS_L_R                 (0x03u)


/* Word size to send/receive data. Word size <= Slot size */
#define WORD_SIZE                             (16u)

/* Sampling Rate which will be used by both transmit and receive sections */
#define SAMPLING_RATE                         (48000u)

/* Number of channels, L & R */
#define NUM_I2S_CHANNELS                      (2u) 

/* Number of samples to be used per audio buffer */
#define NUM_SAMPLES_PER_AUDIO_BUF             (2000u)

/* Number of buffers used per tx/rx */
#define NUM_BUF                               (3u)

/* Number of linked parameter set used per tx/rx */
#define NUM_PAR                               (6u)


/* Number of samples in loop buffer */
#define NUM_SAMPLES_LOOP_BUF                  (10u)

/* AIC3106 codec address */
#define I2C_SLAVE_CODEC_AIC31                 (0x1Bu) 

/* Interrupt channels to map in AINTC */
#define INT_CHANNEL_I2C                       (2u)
#define INT_CHANNEL_MCASP                     (2u)
#define INT_CHANNEL_EDMACC                    (2u)

/* McASP Serializer for Transmit */
#define MCASP_XSER_TX                         (2u)

/*
** Below Macros are calculated based on the above inputs
*/
#define NUM_TX_SERIALIZERS                    ((NUM_I2S_CHANNELS >> 1) \
                                               + (NUM_I2S_CHANNELS & 0x01))
#define I2S_SLOTS                             ((1 << NUM_I2S_CHANNELS) - 1)

#define BYTES_PER_SAMPLE                      ((WORD_SIZE >> 3) \
                                               * NUM_I2S_CHANNELS)

#define AUDIO_BUF_SIZE                        (NUM_SAMPLES_PER_AUDIO_BUF \
                                               * BYTES_PER_SAMPLE)

#define TX_DMA_INT_ENABLE                     (EDMA3CC_OPT_TCC_SET(EDMA3_CHA_MCASP1_TX) | (1 \
                                               << EDMA3CC_OPT_TCINTEN_SHIFT))

#define PAR_TX_START                          (70 + NUM_PAR)

/*
** Definitions which are not configurable 
*/
#define SIZE_PARAMSET                         (32u)
#define OPT_FIFO_WIDTH                        (0x02 << 8u)

//#define TONE_NUM_BYTES                        (sizeof(toneRaw))

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
void SoundClickPlay(unsigned char* toneBase, unsigned int toneSize);
static void AIC31I2SConfigure(void);
static void McASPI2SConfigure(void);
static void McASPTxDMAComplHandler(void);
static void EDMA3CCComplIsr(void);
static void I2SDataTxRxActivate(void);
static void I2SDMAParamInit(void);
static void ParamTxLoopJobSet(unsigned short parId);
static void BufferTxDMAActivate(unsigned int txBuf, unsigned short numSamples,
                                unsigned short parToUpdate, 
                                unsigned short linkAddr);

extern unsigned short const toneRaw;
/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
******************************************************************************/
static unsigned char loopBuf[NUM_SAMPLES_LOOP_BUF * BYTES_PER_SAMPLE] = {0};

/*
** Transmit buffers. If any new buffer is to be added, define it here and 
** update the NUM_BUF.
*/
static unsigned char txBuf0[AUDIO_BUF_SIZE];
static unsigned char txBuf1[AUDIO_BUF_SIZE];
static unsigned char txBuf2[AUDIO_BUF_SIZE];

/*
** The offset of the paRAM ID sent, from starting of the paRAM set.
*/
static volatile unsigned short parOffSent = NUM_PAR - 1;

/*
** The offset of the paRAM ID to be sent next, from starting of the paRAM set. 
*/
static volatile unsigned short parOffTxToSend = 0;

/*
** The transmit buffer which was sent last.
*/
static volatile unsigned int lastSentTxBuf = NUM_BUF - 1;

/******************************************************************************
**                      INTERNAL CONSTATNT DEFINITIONS
******************************************************************************/
/*
** Default paRAM for Transmit section. This will be transmitting from 
** a loop buffer.
*/
static struct EDMA3CCPaRAMEntry const txDefaultPar = 
       {
           (unsigned int)(0x02 << 8u), /* Opt field */
           (unsigned int)loopBuf, /* source address */
           (unsigned short)(BYTES_PER_SAMPLE), /* aCnt */
           (unsigned short)(NUM_SAMPLES_LOOP_BUF), /* bCnt */ 
           (unsigned int) SOC_MCASP_1_DATA_REGS, /* dest address */
           (short) (BYTES_PER_SAMPLE), /* source bIdx */
           (short)(0), /* dest bIdx */
           (unsigned short)(PAR_TX_START * SIZE_PARAMSET), /* link address */
           (unsigned short)(0), /* bCnt reload value */
           (short)(0), /* source cIdx */
           (short)(0), /* dest cIdx */
           (unsigned short)1 /* cCnt */
       };

/* Array of transmit buffer pointers */
static unsigned int const txBufPtr[NUM_BUF] =
       { 
           (unsigned int) txBuf0,
           (unsigned int) txBuf1,
           (unsigned int) txBuf2
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

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, parId, &paramSet);
}

/*
** Initializes the DMA parameters.
*/
static void I2SDMAParamInit(void)
{
    EDMA3CCPaRAMEntry paramSet;
    int idx; 
 
    /* Initialize the 1st paRAM set for transmit */ 
    memcpy(&paramSet, &txDefaultPar, SIZE_PARAMSET);

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MCASP1_TX, &paramSet);

    /* rest of the params, enable loop job */
    for(idx = 0 ; idx < NUM_PAR; idx++)
    {
        ParamTxLoopJobSet(PAR_TX_START + idx);
    }
 
    /* Initialize the variables for transmit */
    parOffSent = NUM_PAR - 1;
    lastSentTxBuf = NUM_BUF - 1; 
}

/*
** Function to configure the codec for I2S mode
*/
static void AIC31I2SConfigure(void)
{
    volatile unsigned int delay = 0xFFF;

    AIC31Reset(SOC_I2C_1_REGS);
    while(delay--);

    /* Configure the data format and sampling rate */
    AIC31DataConfig(SOC_I2C_1_REGS, AIC31_DATATYPE_I2S, SLOT_SIZE, 0);
    AIC31SampleRateConfig(SOC_I2C_1_REGS, AIC31_MODE_BOTH, SAMPLING_RATE);

    /* Initialize both ADC and DAC */
    AIC31ADCInit(SOC_I2C_1_REGS);
    AIC31DACInit(SOC_I2C_1_REGS);
}

/*
** Configures the McASP Transmit Section in I2S mode.
*/
static void McASPI2SConfigure(void)
{
    McASPTxReset(SOC_MCASP_1_CTRL_REGS);

    /* Enable the FIFOs for DMA transfer */
    McASPWriteFifoEnable(SOC_MCASP_1_FIFO_REGS, 1, 1);

    /* Set I2S format in the transmitter/receiver format units */
    McASPTxFmtI2SSet(SOC_MCASP_1_CTRL_REGS, WORD_SIZE, SLOT_SIZE,
                     MCASP_TX_MODE_DMA);

    McASPTxFrameSyncCfg(SOC_MCASP_1_CTRL_REGS, 2, MCASP_TX_FS_WIDTH_WORD, 
                        MCASP_TX_FS_EXT_BEGIN_ON_RIS_EDGE);

    /* configure the clock for transmitter */
    McASPTxClkCfg(SOC_MCASP_1_CTRL_REGS, MCASP_TX_CLK_EXTERNAL, 0, 0);
    McASPTxClkPolaritySet(SOC_MCASP_1_CTRL_REGS, MCASP_TX_CLK_POL_FALL_EDGE); 
    McASPTxClkCheckConfig(SOC_MCASP_1_CTRL_REGS, MCASP_TX_CLKCHCK_DIV32,
                          0x00, 0xFF);
 
    /* Enable the transmitter/receiver slots. I2S uses 2 slots */
    McASPTxTimeSlotSet(SOC_MCASP_1_CTRL_REGS, I2S_SLOTS_L_R);

    /* Set the serializer as transmitter */
    McASPSerializerTxSet(SOC_MCASP_1_CTRL_REGS, MCASP_XSER_TX);

    /*
    ** Configure the McASP pins 
    ** Input - Frame Sync, Clock and Serializer 12
    ** Output - Serializer 11 is connected to the input of the codec 
    */
    McASPPinMcASPSet(SOC_MCASP_1_CTRL_REGS, 0xFFFFFFFF);
    McASPPinDirOutputSet(SOC_MCASP_1_CTRL_REGS, MCASP_PIN_AXR(MCASP_XSER_TX));
    McASPPinDirInputSet(SOC_MCASP_1_CTRL_REGS, MCASP_PIN_AFSX 
                                               | MCASP_PIN_ACLKX);

}

/*
** Sets up the interrupts for EDMA in AINTC
*/
static void EDMA3IntSetup(void)
{
    IntRegister(SYS_INT_EDMACOMPINT, EDMA3CCComplIsr);

    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);
}


/*
** Activates the data transmission/reception
** The DMA parameters shall be ready before calling this function.
*/
static void I2SDataTxRxActivate(void)
{
    /* Start the clocks */
    McASPTxClkStart(SOC_MCASP_1_CTRL_REGS, MCASP_TX_CLK_EXTERNAL);

    /* Enable EDMA for the transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, 
                        EDMA3_CHA_MCASP1_TX, EDMA3_TRIG_MODE_EVENT);

    /* Activate the  serializers */
    McASPTxSerActivate(SOC_MCASP_1_CTRL_REGS);

    /* make sure that the XDATA bit is cleared to zero */
    while(McASPTxStatusGet(SOC_MCASP_1_CTRL_REGS) & MCASP_TX_STAT_DATAREADY);

    /* Activate the state machines */
    McASPTxEnable(SOC_MCASP_1_CTRL_REGS);
}

/*
** Activates the DMA transfer for a parameterset from the given buffer.
*/
void BufferTxDMAActivate(unsigned int txBuf, unsigned short numSamples,
                         unsigned short parId, unsigned short linkPar)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Copy the default paramset */
    memcpy(&paramSet, &txDefaultPar, SIZE_PARAMSET - 2);
    
    /* Enable completion interrupt */
    paramSet.opt |= TX_DMA_INT_ENABLE;
    paramSet.srcAddr =  txBufPtr[txBuf];
    paramSet.linkAddr = linkPar * SIZE_PARAMSET;  
    paramSet.bCnt = numSamples;

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, parId, &paramSet);
}

/* Initializes the sound module */
void SoundInit(void)
{
    /* Set up pin mux for I2C module 1 and McASP module 1 */
    I2C1ModuleClkConfig();
    I2CPinMuxSetup(1);

    McASP1ModuleClkConfig();
    McASP1PinMuxSetup();

    EDMAModuleClkConfig();

    /* Initialize the I2C 1 interface for the codec AIC31 */
    I2CCodecIfInit(SOC_I2C_1_REGS, I2C_SLAVE_CODEC_AIC31);

    EDMA3Init(SOC_EDMA30CC_0_REGS, 0);
    EDMA3IntSetup(); 

    /*
    ** Request EDMA channels.
    */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_CHA_MCASP1_TX, EDMA3_CHA_MCASP1_TX, 0);

    /* Initialize the DMA parameters */
    I2SDMAParamInit();

    /* Configure the Codec for I2S mode */
    AIC31I2SConfigure();

    /* Configure the McASP for I2S */
    McASPI2SConfigure();
  
    /* Activate the audio transmission and reception */ 
    I2SDataTxRxActivate();
}
 
/*
** When called, plays a tone
*/
void SoundClickPlay(unsigned char* toneBase, unsigned int toneSize)
{
    unsigned short parToSend;
    unsigned short parToLink;
    unsigned int toneOffset;
    unsigned int len;
 
    toneOffset = 0;       

    while(toneOffset < toneSize)
    {
        parToSend =  PAR_TX_START + parOffTxToSend;
        parOffTxToSend = (parOffTxToSend + 1) % NUM_PAR;
        parToLink  = PAR_TX_START + parOffTxToSend; 
 
        lastSentTxBuf = (lastSentTxBuf + 1) % NUM_BUF;

        len = toneSize - toneOffset;

        if(len > AUDIO_BUF_SIZE) 
        {
            len = AUDIO_BUF_SIZE;
        }

        /* Copy the buffer */
        memcpy((void *)txBufPtr[lastSentTxBuf],
               (void *)(toneBase + toneOffset),
               len);

        toneOffset += len;

        /*
        ** Send the buffer by setting the DMA params accordingly.
        */
        BufferTxDMAActivate(lastSentTxBuf, 
                            (len / ((WORD_SIZE >> 3) * NUM_I2S_CHANNELS)),
                            (unsigned short)parToSend,
                            (unsigned short)parToLink);
    }
}  


/*
** This function will be called once transmit DMA is completed
*/
static void McASPTxDMAComplHandler(void)
{
    parOffSent = (parOffSent + 1) % NUM_PAR;
    ParamTxLoopJobSet((unsigned short)(PAR_TX_START + parOffSent));
}

/*
** EDMA transfer completion ISR
*/
static void EDMA3CCComplIsr(void) 
{
    /* Check if transmit DMA completed */
    if(EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS) & (1 << EDMA3_CHA_MCASP1_TX)) 
    { 
        /* Clear the interrupt status for the first channel */
        EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MCASP1_TX); 
        McASPTxDMAComplHandler();
    }
}

/***************************** End Of File ***********************************/
