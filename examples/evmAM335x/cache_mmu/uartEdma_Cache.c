/**
 * \file   uartEdma_Cache.c
 *
 * \brief  This is a sample application file which invokes some APIs
 *         from the EDMA3 device abstraction layer as well as UART 
 *         device abstraction layer to perform configuration, and 
 *         transfer of data between UART and CPU RAM by the 
 *         use of EDMA3 with cache and mmu being enabled.
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

#include "edma.h"
#include "edma_event.h"
#include "evmAM335x.h"
#include "string.h"
#include "hw_types.h"
#include "uartStdio.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "uart_irda_cir.h"
#include "mmu.h"
#include "cache.h"

/****************************************************************************/
/*                      INTERNAL MACRO DEFINITIONS                          */
/****************************************************************************/
#define UART_THR_RHR_REG           (SOC_UART_0_REGS)

#define MAX_ACNT                   (1u)
#define MAX_CCNT                   (1u)

#define TX_BUFFER_SIZE             (26u)

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM              (0u)

/* PaRAM Set number for Dummy Transfer. */
#define DUMMY_CH_NUM               (5u)

/* UART Module Input Frequency. */
#define UART_MODULE_INPUT_CLK      (48000000u)

/* Baud Rate of UART for communication. */
#define BAUD_RATE_115200           (115200u)

#define TX_THRESHOLD               (1)
#define RX_THRESHOLD               (1)

#define START_ADDR_DDR             (0x80000000)
#define START_ADDR_DEV             (0x44000000)
#define START_ADDR_OCMC            (0x40300000)
#define NUM_SECTIONS_DDR           (512)
#define NUM_SECTIONS_DEV           (960)
#define NUM_SECTIONS_OCMC          (1)

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
static void UartTransmitData(unsigned int tccNum, unsigned int chNum,
                             volatile unsigned char *buffer, unsigned int buffLength);
/* Callback Function Declaration*/
static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);
static void callback(unsigned int tccNum, unsigned int status);
static void ConfigureAINTCIntEDMA3(void);
static void Edma3ComplHandlerIsr(void);
static void Edma3CCErrHandlerIsr(void);
static void EDMA3Initialize(void);
static void TxDummyPaRAMConfEnable(void);
static void UARTInitialize(void);
static void UartBaudRateSet(void);
static void MMUConfigAndEnable(void);

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
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
volatile unsigned char buffer[TX_BUFFER_SIZE];
volatile unsigned char dummyVal;
volatile unsigned int flag = 0;

/****************************************************************************/
/*                   LOCAL FUNCTION DEFINITIONS                             */
/****************************************************************************/
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
 
/*
** Main function.
*/
int main(void)
{
    unsigned int index;
    unsigned char i = 'a';

    MMUConfigAndEnable();

    /* populate the buffer which is used by DMA for data transfer */
    for(index = 0; index < TX_BUFFER_SIZE ; index++)
    {
         buffer[index] = i++;
    }
 
    CacheEnable(CACHE_ALL);
 
    UARTPinMuxSetup(0);
    UART0ModuleClkConfig();

    EDMAModuleClkConfig();

    /* Initialize UART peripheral */
    UARTInitialize();

    /* Initialize EDMA3 Controller */
    EDMA3Initialize();

    /* Request DMA Channel and TCC for UART Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA, 
                        EDMA3_CHA_UART0_TX, EDMA3_CHA_UART0_TX,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for TX*/
    cb_Fxn[EDMA3_CHA_UART0_TX] = &callback; 

    /* Transmit Data for Enter Message */
    UartTransmitData(EDMA3_CHA_UART0_TX, EDMA3_CHA_UART0_TX,
                     buffer, TX_BUFFER_SIZE);

    /* Enabling UART in DMA Mode */
    UARTDMAEnable(SOC_UART_0_REGS, UART_DMA_MODE_3_ENABLE);

    /* Wait for return from callback */     
    while(0 == flag);
    flag = 0;

    i = 'A';

    /* populate the buffer which is used by DMA for data transfer */
    for(index = 0; index < TX_BUFFER_SIZE; index++)
    {
         buffer[index] = i++;
    }

    /* Clean cache to achieve coherence between cached memory and main memory */
    #ifdef CACHE_FLUSH
    CacheDataCleanBuff((unsigned int)buffer, 27);
    #endif
    
    /* Transmit Data for Entered value */
    UartTransmitData(EDMA3_CHA_UART0_TX, EDMA3_CHA_UART0_TX, 
                     buffer, TX_BUFFER_SIZE);

    /* Enabling UART in DMA Mode*/
    UARTDMAEnable(SOC_UART_0_REGS, UART_DMA_MODE_3_ENABLE);

    /* Wait for return from callback */     
    while(0 == flag); 
    flag = 0;

    /* Free EDMA3 Channels for TX */
    EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_CHA_UART0_TX, EDMA3_TRIG_MODE_EVENT, 
                     EDMA3_CHA_UART0_TX, EVT_QUEUE_NUM);

    while(1);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Transmit Channel
** of UART. EDMA3 Enable Transfer is also called within this API.
*/
static void UartTransmitData(unsigned int tccNum, unsigned int chNum,
                             volatile unsigned char *buffer, unsigned int buffLength)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information */
    paramSet.srcAddr = (unsigned int) buffer;
    paramSet.destAddr = UART_THR_RHR_REG;
    paramSet.aCnt = MAX_ACNT;
    paramSet.bCnt = (unsigned short) buffLength;
    paramSet.cCnt = MAX_CCNT;

    /* The src index should increment for every byte being transferred. */
    paramSet.srcBIdx = 1u;

    /* The dst index should not increment since it is a h/w register. */
    paramSet.destBIdx = 0u;
  
    /* A sync Transfer Mode */
    paramSet.srcCIdx = 0u;
    paramSet.destCIdx = 0u;
    paramSet.linkAddr = (unsigned short)(EDMA3CC_OPT(DUMMY_CH_NUM)); 
    paramSet.bCntReload = 0u;
    paramSet.opt = 0x00000000u;
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, chNum, EDMA3_TRIG_MODE_EVENT);
}

/* Function used to Initialize EDMA3 */
static void EDMA3Initialize(void)
{
    /* Initialization of EDMA3 */    
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEUE_NUM);
   
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Register EDMA3 Interrupts */
    ConfigureAINTCIntEDMA3();
}

/* EDMA3 Completion Handler */
static void edma3ComplHandler(unsigned int baseAdd, unsigned int regionNum)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIPR = 0;

    unsigned int indexl;
    unsigned int Cnt = 0;
    indexl = 1;
    
    isIPR = HWREG(baseAdd + EDMA3CC_S_IPR(regionNum));
    if(isIPR)
    {
        while ((Cnt < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (indexl != 0u))
        {
            indexl = 0u;
            pendingIrqs = HWREG(baseAdd + EDMA3CC_S_IPR(regionNum));
            while (pendingIrqs)
            {
                if((pendingIrqs & 1u) == TRUE)
                {
                    /**
                    * If the user has not given any callback function
                    * while requesting the TCC, its TCC specific bit
                    * in the IPR register will NOT be cleared.
                    */
                    /* here write to ICR to clear the corresponding IPR bits */
                    HWREG(baseAdd + EDMA3CC_S_ICR(regionNum)) = (1u << indexl);

                    (*cb_Fxn[indexl])(indexl, EDMA3_XFER_COMPLETE);
                }
                ++indexl;
                pendingIrqs >>= 1u;
            }
            Cnt++;
        }
    }
}

static void Edma3ComplHandlerIsr(void)
{
    /* Invoke Completion Handler ISR */
    edma3ComplHandler(SOC_EDMA30CC_0_REGS, 0);
}

/* EDMA3 Error Handler */
static void edma3CCErrHandler(unsigned int baseAdd)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int regionNum = 0;
    unsigned int evtqueNum = 0;  
    unsigned int index = 1;
    unsigned int Cnt = 0;
    
    if((HWREG(baseAdd + EDMA3CC_EMR) != 0 ) || \
       (HWREG(baseAdd + EDMA3CC_QEMR) != 0) || \
       (HWREG(baseAdd + EDMA3CC_CCERR) != 0))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks 
           when no pending interrupt is found */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;
            pendingIrqs = HWREG(baseAdd + EDMA3CC_EMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Write to EMCR to clear the corresponding EMR bits.*/
                    HWREG(baseAdd + EDMA3CC_EMCR) = (1u<<index);
                    /*Clear any SER*/
                    HWREG(baseAdd + EDMA3CC_S_SECR(regionNum)) = (1u<<index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = HWREG(baseAdd + EDMA3CC_QEMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Here write to QEMCR to clear the corresponding QEMR bits*/
                    HWREG(baseAdd + EDMA3CC_QEMCR) = (1u<<index);
                    /*Clear any QSER*/
                    HWREG(baseAdd + EDMA3CC_S_QSECR(0)) = (1u<<index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = HWREG(baseAdd + EDMA3CC_CCERR);
    if (pendingIrqs != 0u)
    {
        /* Process all the pending CC error interrupts. */
        /* Queue threshold error for different event queues.*/
        for (evtqueNum = 0u; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
        {
            if((pendingIrqs & (1u << evtqueNum)) != 0u)
            {
                /* Clear the error interrupt. */
                HWREG(baseAdd + EDMA3CC_CCERRCLR) = (1u << evtqueNum);
            }
         }

         /* Transfer completion code error. */
         if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
         {
             HWREG(baseAdd + EDMA3CC_CCERRCLR) = \
                  (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT);
         }
         ++index;
    }
    Cnt++;
        }
    }
}

static void Edma3CCErrHandlerIsr()
{
    /* Invoke CC Error Handler ISR */
    edma3CCErrHandler(SOC_EDMA30CC_0_REGS);
}

/* Function to register EDMA3 Interuppts */
static void ConfigureAINTCIntEDMA3(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3ComplHandlerIsr);

    /* Setting the priority for EDMA3CC0 completion interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 completion interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3CCErrHandlerIsr);

    /* Setting the priority for EDMA3CC0 Error interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

     /* Enabling the EDMA3CC0 Error interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}


/*
** This function is used as a callback from EDMA3 Completion Handler. 
** UART in DMA Mode is Disabled over here.
*/
static void callback(unsigned int tccNum, unsigned int status)
{
/*  UARTDMADisable(SOC_UART_0_REGS);  */
    flag = 1;
}

/*
** This configures the PaRAM set for the Dummy Transfer.
*/
static void TxDummyPaRAMConfEnable(void)
{
    EDMA3CCPaRAMEntry dummyPaRAMSet;

    EDMA3GetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);

    dummyPaRAMSet.aCnt = 1;
    dummyPaRAMSet.bCnt = 0;
    dummyPaRAMSet.cCnt = 0;
    dummyPaRAMSet.srcAddr = 0;
    dummyPaRAMSet.destAddr = 0;
    dummyPaRAMSet.srcBIdx = 0;
    dummyPaRAMSet.destBIdx = 0;
    dummyPaRAMSet.srcCIdx = 0;
    dummyPaRAMSet.destCIdx = 0;
    dummyPaRAMSet.linkAddr = 0xFFFFu;
    dummyPaRAMSet.bCntReload = 0;
    dummyPaRAMSet.opt = 0;

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);
}

/*
** This function initializes the UART0 instance for use.
*/
static void UARTInitialize(void)
{
    /* Performing a module reset. */
    UARTModuleReset(SOC_UART_0_REGS);

    UARTDMAEnable(SOC_UART_0_REGS, UART_DMA_MODE_1_ENABLE);

    /* Performing Baud Rate settings. */
    UartBaudRateSet();

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(SOC_UART_0_REGS, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(SOC_UART_0_REGS);

    /* Disabling Break Control. */
    UARTBreakCtl(SOC_UART_0_REGS, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(SOC_UART_0_REGS, UART16x_OPER_MODE);
}

/*
** A wrapper function performing Baud Rate settings.
*/
static void UartBaudRateSet(void)
{
    unsigned int divisorValue = 0;

    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         BAUD_RATE_115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(SOC_UART_0_REGS, divisorValue);

}


/********************************* End of file ******************************/
