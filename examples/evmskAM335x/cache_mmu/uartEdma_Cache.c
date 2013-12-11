/**
 * \file   uartEdma_Cache.c
 *
 * \brief  This is a sample application file which invokes some APIs
 *         from the EDMA3 device abstraction layer as well as UART 
 *         device abstraction layer to perform configuration and 
 *         transfer of data between UART and RAM by the use of
 *         EDMA3 with Cache and MMU being enabled.
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

#include "uart_irda_cir.h"
#include "evmskAM335x.h"
#include "edma_event.h"
#include "soc_AM335x.h"
#include "uartStdio.h"
#include "interrupt.h"
#include "hw_types.h"
#include "cache.h"
#include "edma.h"
#include "mmu.h"

/****************************************************************************/
/*                      INTERNAL MACRO DEFINITIONS                          */
/****************************************************************************/
#define UART_THR_RHR_REG           (SOC_UART_0_REGS)

#define TX_BUFFER_SIZE             (26u)

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM              (0u)

/* PaRAM Set number for Dummy Transfer. */
#define DUMMY_CH_NUM               (5u)

/* UART Module Input Frequency. */
#define UART_MODULE_INPUT_CLK      (48000000u)

/* Baud Rate of UART for communication. */
#define BAUD_RATE_115200           (115200u)

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR             (0x80000000)
#define START_ADDR_DEV             (0x44000000)
#define START_ADDR_OCMC            (0x40300000)
#define NUM_SECTIONS_DDR           (512)
#define NUM_SECTIONS_DEV           (960)
#define NUM_SECTIONS_OCMC          (1)

/* Wrapper Definitions. */
#define UART_INSTANCE_BASE_ADD    (SOC_UART_0_REGS)
#define EDMA3_INSTANCE_BASE_ADD   (SOC_EDMA30CC_0_REGS)
#define EDMA3_UART_TX_CHA_NUM     (EDMA3_CHA_UART0_TX)

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
static void UartEDMATxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *buffer,
                                   unsigned short buffLength);
static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);
static void callback(unsigned int tccNum, unsigned int status);
static void TxDummyPaRAMConfEnable(void);
static void MMUConfigAndEnable(void);
static void Edma3CompletionIsr(void);
static void EDMA3INTCConfigure(void);
static void UartBaudRateSet(void);
static void Edma3CCErrorIsr(void);
static void EDMA3Initialize(void);
static void UARTInitialize(void);

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
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

volatile unsigned char buffer[TX_BUFFER_SIZE];
volatile unsigned int clBackFlag = 0;
volatile unsigned char dummyVal;

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
    unsigned char alphabet = 'a';
    unsigned int index;

    MMUConfigAndEnable();

    /* populate the buffer which is used by DMA for data transfer */
    for(index = 0; index < TX_BUFFER_SIZE ; index++)
    {
         buffer[index] = alphabet++;
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
    EDMA3RequestChannel(EDMA3_INSTANCE_BASE_ADD, EDMA3_CHANNEL_TYPE_DMA, 
                        EDMA3_UART_TX_CHA_NUM, EDMA3_UART_TX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for TX*/
    cb_Fxn[EDMA3_UART_TX_CHA_NUM] = &callback; 

    UARTPuts("The buffer contents in Main Memory.\r\n", -1);

    /* Transmit Data for Enter Message */
    UartEDMATxConfTransfer(EDMA3_UART_TX_CHA_NUM,
                           EDMA3_UART_TX_CHA_NUM,
                           buffer,
                           TX_BUFFER_SIZE);

    /* Wait for return from callback */     
    while(0 == clBackFlag);
    clBackFlag = 0;

    alphabet = 'A';

    /* Populate the buffer which is used by DMA for data transfer */
    for(index = 0; index < TX_BUFFER_SIZE; index++)
    {
         buffer[index] = alphabet++;
    }

    /* Clean cache to achieve coherence between cached memory and main memory */
    #ifdef CACHE_CLEAN

    CacheDataCleanBuff((unsigned int)buffer, 27);

    #endif
    
    /* Transmit Data for Entered value */
    UartEDMATxConfTransfer(EDMA3_UART_TX_CHA_NUM, EDMA3_UART_TX_CHA_NUM, 
                     buffer, TX_BUFFER_SIZE);

    #ifdef CACHE_CLEAN

    UARTPuts("\r\n\r\nThe buffer contents in main memory after the Cache", -1);
    UARTPuts(" has been cleaned.\r\n", -1);

    #else

    UARTPuts("\r\n\r\nThe buffer contents in main memory when the Cache", -1);
    UARTPuts(" has not been cleaned.\r\n", -1);

    #endif

    /* Enabling UART in DMA Mode*/
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_3_ENABLE);

    /* Wait for return from callback */     
    while(0 == clBackFlag); 
    clBackFlag = 0;

    /* Free EDMA3 Channels for TX */
    EDMA3FreeChannel(EDMA3_INSTANCE_BASE_ADD, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_UART_TX_CHA_NUM, EDMA3_TRIG_MODE_EVENT, 
                     EDMA3_UART_TX_CHA_NUM, EVT_QUEUE_NUM);

    while(1);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Transmit Channel
** of UART. On the EDMA front, EDMA shall signal a green flag for data transfer.
** However, data transfer shall happen only if the DMA feature in UART is also
** enabled.
*/
static void UartEDMATxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *buffer,
                                   unsigned short buffLength)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information */
    paramSet.srcAddr = (unsigned int) buffer;
    paramSet.destAddr = UART_THR_RHR_REG;
    paramSet.aCnt = 1;
    paramSet.bCnt = buffLength;
    paramSet.cCnt = 1;

    /* The source index should increment for every byte being transferred. */
    paramSet.srcBIdx = 1u;

    /* The destination index should not increment since it is a h/w register. */
    paramSet.destBIdx = 0u;
  
    paramSet.srcCIdx = 0u;
    paramSet.destCIdx = 0u;
    paramSet.linkAddr = (unsigned short)(EDMA3CC_OPT(DUMMY_CH_NUM)); 
    paramSet.bCntReload = 0u;
    paramSet.opt = 0x00000000u;
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(EDMA3_INSTANCE_BASE_ADD, chNum, &paramSet);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(EDMA3_INSTANCE_BASE_ADD, chNum, EDMA3_TRIG_MODE_EVENT);
}

/* Function used to Initialize EDMA3 */
static void EDMA3Initialize(void)
{
    /* Initialization of EDMA3 */    
    EDMA3Init(EDMA3_INSTANCE_BASE_ADD, EVT_QUEUE_NUM);
   
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Register EDMA3 Interrupts */
    EDMA3INTCConfigure();
}

/*
** EDMA Completion Interrupt Service Routine(ISR).
*/

static void Edma3CompletionIsr(void)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIPR = 0;
    unsigned int count = 0;
    unsigned int index;

    index = 1;

    isIPR  = EDMA3GetIntrStatus(EDMA3_INSTANCE_BASE_ADD);
    if(isIPR)
    {
        while ((count < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (index != 0u))
        {
            index = 0u;
            pendingIrqs = EDMA3GetIntrStatus(EDMA3_INSTANCE_BASE_ADD);
            while (pendingIrqs)
            {
                if((pendingIrqs & 1u) == TRUE)
                {
                    /**
                    * If the user has not given any callback function
                    * while requesting the TCC, its TCC specific bit
                    * in the IPR register will NOT be cleared.
                    */
                    /* Writing to ICR to clear the corresponding IPR bits. */
                    EDMA3ClrIntr(EDMA3_INSTANCE_BASE_ADD, index);

                    (*cb_Fxn[index])(index, EDMA3_XFER_COMPLETE);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            count++;
        }
    }
}


/*
** EDMA3 Channel Controller Error Interrupt Service Routine(ISR).
*/

static void Edma3CCErrorIsr(void)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int evtqueNum = 0;
    unsigned int index = 1;
    unsigned int Cnt = 0;

    if((0 != EDMA3GetErrIntrStatus(EDMA3_INSTANCE_BASE_ADD)) ||
       (0 != (EDMA3GetCCErrStatus(EDMA3_INSTANCE_BASE_ADD))))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks
           when no pending interrupt is found */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;
            pendingIrqs = EDMA3GetErrIntrStatus(EDMA3_INSTANCE_BASE_ADD);

            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Writing to EMCR to clear the corresponding EMR bits.
                       Also clearing any Secondary events in SER. */
                    EDMA3ClrMissEvt(EDMA3_INSTANCE_BASE_ADD, index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;

            pendingIrqs = EDMA3GetCCErrStatus(EDMA3_INSTANCE_BASE_ADD);
            if (pendingIrqs != 0u)
            {
                /* Process all the pending CC error interrupts. */
                /* Queue threshold error for different event queues. */
                for (evtqueNum = 0u; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
                {
                    if((pendingIrqs & (1u << evtqueNum)) != 0u)
                    {
                        /* Clear the error interrupt. */
                        EDMA3ClrCCErr(EDMA3_INSTANCE_BASE_ADD, (1u << evtqueNum));
                    }
                }

                /* Transfer completion code error. */
                if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                {
                    EDMA3ClrCCErr(EDMA3_INSTANCE_BASE_ADD,
                                  (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT));
                }
                ++index;
            }

            Cnt++;
        }
    }
}

/*
** This function initializes the Interrupt Controller.
** It also registers, sets the priority and enables the system interrupts
** of EDMA Completion and Error.
*/

static void EDMA3INTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3CompletionIsr);

    /* Setting the priority for EDMA3CC0 completion interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3CCErrorIsr);

    /* Setting the priority for EDMA3CC0 Error interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 completion interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);

     /* Enabling the EDMA3CC0 Error interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}


/*
** This function is used as a callback from EDMA3 Completion Handler. 
*/
static void callback(unsigned int tccNum, unsigned int status)
{
    /* Disabling DMA feature in UART. */
    UARTDMADisable(UART_INSTANCE_BASE_ADD);
    clBackFlag = 1;
}

/*
** This configures the PaRAM set for the Dummy Transfer.
*/
static void TxDummyPaRAMConfEnable(void)
{
    EDMA3CCPaRAMEntry dummyPaRAMSet;

    EDMA3GetPaRAM(EDMA3_INSTANCE_BASE_ADD, DUMMY_CH_NUM, &dummyPaRAMSet);

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

    EDMA3SetPaRAM(EDMA3_INSTANCE_BASE_ADD, DUMMY_CH_NUM, &dummyPaRAMSet);
}

/*
** This function initializes the UART instance for use.
*/
static void UARTInitialize(void)
{
    /* Performing a module reset. */
    UARTModuleReset(UART_INSTANCE_BASE_ADD);

    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Performing Baud Rate settings. */
    UartBaudRateSet();

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(UART_INSTANCE_BASE_ADD, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(UART_INSTANCE_BASE_ADD,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(UART_INSTANCE_BASE_ADD);

    /* Disabling Break Control. */
    UARTBreakCtl(UART_INSTANCE_BASE_ADD, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(UART_INSTANCE_BASE_ADD, UART16x_OPER_MODE);
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
    UARTDivisorLatchWrite(UART_INSTANCE_BASE_ADD, divisorValue);

}

/********************************* End of file ******************************/
