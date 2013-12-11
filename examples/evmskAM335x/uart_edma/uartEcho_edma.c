/**
 * \file   uartEcho_edma.c
 *
 * \brief  This application demonstrates UART communication with the Host machine
 *         and using EDMA to transfer data between memory and UART FIFO.
 *         
 *         Application Configuration:
 *   
 *             Modules Used:
 *                 UART0
 *                 EDMA
 *                 Interrupt Controller
 *   
 *             Configurable Parameters:
 *                 None
 *             
 *             Hard-Coded Configuration of other parameters:
 *                 UART:
 *                 a) FIFO Mode enabled
 *                 b) Baud Rate - 115200 bps
 *                 c) Word Length - 8 bits
 *                 d) Parity - None
 *                 e) Stop Bits - 1 stop bit
 *                 f) TX DMA Threshold Programming Mode -
 *                     Direct Method (MDR3[2] = 1; TX_DMA_THRESHOLD Register
 *                     holds threshold value)
 *                 g) TX DMA Threshold level - 5
 *                 h) TX Trigger Space value - 8
 *                 i) RX Threshold level - 8
 *   
 *         Application Use Cases:
 *             1) Application demonstrates UART Transmit and Receive
 *                operations with EDMA writing data into and reading data out
 *                of the UART TX and RX FIFOs respectively.
 *             2) Demonstrates EDMA Completion Interrupt Handling and Error
 *                Interrupt Handling.
 *             3) Demonstrates EDMA Channel PaRAM Set linking to a Dummy
 *                PaRAM Set.
 *   
 *         Running the Example:
 *             On executing the example:
 *             1) Three strings will be transmitted by the application and will be
 *                displayed on the serial console of the host. These inform the
 *                user about the application and provides directions.
 *             2) The user is expected to key in 8 characters from the keyboard.
 *                The application echoes these characters at once after all the
 *                8 characters have been received.
 *                
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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
*
*/

#include "uart_irda_cir.h"
#include "evmskAM335x.h"
#include "soc_AM335x.h"
#include "edma_event.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "cache.h"
#include "edma.h"
#include "mmu.h"
#include "error.h"
#include "consoleUtils.h"

/****************************************************************************/
/*                      INTERNAL MACROS                                     */
/****************************************************************************/

/* Address of THR and RHR registers of UART. */
#define UART_THR_RHR_REG          (SOC_UART_0_REGS)

/* UART Module Input Frequency. */
#define UART_MODULE_INPUT_CLK     (48000000)

/* Baud Rate of UART for communication. */
#define BAUD_RATE_115200          (115200)
#define BAUD_RATE_128000          (128000)
#define BAUD_RATE_230400          (230400)
#define BAUD_RATE_460800          (460800)
#define BAUD_RATE_921600          (921600)

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM             (0)

/* PaRAM Set number for Dummy Transfer. */
#define DUMMY_CH_NUM              (5)

/* Wrapper Definitions. */
#define UART_INSTANCE_BASE_ADD    (SOC_UART_0_REGS)
#define EDMA3_UART_TX_CHA_NUM     (EDMA3_CHA_UART0_TX)
#define EDMA3_UART_RX_CHA_NUM     (EDMA3_CHA_UART0_RX)
#define UART_INT_NUM              (SYS_INT_UART0INT)

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR            (0x80000000u)
#define START_ADDR_DEV            (0x44000000u)
#define START_ADDR_OCMC           (0x40300000u)
#define NUM_SECTIONS_DDR          (512u)
#define NUM_SECTIONS_DEV          (960u)
#define NUM_SECTIONS_OCMC         (1u)

/* Enable FIFO mode of operation. */
#define UART_ENABLE_FIFO

/******************* Transmit related definitions  **************************/

/*
** Use this macro if TX_DMA_THRESHOLD register is used to configure TX
** Threshold value.
*/
#define DIRECT_TX_DMA_THRESH_MODE

/*
** Transmit DMA Threshold Value. This is set in TX_DMA_THRESHOLD register.
*/
#define TX_DMA_THRESHOLD          (5)

/*
** Transmit Trigger Space value. Use this if TX Trigger Level granularity
** is selected to be 1.
*/
#define TX_TRIGGER_SPACE_GRAN_1   (8)

/* Number of bytes transmitted by EDMA per TX event sent by UART. */
#ifdef UART_ENABLE_FIFO
#define TX_BYTES_PER_EVENT        (8)
#else
#define TX_BYTES_PER_EVENT        (1)
#endif

/******************* Receive related definitions  ***************************/

/* Receiver Buffer Size. */
#define RX_BUFFER_SIZE            (50)

/* Receive DMA Threshold Value. */
#ifdef UART_ENABLE_FIFO
#define RX_DMA_THRESHOLD          (8)
#else
#define RX_DMA_THRESHOLD          (1)
#endif

/* Number of bytes to be received from the user. */
#define NUM_RX_BYTES              (8)

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/
static void UARTTxEDMAPaRAMSetConfig(unsigned char *txBuffer,
                                     unsigned int length,
                                     unsigned int tccNum,
                                     unsigned short linkAddr,
                                     unsigned int chNum);
static void UARTRxEDMAPaRAMSetConfig(unsigned char *rxBuffer,
                                     unsigned int length,
                                     unsigned int tccNum,
                                     unsigned short linkAddr,
                                     unsigned int chNum);

#ifdef UART_ENABLE_FIFO
static void UartFIFOConfigure(void);
#endif

static void callback(unsigned int tccNum);
static void TxDummyPaRAMConfEnable(void);
static void EDMA3INTCConfigure(void);
static void Edma3CompletionIsr(void);
static void MMUConfigAndEnable(void);
static void UartBaudRateSet(void);
static void Edma3CCErrorIsr(void);
static void EDMA3Initialize(void);
static void UARTInitialize(void);

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
unsigned char intent[] = "The application echoes the characters that you type on the console.\r\n";
unsigned char welcome[] = "StarterWare AM335X UART DMA application.\r\n";
unsigned char enter[] = "Please Enter 08 bytes from keyboard.\r\n";

static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc);
unsigned char rxBuffer[RX_BUFFER_SIZE];
volatile unsigned int clBackFlag = 0;

/*
** Transmit Trigger Space value. This is applicable only when UART FIFO mode
** is used. Refer to the comments of the API UARTFIFOConfig() to find the
** possible values of TX Trigger Space.
*/
unsigned int txTrigSpace = TX_TRIGGER_SPACE_GRAN_1;

/*
** Number of bytes transmitted by EDMA per TX event sent by UART.
** In UART FIFO mode, this should be equal to the TX Trigger Space value.
*/
unsigned int txBytesPerEvent = TX_BYTES_PER_EVENT;

/*
** Receive DMA Thresold Level. This applies to both UART FIFO and Non-FIFO
** modes of operation. For FIFO mode, refer to the comments of the API
** UARTFIFOConfig() to find the possible values of RX Trigger Level.
** For Non-FIFO mode, this value is 1.
*/
unsigned int rxTrigLevel = RX_DMA_THRESHOLD;

/* Transmit DMA Threshold Level. This is set in TX_DMA_THRESHOLD register. */
unsigned int txThreshLevel = TX_DMA_THRESHOLD;

/* Page tables start must be aligned to 16K boundary */
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
#error "Unsupported Compiler.\r\n"

#endif

/****************************************************************************/
/*                      NOTE TO THE USER                                    */
/****************************************************************************/
/*
** 1) The application can be used with or without Transmit and Receive
**    UART FIFOs being enabled. The macro UART_ENABLE_FIFO should be defined
**    if UART FIFOs have to used.
** 2) The number of bytes transferred by EDMA to the TX FIFO per TX event
**    sent by UART should be equal to the TX Trigger Space setting in TLR
**    and/or FCR.
** 3) This application uses Direct TX DMA Threshold Programming method
**    to program the TX DMA Threshold level. Here the TX DMA Threshold level
**    is programmed in TX_DMA_THRESHOLD register of UART.
** 4) If Direct TX DMA Threshold Programming is used, the TX Trigger Space
**    setting should be greater than TX Threshold Level.
*/

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
    unsigned int numByteChunks = 0;
    unsigned char *pBuffer = NULL;
    unsigned int remainBytes = 0;

    /* Configure and enable the MMU. */
    MMUConfigAndEnable();

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);

    /* Configuring the system clocks for EDMA. */
    EDMAModuleClkConfig();

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Performing Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Initializing the EDMA. */
    EDMA3Initialize();

    /* Initializing the UART0 instance for use. */
    UARTInitialize();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /*
    ** Configuring the EDMA.
    */

    /* Request DMA Channel and TCC for UART Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_UART_TX_CHA_NUM, EDMA3_UART_TX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for TX*/
    cb_Fxn[EDMA3_UART_TX_CHA_NUM] = &callback;

    /* Request DMA Channel and TCC for UART Receive */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_UART_RX_CHA_NUM, EDMA3_UART_RX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for RX*/
    cb_Fxn[EDMA3_UART_RX_CHA_NUM] = &callback;

    /******************** Transmission of a string **************************/

    numByteChunks = (sizeof(welcome) - 1) / txBytesPerEvent;
    remainBytes = (sizeof(welcome) - 1) % txBytesPerEvent;

    /* Configuring EDMA PaRAM sets to transmit data. */
    UARTTxEDMAPaRAMSetConfig(welcome,
                             numByteChunks * txBytesPerEvent,
                             EDMA3_UART_TX_CHA_NUM,
                             EDMA3CC_OPT(DUMMY_CH_NUM),
                             EDMA3_UART_TX_CHA_NUM);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_UART_TX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /* Remaining bytes are transferred through polling method. */
    if(0 != remainBytes)
    {
        pBuffer = welcome + (sizeof(welcome) - 1) - remainBytes;
        UARTPuts((char*)pBuffer, remainBytes);
    }

    /******************** Transmission of a string **************************/

    numByteChunks = (sizeof(intent) - 1) / txBytesPerEvent;
    remainBytes = (sizeof(intent) - 1) % txBytesPerEvent;

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring EDMA PaRAM sets to transmit data. */
    UARTTxEDMAPaRAMSetConfig(intent,
                             numByteChunks * txBytesPerEvent,
                             EDMA3_UART_TX_CHA_NUM,
                             EDMA3CC_OPT(DUMMY_CH_NUM),
                             EDMA3_UART_TX_CHA_NUM);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_UART_TX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /* Remaining bytes are transferred through polling method. */
    if(0 != remainBytes)
    {
        pBuffer = intent + (sizeof(intent) - 1) - remainBytes;
        UARTPuts((char*)pBuffer, remainBytes);
    }

    /******************** Transmission of a string **************************/

    numByteChunks = (sizeof(enter) - 1) / txBytesPerEvent;
    remainBytes = (sizeof(enter) - 1) % txBytesPerEvent;

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring EDMA PaRAM sets to transmit data. */
    UARTTxEDMAPaRAMSetConfig(enter,
                             numByteChunks * txBytesPerEvent,
                             EDMA3_UART_TX_CHA_NUM,
                             EDMA3CC_OPT(DUMMY_CH_NUM),
                             EDMA3_UART_TX_CHA_NUM);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_UART_TX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /* Remaining bytes are transferred through polling method. */
    if(0 != remainBytes)
    {
        pBuffer = enter + (sizeof(enter) - 1) - remainBytes;
        UARTPuts((char*)pBuffer, remainBytes);
    }

    /********************* Receiving Data from User *************************/

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring the PaRAM set for reception. */
    UARTRxEDMAPaRAMSetConfig(rxBuffer,
                             NUM_RX_BYTES,
                             EDMA3_UART_RX_CHA_NUM,
                             0xFFFF,
                             EDMA3_UART_RX_CHA_NUM);

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_UART_RX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /******************* Echoing received bytes *****************************/

    numByteChunks = (NUM_RX_BYTES) / txBytesPerEvent;
    remainBytes = (NUM_RX_BYTES) % txBytesPerEvent;

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring EDMA PaRAM sets to transmit data. */
    UARTTxEDMAPaRAMSetConfig(rxBuffer,
                             numByteChunks * txBytesPerEvent,
                             EDMA3_UART_TX_CHA_NUM,
                             EDMA3CC_OPT(DUMMY_CH_NUM),
                             EDMA3_UART_TX_CHA_NUM);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_UART_TX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /* Remaining bytes are transferred through polling method. */
    if(0 != remainBytes)
    {
        pBuffer = rxBuffer + NUM_RX_BYTES - remainBytes;
        UARTPuts((char*)pBuffer, remainBytes);
    }

    /******************* Freeing of allocated channels **********************/

    /* Free EDMA3 Channels for TX and RX */
    EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_UART_TX_CHA_NUM, EDMA3_TRIG_MODE_EVENT,
                     EDMA3_UART_TX_CHA_NUM, EVT_QUEUE_NUM);

    EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_UART_RX_CHA_NUM, EDMA3_TRIG_MODE_EVENT,
                     EDMA3_UART_RX_CHA_NUM, EVT_QUEUE_NUM);

    /* Support for Automation Testing. */
    PRINT_STATUS(S_PASS);

    while(1);
}

/*
** This function configures and sets the EDMA PaRAM set values for
** transferring data to UART TX FIFO.
*/
static void UARTTxEDMAPaRAMSetConfig(unsigned char *txBuffer,
                                     unsigned int length,
                                     unsigned int tccNum,
                                     unsigned short linkAddr,
                                     unsigned int chNum)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information. */
    paramSet.srcAddr = (unsigned int) txBuffer;
    paramSet.destAddr = (unsigned int)UART_THR_RHR_REG;

    paramSet.aCnt = (unsigned short)1;
    paramSet.bCnt = (unsigned short)txBytesPerEvent;
    paramSet.cCnt = (unsigned short)(length / txBytesPerEvent);
    paramSet.srcBIdx = (short)1;
    paramSet.srcCIdx = (short)txBytesPerEvent;

    /* The destination indexes should not increment since it is a h/w register. */
    paramSet.destBIdx = (short)0;
    paramSet.destCIdx = (short)0;

    paramSet.linkAddr = (unsigned short)linkAddr;
    paramSet.bCntReload = (unsigned short)0;

    /* OPT PaRAM entries. */
    paramSet.opt = (unsigned int)0x0;

    /* Source and Destination addressing modes are Incremental. */

    /* AB Synchronized Transfer. */
    paramSet.opt |= (1 << EDMA3CC_OPT_SYNCDIM_SHIFT);

    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);
}

/*
** This function configures and sets the EDMA PaRAM set values for
** receiving data from UART RX FIFO.
*/
static void UARTRxEDMAPaRAMSetConfig(unsigned char *rxBuffer,
                                     unsigned int length,
                                     unsigned int tccNum,
                                     unsigned short linkAddr,
                                     unsigned int chNum)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information. */
    paramSet.srcAddr = (unsigned int)UART_THR_RHR_REG;
    paramSet.destAddr = (unsigned int)rxBuffer;

    paramSet.aCnt = (unsigned short)1;
    paramSet.bCnt = (unsigned short)(rxTrigLevel);
    paramSet.cCnt = (unsigned short)(length / rxTrigLevel);
    paramSet.destBIdx = (short)1;
    paramSet.destCIdx = (short)rxTrigLevel;

    paramSet.srcBIdx = (short)0;
    paramSet.srcCIdx = (short)0;

    paramSet.linkAddr = (unsigned short)linkAddr;
    paramSet.bCntReload = (unsigned short)0;

    /* OPT PaRAM Entries. */
    paramSet.opt = (unsigned int)0x0;

    /* Source and Destination addressing modes are Incremental. */

    /* Enable AB Synchronized Transfer. */
    paramSet.opt |= (1 << EDMA3CC_OPT_SYNCDIM_SHIFT);

    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);
}

/*
** This configures the PaRAM set for the Dummy Transfer.
*/

static void TxDummyPaRAMConfEnable(void)
{
    EDMA3CCPaRAMEntry dummyPaRAMSet;

    dummyPaRAMSet.aCnt = 1;
    dummyPaRAMSet.bCnt = 0;
    dummyPaRAMSet.cCnt = 0;
    dummyPaRAMSet.srcAddr = 0;
    dummyPaRAMSet.destAddr = 0;
    dummyPaRAMSet.srcBIdx = 0;
    dummyPaRAMSet.destBIdx = 0;
    dummyPaRAMSet.srcCIdx = 0;
    dummyPaRAMSet.destCIdx = 0;
    dummyPaRAMSet.linkAddr = 0xFFFF;
    dummyPaRAMSet.bCntReload = 0;
    dummyPaRAMSet.opt = 0;

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);
}

/*
** EDMA Completion Interrupt Service Routine(ISR).
*/
static void Edma3CompletionIsr(void)
{
    volatile unsigned int pendingIrqs;
    unsigned int index = 1;
    unsigned int count = 0;

    if(EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS))
    {
        /*
        ** Wait for a finite time to monitor the EDMA Completion Interrupt
        ** status.
        */
        while ((count < EDMA3CC_COMPL_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0;

            /* Get the Interrupt status. */
            pendingIrqs = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
            while(pendingIrqs)
            {
                if((pendingIrqs & 1u) == TRUE)
                {
                    /* Clear the interrupt status. */
                    EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, index);

                    (*cb_Fxn[index])(index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            count++;
        }
    }
}

/*
** This function is used as a callback from EDMA3 Completion Handler.
*/
static void callback(unsigned int tccNum)
{
    /* Disabling DMA Mode of operation in UART. */
    UARTDMADisable(UART_INSTANCE_BASE_ADD);

    /* Disabling DMA transfer on the specified channel. */
    EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, tccNum, EDMA3_TRIG_MODE_EVENT);

    clBackFlag = 1;
}

/*
** EDMA Error Interrupt Service Routine(ISR).
*/
static void Edma3CCErrorIsr(void)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int evtqueNum = 0;
    unsigned int index = 1;
    unsigned int Cnt = 0;

    if((0 != EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS)) ||
       (0 != (EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS))))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks
           when no pending interrupt is found */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;
            pendingIrqs = EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS);

            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Writing to EMCR to clear the corresponding EMR bits.
                       Also clearing any Secondary events in SER. */
                    EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;

            pendingIrqs = EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS);
            if (pendingIrqs != 0u)
            {
                /* Process all the pending CC error interrupts. */
                /* Queue threshold error for different event queues. */
                for (evtqueNum = 0u; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
                {
                    if((pendingIrqs & (1u << evtqueNum)) != 0u)
                    {
                        /* Clear the error interrupt. */
                        EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS, (1u << evtqueNum));
                    }
                }

                /* Transfer completion code error. */
                if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                {
                    EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS,
                                  (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT));
                }
                ++index;
            }

            Cnt++;
        }
        
        /* Enable error to be evaluated again */
        EDMA3CCErrorEvaluate(SOC_EDMA30CC_0_REGS);
    }
}

/* 
** Powering up, initializing and registering interrupts for EDMA.
*/
static void EDMA3Initialize(void)
{
    /* Initialization of EDMA3 */
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEUE_NUM);

    /* Configuring the AINTC to receive EDMA3 interrupts. */ 
    EDMA3INTCConfigure();
}

/*
** This function initializes the UART instance for use.
*/
static void UARTInitialize(void)
{
    /* Performing a module reset. */
    UARTModuleReset(UART_INSTANCE_BASE_ADD);

#ifdef UART_ENABLE_FIFO
    /* Performing FIFO configurations. */
    UartFIFOConfigure();
#else
    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);
#endif

#if (defined UART_ENABLE_FIFO) && (defined DIRECT_TX_DMA_THRESH_MODE)
    /* Selecting the method of setting the Transmit DMA Threshold value. */
    UARTTxDMAThresholdControl(UART_INSTANCE_BASE_ADD, UART_TX_DMA_THRESHOLD_REG);

    /* Configuring the Transmit DMA Threshold value. */
    UARTTxDMAThresholdValConfig(UART_INSTANCE_BASE_ADD, TX_DMA_THRESHOLD);
#endif

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


#ifdef UART_ENABLE_FIFO
/*
** A wrapper function performing FIFO configurations.
*/
static void UartFIFOConfigure(void)
{
    unsigned int fifoConfig = 0;

    /*
    ** Transmitter Trigger Level Granularity is 1.
    ** Receiver Trigger Level Granularity is 1.
    ** Transmit Trigger Space set using 'txTrigSpace'.
    ** Receive Trigger level set using 'rxTrigLevel'.
    ** Clear the Trasnmit FIFO.
    ** Clear the Receive FIFO.
    ** DMA Mode enabling shall happen through SCR register.
    ** DMA Mode 1 is enabled.
    */
    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  txTrigSpace,
                                  rxTrigLevel,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_1_ENABLE);

    /* Configuring the FIFO settings. */
    UARTFIFOConfig(UART_INSTANCE_BASE_ADD, fifoConfig);
}
#endif

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

/*
** This function configures the AINTC to receive EDMA3 interrupts.
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

/******************************** End of file *******************************/
