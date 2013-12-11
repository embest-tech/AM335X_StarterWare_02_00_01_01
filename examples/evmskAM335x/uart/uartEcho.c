/**
 * \file   uartEcho.c
 *
 * \brief  This example application demonstrates the working of UART for serial
 *         communication with another device/host by echoing back the data
 *         entered from serial console.
 *   
 *         Application Configuration:
 *   
 *             Modules Used:
 *                 UART0
 *                 Interrupt Controller
 *   
 *             Configurable Parameters:
 *                 1)Baud Rate - 115200 bps to 926100 bps
 *                 2)Word Length - 5,6,7,8 bits
 *                 3)Parity - None, Odd, Even, Mark, Space
 *                 4)Stop Bits - 1, 1.5, 2 stop bits
 *                 
 *             Hard-coded configuration of other parameters:
 *                 1) FIFO Threshold Levels:
 *                    a) TX Trigger Space value - 56
 *                    b) TX Threshold Level - 8 (TX FIFO Size - TX Trigger Space)
 *                    c) RX Threshold Level - 1
 *   
 *         Application Use case:
 *             1) Application demonstrates the Receive/Transmit features
 *                of UART using a FIFO.
 *             2) Interrupt features related to FIFO trigger levels are
 *                demonstrated for reading/writing to the FIFO.
 *             3) Functionality of UART with different Line characterics
 *                and Baud rates are demonstrated.
 *   
 *         Running the Example:
 *             On executing the example:
 *             1) The user will be requested to enter 8 bytes of data from the
 *                serial console. The keyed in data will be echoed back
 *                immediately.
 *             2) Then the user will be given options to either re-run the
 *                application using default or non-default Baud Rate and Line
 *                characteristics or quit the application.
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

#include "uart_irda_cir.h"
#include "evmskAM335x.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "hw_types.h"
#include <string.h>
#include "cache.h"
#include "mmu.h"
#include "error.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define BAUD_RATE_115200           (115200u)
#define UART_MODULE_INPUT_CLK      (48000000u)

#define START_ADDR_DDR             (0x80000000u)
#define START_ADDR_DEV             (0x44000000u)
#define START_ADDR_OCMC            (0x40300000u)
#define NUM_SECTIONS_DDR           (512u)
#define NUM_SECTIONS_DEV           (960u)
#define NUM_SECTIONS_OCMC          (1u)

#define UART_INSTANCE_BASE_ADD     (SOC_UART_0_REGS)
#define UART_INSTANCE_INT_NUM      (SYS_INT_UART0INT)

/* Baud Rates of UART for communication. */
#define BAUD_RATE_115200          (115200u)
#define BAUD_RATE_128000          (128000u)
#define BAUD_RATE_230400          (230400u)
#define BAUD_RATE_460800          (460800u)
#define BAUD_RATE_921600          (921600u)

#define APP_EXIT                  (FALSE)
#define APP_CONTINUE              (TRUE)
#define NUM_TX_STRINGS            (2)

/*
** The number of data bytes to be transmitted to Transmit FIFO of UART
** per generation of the Transmit Empty interrupt. This can take a maximum
** value of TX Trigger Space which is 'TX FIFO size - TX Threshold Level'.
*/
#define NUM_TX_BYTES_PER_TRANS    (56)

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static unsigned int UartUserInputGet(unsigned int *baudRate,
                                     unsigned int *lineCharConfig);
static void UartCustomizeSettings(unsigned int *baudRate,
                                  unsigned int *lineCharConfig);
static void UartConfigure(unsigned int baudRate, unsigned int lineCharConfig);
static void UartLineCharacSet(unsigned int lineCharConfig);
static void UartBaudRateSet(unsigned int baudRate);
static void MMUConfigAndEnable(void);
static void UARTINTCConfigure(void);
static void UartFIFOConfigure(void);
static void UARTIsr(void);

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
unsigned char welcome[] = "\r\n\r\nStarterWare AM335X UART Interrupt application.\r\n";
unsigned char enter[] = "Please enter 8 bytes. They will echoed back individually.\r\n";
volatile unsigned int numRdBytesISR = 0;

/* A flag used to signify the application to transmit data to UART TX FIFO. */
volatile unsigned int txFIFOEmptyFlag = FALSE;

/* This keeps a count of the number of strings transferred to TX FIFO. */
volatile unsigned int txStrCount = 0;

/*
** Flag which indicates whether the total Transmit transaction which involves
** transfer of all the strings in context is complete.
*/
volatile unsigned int txComplFlag = FALSE;

/* Page tables start must be aligned in 16K boundary */
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
**              FUNCTION DEFINITIONS
******************************************************************************/

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
    unsigned char *pTxStr[NUM_TX_STRINGS] = {welcome, enter};
    unsigned int numByteChunks[NUM_TX_STRINGS] = {0};
    unsigned int remainBytes[NUM_TX_STRINGS] = {0};
    static unsigned char *qTxStr = NULL;
    unsigned int lineCharConfig = 0;
    unsigned int numBytesTx = 0;
    unsigned int baudRate = 0;
    unsigned int cntIndex = 0;
    int retVal = -1;

    /* Setup the MMU and do necessary MMU configurations. */
    MMUConfigAndEnable();

    /* Enable all levels of CACHE. */
    CacheEnable(CACHE_ALL);

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Performing the Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Performing a module reset. */
    UARTModuleReset(UART_INSTANCE_BASE_ADD);

    /* Performing FIFO configurations. */
    UartFIFOConfigure();

    UartConfigure(BAUD_RATE_115200, (UART_FRAME_WORD_LENGTH_8 |
                                     UART_FRAME_NUM_STB_1 |
                                     UART_PARITY_NONE));

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Configuring AINTC to receive UART0 interrupts. */
    UARTINTCConfigure();

    /* Collecting the quotient and remainder values to aid in transmission. */
    for(cntIndex = 0; cntIndex < NUM_TX_STRINGS; cntIndex++)
    {
        numByteChunks[cntIndex] = (strlen((const char *)pTxStr[cntIndex])) /
                                   NUM_TX_BYTES_PER_TRANS;

        remainBytes[cntIndex] = (strlen((const char *)pTxStr[cntIndex])) %
                                 NUM_TX_BYTES_PER_TRANS;
    }

    /* Enabling required UART Interrupts. */
    UARTIntEnable(UART_INSTANCE_BASE_ADD,
                  (UART_INT_LINE_STAT | UART_INT_THR | UART_INT_RHR_CTI));

    while(1)
    {
        txComplFlag = FALSE;
        qTxStr = pTxStr[txStrCount];
        cntIndex = 0;

        while(TRUE != txComplFlag)
        {
            /* This branch is entered if a TX transaction is not yet complete. */
            if(TRUE == txFIFOEmptyFlag)
            {
                if(cntIndex < numByteChunks[txStrCount])
                {
                    /* Transmitting bytes in chunks of NUM_TX_BYTES_PER_TRANS. */
                    numBytesTx = UARTFIFOWrite(SOC_UART_0_REGS,
                                               qTxStr,
                                               NUM_TX_BYTES_PER_TRANS);

                    qTxStr = qTxStr + numBytesTx;
                    cntIndex++;
                }

                else
                {
                    /* Transmitting remaining data from the data block. */
                    numBytesTx = UARTFIFOWrite(SOC_UART_0_REGS,
                                               qTxStr,
                                               remainBytes[txStrCount]);

                    txStrCount++;
                    if(NUM_TX_STRINGS != txStrCount)
                    {
                        qTxStr = pTxStr[txStrCount];
                    }
                }

                txFIFOEmptyFlag = FALSE;

                /*
                ** Re-enables the Transmit Interrupt. This interrupt
                ** was disabled in the Transmit section of the UART ISR.
                */
                UARTIntEnable(SOC_UART_0_REGS, UART_INT_THR);
            }
        }

        while(numRdBytesISR < 8)
        {

        }
        numRdBytesISR = 0;
        /* Disabling the currently enabled UART interrupts. */
        UARTIntDisable(UART_INSTANCE_BASE_ADD, (UART_INT_RHR_CTI | UART_INT_LINE_STAT));

        retVal = UartUserInputGet(&baudRate, &lineCharConfig);
        if(APP_EXIT == retVal)
        {
            ConsoleUtilsPrintf("\r\nApplication is exiting.\r\n");
            break;
        }

        ConsoleUtilsPrintf(
               "\r\nReconfiguring the UART with the selected parameters.\r\n");
        UARTModuleReset(UART_INSTANCE_BASE_ADD);
        UartFIFOConfigure();
        UartConfigure(baudRate, lineCharConfig);

        txStrCount = 0;

        /* Enabling required UART Interrupts. */
        UARTIntEnable(UART_INSTANCE_BASE_ADD,
                      (UART_INT_LINE_STAT | UART_INT_THR | UART_INT_RHR_CTI));
    }

    /* Support for Automation Testing. */
    PRINT_STATUS(S_PASS);

    while(1);
}


/*
** A wrapper function performing FIFO configurations.
*/

static void UartFIFOConfigure(void)
{
    unsigned int fifoConfig = 0;

    /*
    ** - Transmit Trigger Level Granularity is 4
    ** - Receiver Trigger Level Granularity is 1
    ** - Transmit FIFO Space Setting is 56. Hence TX Trigger level
    **   is 8 (64 - 56). The TX FIFO size is 64 bytes.
    ** - The Receiver Trigger Level is 1.
    ** - Clear the Transmit FIFO.
    ** - Clear the Receiver FIFO.
    ** - DMA Mode enabling shall happen through SCR register.
    ** - DMA Mode 0 is enabled. DMA Mode 0 corresponds to No
    **   DMA Mode. Effectively DMA Mode is disabled.
    */
    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_4,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  UART_FCR_TX_TRIG_LVL_56,
                                  1,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_0_ENABLE);

    /* Configuring the FIFO settings. */
    UARTFIFOConfig(UART_INSTANCE_BASE_ADD, fifoConfig);
}

/*
** A wrapper function performing UART Baud Rate settings.
*/

static void UartBaudRateSet(unsigned int baudRate)
{
    unsigned int divisorValue = 0;

    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         baudRate,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(UART_INSTANCE_BASE_ADD, divisorValue);
}

/*
** A wrapper function performing UART Line Characteristics Configuration.
*/

static void UartLineCharacSet(unsigned int lineCharConfig)
{
    unsigned int wLenStbFlag = 0;
    unsigned int parityFlag = 0;

    wLenStbFlag = (lineCharConfig & (UART_LCR_NB_STOP | UART_LCR_CHAR_LENGTH));
    parityFlag = (lineCharConfig & (UART_LCR_PARITY_TYPE2 |
                                    UART_LCR_PARITY_TYPE1 |
                                    UART_LCR_PARITY_EN));

    UARTLineCharacConfig(UART_INSTANCE_BASE_ADD, wLenStbFlag, parityFlag);
}

/*
** A wrapper function performing UART Configurations.
*/

static void UartConfigure(unsigned int baudRate, unsigned int lineCharConfig)
{
    UartBaudRateSet(baudRate);

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(UART_INSTANCE_BASE_ADD, UART_REG_CONFIG_MODE_B);

    UartLineCharacSet(lineCharConfig);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(UART_INSTANCE_BASE_ADD);

    /* Disabling Break Control. */
    UARTBreakCtl(UART_INSTANCE_BASE_ADD, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(UART_INSTANCE_BASE_ADD, UART16x_OPER_MODE);
}

/*
** Interrupt Service Routine for UART.
*/

static void UARTIsr(void)
{
    unsigned int rxErrorType = 0;
    unsigned char rxByte = 0;
    unsigned int intId = 0;
    unsigned int idx = 0;

    /* Checking ths source of UART interrupt. */
    intId = UARTIntIdentityGet(UART_INSTANCE_BASE_ADD);

    switch(intId)
    {
        case UART_INTID_TX_THRES_REACH:

            /* Checking if the total TX transactions are complete. */
            if(txStrCount < NUM_TX_STRINGS)
            {
                txFIFOEmptyFlag = TRUE;
            }
            else
            {
                txComplFlag = TRUE;
            }

            UARTIntDisable(SOC_UART_0_REGS, UART_INT_THR);

        break;

        case UART_INTID_RX_THRES_REACH:
            rxByte = UARTCharGetNonBlocking(UART_INSTANCE_BASE_ADD);
            UARTCharPutNonBlocking(UART_INSTANCE_BASE_ADD, rxByte);
            numRdBytesISR++;
        break;

        case UART_INTID_RX_LINE_STAT_ERROR:

            rxErrorType = UARTRxErrorGet(SOC_UART_0_REGS);

            /* Check if Overrun Error has occured. */
            if(rxErrorType & UART_LSR_RX_OE)
            {
                ConsoleUtilsPrintf("\r\nUART Overrun Error occured."
                              " Reading and Echoing all data in RX FIFO.\r\n");

                /* Read the entire RX FIFO and the data in RX Shift register. */
                for(idx = 0; idx < (RX_FIFO_SIZE + 1); idx++)
                {
                    rxByte = UARTFIFOCharGet(SOC_UART_0_REGS);
                    UARTFIFOCharPut(SOC_UART_0_REGS, rxByte);
                }

                break;
            }

            /* Check if Break Condition has occured. */
            else if(rxErrorType & UART_LSR_RX_BI)
            {
                ConsoleUtilsPrintf("\r\nUART Break Condition occured.");
            }

            /* Check if Framing Error has occured. */
            else if(rxErrorType & UART_LSR_RX_FE)
            {
                ConsoleUtilsPrintf("\r\nUART Framing Error occured.");
            }

            /* Check if Parity Error has occured. */
            else if(rxErrorType & UART_LSR_RX_PE)
            {
                ConsoleUtilsPrintf("\r\nUART Parity Error occured.");
            }

            ConsoleUtilsPrintf(" Data at the top of RX FIFO is: ");
            rxByte = UARTFIFOCharGet(SOC_UART_0_REGS);
            UARTFIFOCharPut(SOC_UART_0_REGS, rxByte);

        break;

        case UART_INTID_CHAR_TIMEOUT:

            ConsoleUtilsPrintf("\r\nUART Character Timeout Interrupt occured."
                              " Reading and Echoing all data in RX FIFO.\r\n");

            /* Read all the data in RX FIFO. */
            while(TRUE == UARTCharsAvail(SOC_UART_0_REGS))
            {
                rxByte = UARTFIFOCharGet(SOC_UART_0_REGS);
                UARTFIFOCharPut(SOC_UART_0_REGS, rxByte);
                numRdBytesISR++;
            }

        break;
    
        default:
        break;    
    }
}

/*
** This function configures the AINTC to receive UART interrupts.
*/

static void UARTINTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(UART_INSTANCE_INT_NUM, UARTIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(UART_INSTANCE_INT_NUM, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(UART_INSTANCE_INT_NUM);
}

/*
** A wrapper function that collects inputs from the user regarding the future
** course of the application.
*/

static unsigned int UartUserInputGet(unsigned int *baudRate, unsigned int *lineCharConfig)
{
    unsigned int retVal = APP_CONTINUE;
    unsigned char choice = 0;

    ConsoleUtilsPrintf("\r\n\r\nThe application presents three options.\r\n"
                       "1> Continue using the application with the Default"
                                                "Settings (115200, 8-N-1).\r\n"
                       "2> Customize the Baud Rate and Line"
                                                         "Characteristics.\r\n"
                       "3> Exit the application.\r\n"
                       "\r\nEnter the serial number of your choice:\r\n");

    ConsoleUtilsScanf("%c", &choice);

    switch(choice)
    {
        case '2':
            UartCustomizeSettings(baudRate, lineCharConfig);
        break;

        case '3':
            retVal = APP_EXIT;
        break;

        case '1':
        default:
            *baudRate = BAUD_RATE_115200;
            *lineCharConfig = (UART_FRAME_WORD_LENGTH_8 |
                               UART_FRAME_NUM_STB_1 |
                               UART_PARITY_NONE);
            ConsoleUtilsPrintf(
                              "\r\nContinuing the application with the Default"
                          " Settings:\r\nBaud Rate = 115200 bps, 8 Data Bits, "
                              "No Parity and 1 Stop Bit.\r\n");
        break;
    }

    return retVal;
}

/*
** A wrapper function that collects inputs from the user related to UART
** Configuration parameters.
*/

static void UartCustomizeSettings(unsigned int *baudRate, unsigned int *lineCharConfig)
{
    unsigned char choice = 0;
    unsigned int wordLen = 0;
    unsigned int stopBit = 0;
    unsigned int parity = 0;

    ConsoleUtilsPrintf("\r\n\r\nThere is a provision for the user"
                                                "to choose from the available"
                       " Baud Rates and Line Characterstics.\r\n"
                       "\r\nChoose a Baud Rate from the following choices:\r\n"
                       "1> 115200 bps\r\n"
                       "2> 230400 bps\r\n"
                       "3> 460800 bps\r\n"
                       "4> 921600 bps\r\n"
                       "\r\nEnter the serial number of your choice:\r\n");

    ConsoleUtilsScanf("%c", &choice);

    switch(choice)
    {
        case '1':
            *baudRate = BAUD_RATE_115200;
            ConsoleUtilsPrintf("\r\nBaud Rate selected = 115200 bps.\r\n");
        break;

        case '2':
            *baudRate = BAUD_RATE_230400;
            ConsoleUtilsPrintf("\r\nBaud Rate selected = 230400 bps.\r\n");
        break;

        case '3':
            *baudRate = BAUD_RATE_460800;
            ConsoleUtilsPrintf("\r\nBaud Rate selected = 460800 bps.\r\n");
        break;

        case '4':
            *baudRate = BAUD_RATE_921600;
            ConsoleUtilsPrintf("\r\nBaud Rate selected = 921600 bps.\r\n");
        break;

        default:
            *baudRate = BAUD_RATE_115200;
            ConsoleUtilsPrintf(
                           "\r\nSelected Default Baud Rate = 115200 bps.\r\n");
        break;
    }

    ConsoleUtilsPrintf(
      "\r\nConfiguring the Line Characteristics  involves two steps:\r\n"
      "1> Configuring the Word Length and number of Stop Bits per frame\r\n"
      "2> Configuring the Parity of the frame\r\n"
      "\r\nFirst Step:\r\nChoose a Word Length from the following choices:\r\n"
      "1> 5 bits\r\n2> 6 bits\r\n3> 7 bits\r\n4> 8 bits\r\n"
      "\r\nEnter the serial number of your choice:\r\n");

    ConsoleUtilsScanf("%c", &choice);

    switch(choice)
    {
        case '1':
            wordLen = UART_FRAME_WORD_LENGTH_5;
            ConsoleUtilsPrintf("\r\nSelected Word Length per frame = 5.\r\n");
        break;

        case '2':
            wordLen = UART_FRAME_WORD_LENGTH_6;
            ConsoleUtilsPrintf("\r\nSelected Word Length per frame = 6.\r\n");
        break;

        case '3':
            wordLen = UART_FRAME_WORD_LENGTH_7;
            ConsoleUtilsPrintf("\r\nSelected Word Length per frame = 7.\r\n");
        break;

        case '4':
            wordLen = UART_FRAME_WORD_LENGTH_8;
            ConsoleUtilsPrintf("\r\nSelected Word Length per frame = 8.\r\n");
        break;

        default:
            wordLen = UART_FRAME_WORD_LENGTH_8;
            ConsoleUtilsPrintf("\r\nSelected Default Word Length per frame = 8.\r\n");
        break;
    }

    ConsoleUtilsPrintf("\r\nChoose the number of Stop Bits per frame"
                       "from the following options:\r\n"
                       "1> 1 Stop Bit (for word length = 5, 6, 7, 8)\r\n"
                       "2> 1.5 Stop Bits (for word length = 5)\r\n"
                       "3> 2 Stop Bits (for word length = 6, 7, 8)\r\n"
                       "\r\nEnter the serial number of your choice:\r\n");

    ConsoleUtilsScanf("%c", &choice);

    switch(choice)
    {
        case '1':
            stopBit = UART_FRAME_NUM_STB_1;
            ConsoleUtilsPrintf(
                        "\r\nSelected Number of Stop Bits per frame = 1.\r\n");
        break;

        case '2':
            stopBit = UART_FRAME_NUM_STB_1_5_2;
            ConsoleUtilsPrintf(
                      "\r\nSelected Number of Stop Bits per frame = 1.5.\r\n");
        break;

        case '3':
            stopBit = UART_FRAME_NUM_STB_1_5_2;
            ConsoleUtilsPrintf(
                        "\r\nSelected Number of Stop Bits per frame = 2.\r\n");
        break;

        default :
            stopBit = UART_FRAME_NUM_STB_1;
            ConsoleUtilsPrintf(
                "\r\nSelected Default Number of Stop Bits per frame = 1.\r\n");
        break;
    }

    ConsoleUtilsPrintf(
            "\r\nLine Characteristics Config - Second Step - Parity\r\n"
            "Choose a Parity Configuration from the following options:\r\n"
            "1> No Parity\r\n"
            "2> Odd Parity is generated.\r\n"
            "3> Even Parity is generated.\r\n"
            "4> The Parity Bit is forced to 1"
                                   " in the transmitted and received data.\r\n"
            "5> The Parity Bit is forced to 0"
                                   " in the transmitted and received data.\r\n"
            "\r\nEnter the serial number of your choice:\r\n");
    ConsoleUtilsScanf("%c", &choice);

    switch(choice)
    {
        case '1':
            parity = UART_PARITY_NONE;
            ConsoleUtilsPrintf(
                              "\r\nSelected Parity Configuration = None.\r\n");
        break;

        case '2':
            parity = UART_ODD_PARITY;
            ConsoleUtilsPrintf("\r\nSelected Parity Configuration = Odd.\r\n");
        break;

        case '3':
            parity = UART_EVEN_PARITY;
            ConsoleUtilsPrintf(
                              "\r\nSelected Parity Configuration = Even.\r\n");
        break;

        case '4':
            parity = UART_PARITY_REPR_1;
            ConsoleUtilsPrintf(
                   "\r\nSelected Parity Configuration = Parity bit is 1.\r\n");
        break;

        case '5': parity = UART_PARITY_REPR_0;
            ConsoleUtilsPrintf(
                   "\r\nSelected Parity Configuration = Parity bit is 0.\r\n");
        break;

        default:
            parity = UART_PARITY_NONE;
            ConsoleUtilsPrintf(
                      "\r\nSelected Default Parity Configuration = None.\r\n");
        break;
    }

    *lineCharConfig = (parity | stopBit | wordLen);
}

/******************************* End of file *********************************/
