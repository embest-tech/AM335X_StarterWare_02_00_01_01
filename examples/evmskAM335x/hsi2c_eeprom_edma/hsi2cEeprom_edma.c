/**
 * \file  hsi2cEeprom_edma.c
 *
 * \brief Sample application for HSI2C using EDMA for data transfer.
 *        This application reads the specified number of bytes from
 *        EEPROM using HSI2C and displays them on the console of the
 *        host machine. The EDMA will be used for data transfer
 *        between I2C registers and DDR memory.
 *
 *        Application Configuration:
 *
 *            Modules Used:
 *                I2C0
 *                UART0
 *                Interrupt Controller
 *                EDMA
 *
 *            Configurable Parameters:
 *                Bus frequency     - 100kHz and 400kHz
 *                EEPROM memory address - [0x000 - 0x7FFF]
 *                No of bytes to be read
 *
 *            Hard-coded configuration of other parameters:
 *                Addressing mode   - 7bit
 *                I2C Instance      - 0
 *                Slave Address     - 0x50
 *
 *        Application Use Case:
 *        1.I2C controller is configured in Master mode.
 *        2.Master will first send the address offset value to EEPROM, which
 *          indicates the address from which data read should start.
 *        3.Then master will read the data from EEPROM and display on console.
 *        4.The functionality is demonstrated in DMA mode.
 *
 *        Running the example:
 *        1.EEPROM with the part number CAT24C256W is connected on the board.
 *        2.On executing the application, the data flashed to EEPROM is read
 *          over I2C bus and same data is displayed on the console.
 *
 *        Limitations:
 *        With no flashed data in EEPROM, if the application tries to read from
 *        EEPROM, then the data values read would be "0xFF", which indicates an
 *        invalid EEPROM data.
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

#include "evmskAM335x.h"
#include "edma_event.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "cache.h"
#include "hsi2c.h"
#include "edma.h"
#include "mmu.h"
#include "error.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
******************************************************************************/

/* I2C address of CAT24C256 EEPROM */
#define I2C_SLAVE_ADDR                (0x50u)

/* Higher byte address (i.e A8-A15) */
#define EEPROM_ADDR_MSB               (0x00u)

/* Lower byte address (i.e A0-A7) */
#define EEPROM_ADDR_LSB               (0x00u)

#define I2C_INST_BASE                 (SOC_I2C_0_REGS)
#define I2C_INST_NUM                  (0u)
#define I2C_INT_NUM                   (SYS_INT_I2C0INT)
#define EDMA3_I2C_TX_CHA_NUM          (EDMA3_CHA_I2C0_TX)
#define EDMA3_I2C_RX_CHA_NUM          (EDMA3_CHA_I2C0_RX)

/* Number of bytes to be read from EEPROM. */
#define NUM_BYTES_READ                (1024u)

/* Clock related definitions. */
#define I2C_SYS_CLK_48MHZ             (48000000u)
#define I2C_INTERNAL_CLK_12MHZ        (12000000u)
#define BUS_FREQ_MAX                  (400u)
#define BUS_FREQ_MIN                  (100u)

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM                 (0u)

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR                (0x80000000u)
#define START_ADDR_DEV                (0x44000000u)
#define START_ADDR_OCMC               (0x40300000u)
#define NUM_SECTIONS_DDR              (512u)
#define NUM_SECTIONS_DEV              (960u)
#define NUM_SECTIONS_OCMC             (1u)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void I2CEdmaTransmitConfig(unsigned char* txBuffer, unsigned short length);
static void I2CEdmaReceiveConfig(unsigned char* rxBuffer, unsigned short length);
static void SetupI2CReception(unsigned short dataCountVal);
static void MMUConfigAndEnable(void);
static void EDMA3INTCConfigure(void);
static void I2CEdmaErrIsr(void);
static void EEPROMRead(void);
static void I2CEdmaIsr(void);
static void SetupI2C(void);

/******************************************************************************
**                      INTERNAL VARIABLE DECLARATIONS
******************************************************************************/
volatile unsigned int numBytesRead = 0;
volatile unsigned int addEeprom = 0;
volatile unsigned int complFlag = 1;
volatile unsigned int numOfBytes;
volatile unsigned int busFreq;

/* Page tables start must be aligned in 16K boundary. */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#pragma DATA_ALIGN(dataFromSlave, SOC_CACHELINE_SIZE_MAX);
volatile unsigned char dataFromSlave[NUM_BYTES_READ];

#pragma DATA_ALIGN(dataToSlave, SOC_CACHELINE_SIZE_MAX);
volatile unsigned char dataToSlave[2];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#pragma data_alignment=SOC_CACHELINE_SIZE_MAX
volatile unsigned char dataFromSlave[NUM_BYTES_READ];

#pragma data_alignment=SOC_CACHELINE_SIZE_MAX
volatile unsigned char dataToSlave[2];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

volatile unsigned char dataFromSlave[NUM_BYTES_READ]
 __attribute__((aligned(SOC_CACHELINE_SIZE_MAX)));

volatile unsigned char dataToSlave[2]
 __attribute__((aligned(SOC_CACHELINE_SIZE_MAX)));

#else
#error "Unsupported Compiler.\r\n"

#endif

/******************************************************************************
**                     FUNCTION DEFINITIONS
******************************************************************************/

/*
** This function will setup the MMU. The function maps three regions -
** 1. DDR
** 2. OCMC RAM
** 3. Device memory
** The function also enables the MMU.
*/
static void MMUConfigAndEnable(void)
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
    unsigned char nibbleData = 0;
    unsigned char choice = 0;
    unsigned int lIndex2;
    unsigned int lIndex;

    /* Configure and enable the MMU. */
    MMUConfigAndEnable();

    /* Enable Instruction, Data and Unified Cache. */
    CacheEnable(CACHE_ALL);

    /* Initialize console for communication with the Host Machine. */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable the clock for I2C0. */
    I2C0ModuleClkConfig();

    /* Configuring the system clocks for EDMA. */
    EDMAModuleClkConfig();

    /* Performing Pin Multiplexing for the specified I2C instance. */
    I2CPinMuxSetup(I2C_INST_NUM);

    /* Initialization of EDMA3 */
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEUE_NUM);

    /* Requesting a DMA channel for I2C Reception. */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS,
                        EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_I2C_RX_CHA_NUM,
                        EDMA3_I2C_RX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Requesting a DMA channel for I2C Transmission. */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS,
                        EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_I2C_TX_CHA_NUM,
                        EDMA3_I2C_TX_CHA_NUM,
                        EVT_QUEUE_NUM);

    ConsoleUtilsPrintf(
                     "StarterWare HSI2C EEPROM EDMA example application.\r\n");
    ConsoleUtilsPrintf(
                 "\r\nThis application reads data from EEPROM which is 32KB ");
    ConsoleUtilsPrintf(
                      "in size. The addressable memory range is from 0x0000 ");
    ConsoleUtilsPrintf(
                  "to 0x7FFF. Make sure the reads are within this range.\r\n");
    ConsoleUtilsPrintf(
                   "\r\nEnter the bus frequency(in KHz) at which I2C has to ");
    ConsoleUtilsPrintf("communicate(Freq. range %d-%d KHz): ",
                                                   BUS_FREQ_MIN, BUS_FREQ_MAX);

    while(1)
    {
        ConsoleUtilsScanf("%u", &busFreq);

        if((busFreq < BUS_FREQ_MIN) || (busFreq > BUS_FREQ_MAX))
        {
            ConsoleUtilsPrintf("\r\nInvalid entry!. Re-enter a valid value: ");
        }
        else
        {
            break;
        }
    }

    /* Configure the Bus Frequency, Slave address and enable the I2C. */
    SetupI2C();

    /* Configures the INTC to receive EDMA3 interrupts. */
    EDMA3INTCConfigure();

    /* Enabling IRQ in CPSR of the ARM processor. */
    IntMasterIRQEnable();

    do
    {
        ConsoleUtilsPrintf(
                    "\r\n\r\nEnter an address in EEPROM starting from which ");
        ConsoleUtilsPrintf(
                     "data have to be read(Address range: 0x0000 - 0x7FFF): ");

        ConsoleUtilsScanf("%X", &addEeprom);

        ConsoleUtilsPrintf(
                          "\r\n\r\nEnter the number of data bytes to be read");
        ConsoleUtilsPrintf("(Range: 1 - %d): ", NUM_BYTES_READ);

        ConsoleUtilsScanf("%u", &numBytesRead);

        ConsoleUtilsPrintf("\r\n\r\nData read from EEPROM:\r\n");

        /* Read data from the specified address of EEPROM. */
        EEPROMRead();

        /*
        ** Invalidate the Cache contents corresponding to 'dataFromSlave'
        ** buffer.
        */
        CacheDataInvalidateBuff((unsigned int)dataFromSlave, numBytesRead);

        /*
        ** Print the specified number of bytes of the buffer to which
        ** the EEPROM contents have been read.
        */
        for(lIndex = 0; lIndex < numBytesRead; lIndex++)
        {
            /* Collect the most significant nibble from the read data. */
            nibbleData = ((dataFromSlave[lIndex] & 0xf0) >> 4);

            for(lIndex2 = 0; lIndex2 < 2; lIndex2++)
            {
                /* Check if the value is a numeric value. */
                if(nibbleData < 10)
                {
                    /* Converting the digit to its ASCII equivalent. */
                    ConsoleUtilsPrintf("%c", (nibbleData + 0x30));
                }

                /* The value is a hexadecimal alphanumeric value. */
                else
                {
                    /*
                    ** Converting the hexadecimal alphanumeric number to its
                    ** ASCII equivalent.
                    */
                    ConsoleUtilsPrintf("%c", (nibbleData + 0x37));
                }

                /* Collect the least significant nibble from the read data. */
                nibbleData = dataFromSlave[lIndex] & 0x0f;
            }

            /* Separating displayed bytes by comma. */
            if(lIndex != (numBytesRead - 1))
            {
                ConsoleUtilsPrintf("%c", ',');
            }

            /* Ending the display process by a period. */
            else
            {
                ConsoleUtilsPrintf("%c", '.');
            }
        }

        ConsoleUtilsPrintf("\r\n\r\n");
        ConsoleUtilsPrintf(
                        "Do you want to read more data from EEPROM ? (y/n): ");
        ConsoleUtilsScanf("%c", &choice);

    }while(('y' == choice) || ('Y' == choice));

    ConsoleUtilsPrintf("\r\n");

    /* Support for Automation testing. */
    PRINT_STATUS(S_PASS);

    while(1);
}

/*
** This function does the following operations:
** - configures the Bus Frequency at which I2C has to communicate\n
** - directs the I2C to communicate with a particular slave\n
** - enables the I2C\n
*/
static void SetupI2C(void)
{
    /* Put I2C in reset/disabled state. */
    I2CMasterDisable(I2C_INST_BASE);

    /* Disable Auto-Idle functionality. */
    I2CAutoIdleDisable(I2C_INST_BASE);

    /* Configure I2C bus frequency. */
    I2CMasterInitExpClk(I2C_INST_BASE,
                        I2C_SYS_CLK_48MHZ,
                        I2C_INTERNAL_CLK_12MHZ,
                        busFreq * 1000);

    /* Set I2C slave address. */
    I2CMasterSlaveAddrSet(I2C_INST_BASE, I2C_SLAVE_ADDR);

    /* Bring I2C out of reset. */
    I2CMasterEnable(I2C_INST_BASE);
}

/* 
** Reads data from EEPROM starting from the specified address.
*/
static void EEPROMRead(void)
{
    dataToSlave[0] = (unsigned char)((addEeprom & 0xFF00) >> 8);
    dataToSlave[1] = (unsigned char)(addEeprom & 0x00FF);

    /*
    ** Clean the Cache contents corresponding to 'dataToSlave' buffer.
    */
    CacheDataCleanBuff((unsigned int)dataToSlave, 2);

    I2CEdmaTransmitConfig((unsigned char*)dataToSlave, 2);

    SetupI2CReception(numBytesRead);
}

/*
** Transmits and Receives data over I2C bus. 
*/
static void SetupI2CReception(unsigned short dataCountVal)
{
    /** I2C as a Master-Transmitter transmits data to EEPROM(Slave). **/

    /*
    ** Data Count specifies the number of bytes to be transmitted.
    ** The two bytes transmitted account for the EEPROM address which is
    ** 16 bits in length. This is the address starting from which the
    ** specified number of bytes have to be read.
    */
    I2CSetDataCount(I2C_INST_BASE, 0x02);

    /* Reading the value of Data Count that was set above. */
    numOfBytes = I2CDataCountGet(I2C_INST_BASE);

    /* Clear status of all interrupts */
    I2CMasterIntClearEx(I2C_INST_BASE, 0x7FF);

    /* Configure I2C controller in Master Transmitter mode. */
    I2CMasterControl(I2C_INST_BASE, I2C_CFG_MST_TX);

    /*I2C Transmit Event is enabled */
    I2CDMATxEventEnable(I2C_INST_BASE);

    /* Generate Start Condition over I2C bus. */
    I2CMasterStart(I2C_INST_BASE);

    /*
    ** Wait for the START to be reflected on the bus.
    ** This can be checked by waiting for BUS BUSY condition set.
    */
    while(I2CMasterBusBusy(I2C_INST_BASE) == 0);

    /* while(txCount != numOfBytes); */
    while(complFlag);

    complFlag = 1;

    /* Wait until I2C registers are ready to be accessed. */
    while(!(I2CMasterIntRawStatus(I2C_INST_BASE) & (I2C_INT_ADRR_READY_ACESS)));

    /** I2C as a Master-Receiver receives data from EEPROM. **/

    /* Configure the PaRAM set for reception. */
    I2CEdmaReceiveConfig((unsigned char*)dataFromSlave, dataCountVal);

    /* Data Count specifies the number of bytes to be received. */
    I2CSetDataCount(I2C_INST_BASE, dataCountVal);

    /* Reading the value of Data Count that was set above. */
    numOfBytes = I2CDataCountGet(I2C_INST_BASE);

    /* Clear status of all interrupts */
    I2CMasterIntClearEx(I2C_INST_BASE, 0x7FF);

    /* Configure I2C controller in Master Receiver mode. */
    I2CMasterControl(I2C_INST_BASE, I2C_CFG_MST_RX);

    /* I2C Receive Event is Enable */
    I2CDMARxEventEnable(I2C_INST_BASE);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(I2C_INST_BASE);

    while(I2CMasterBusBusy(I2C_INST_BASE) == 0);

    while(complFlag);

    complFlag = 1;

    /* Generate stop condition */
    I2CMasterStop(I2C_INST_BASE);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Receive
** Channel of I2C. The EDMA is also enabled to transfer the data.
** However, the actual transfer shall happen only when the DMA
** feature is enabled in the I2C.
*/
static void I2CEdmaReceiveConfig(unsigned char* rxBuffer, unsigned short length)
{
    EDMA3CCPaRAMEntry paramSet;
  
    paramSet.srcAddr   = (I2C_INST_BASE + I2C_DATA);
    paramSet.destAddr  = (unsigned int)rxBuffer;

    paramSet.aCnt       = 0x01;
    paramSet.bCnt       = length;
    paramSet.cCnt       = 0x01;

    /*
    ** The Source indices should not increment since the Source location is a
    ** hardware register.
    */
    paramSet.srcBIdx    = 0x00;
    paramSet.srcCIdx    = 0x00;

    /*
    ** The Destination indices should increment since the Destination location
    ** is memory. 
    */
    paramSet.destBIdx   = 0x01;
    paramSet.destCIdx   = 0x00;

    paramSet.bCntReload = 0x00;
    paramSet.linkAddr   = 0xFFFF;

    paramSet.opt        = 0x00;

    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= ((EDMA3_I2C_RX_CHA_NUM << EDMA3CC_OPT_TCC_SHIFT) &
                      EDMA3CC_OPT_TCC);

    /* Enable the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);
 
    /* Now write to the respective PaRAM set. */ 
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_I2C_RX_CHA_NUM, &paramSet);

    /* Enable the EDMA transfer. */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS,
                        EDMA3_I2C_RX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Transmit 
** Channel of I2C. The EDMA is also enabled to transfer the data.
** However, the actual transfer shall happen only when the DMA
** feature is enabled in the I2C.
*/
static void I2CEdmaTransmitConfig(unsigned char* txBuffer, unsigned short length)
{
    EDMA3CCPaRAMEntry paramSet;

    /*
    ** I2C generates one TX EDMA event whenever I2CXSR is empty. There is space
    ** for only one byte of data in I2CXSR. There is no FIFO. Hence one byte
    ** needs to be transferred per event.
    */

    paramSet.srcAddr    = (unsigned int)txBuffer;
    paramSet.destAddr   = I2C_INST_BASE + I2C_DATA;

    paramSet.aCnt       = 0x01;
    paramSet.bCnt       = length;
    paramSet.cCnt       = 0x01;

    /*
    ** The Source indices should increment since the Source location is memory.
    */ 
    paramSet.srcBIdx    = 0x01;
    paramSet.srcCIdx    = 0x00;

    /*
    ** The Destination indices should not increment since the Destination location
    ** is a hardware register.
    */
    paramSet.destBIdx   = 0x00;
    paramSet.destCIdx   = 0x00;

    paramSet.bCntReload = 0x00;
    paramSet.linkAddr   = 0xFFFF;
    paramSet.opt        = 0;

    /* Setting the Transfer Complete Code(TCC).*/
    paramSet.opt |= ((EDMA3_I2C_TX_CHA_NUM << EDMA3CC_OPT_TCC_SHIFT) &
                      EDMA3CC_OPT_TCC);

    /* Enable the Completion Interrupt.*/
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write to the respective PaRAM set. */ 
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_I2C_TX_CHA_NUM , &paramSet);

    /* Enable the EDMA transfer. */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS,
                        EDMA3_I2C_TX_CHA_NUM,
                        EDMA3_TRIG_MODE_EVENT);
}

/*
** This function configures the INTC to receive EDMA3 interrupts.
*/
static void EDMA3INTCConfigure(void)
{
    /* Intialize ARM interrupt controller */
    IntAINTCInit();

    /*
    ** Register the routine 'I2CEdmaIsr' corresponding to EDMA Completion
    ** Interrupt.
    */
    IntRegister(SYS_INT_EDMACOMPINT , I2CEdmaIsr);

     /* Set priority for EDMA3CC0 completion interrupt in INTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /*
    ** Regsiter the routine 'I2CEdmaErrIsr' corresponding to EDMA Error
    ** Interrupt.
    */
    IntRegister(SYS_INT_EDMAERRINT, I2CEdmaErrIsr);

    /* Set priority for EDMA3CC0 Error interrupt in INTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the EDMA3CC0 Completion interrupt in INTC.*/
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /* Enable the EDMA3CCERR interrupt in INTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}

/*
** EDMA Completion Interrupt Service Routine(ISR).
** This function also disables the DMA feature in I2C for Transmit/Receive
** depending on the event.
*/
static void I2CEdmaIsr(void)
{
    volatile unsigned int pendingIrqs = 0;

    /* Get status of the interrupt */
    pendingIrqs = EDMA3IntrStatusHighGet(SOC_EDMA30CC_0_REGS);

    /* Check for the Completion interrupt status of I2C Transmit event. */
    if(pendingIrqs & (0x01 << (EDMA3_I2C_TX_CHA_NUM - 32)))
    {
        /* Clear the pending interrupt. */
        EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, EDMA3_I2C_TX_CHA_NUM);

        /* Disable the I2C Transmit event. */
        I2CDMATxEventDisable(I2C_INST_BASE);

        /* Disable Edma Transfer. */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS,
                             EDMA3_I2C_TX_CHA_NUM,
                             EDMA3_TRIG_MODE_EVENT);

        complFlag = 0;
    }

    /* Check for the Completion interrupt status of I2C Receive event. */
    if(pendingIrqs & (0x01 << (EDMA3_I2C_RX_CHA_NUM - 32)))
    {
        /* Clear the pending interrupt. */
        EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, EDMA3_I2C_RX_CHA_NUM);

        /* Disable the I2C Receive event. */
        I2CDMARxEventDisable(I2C_INST_BASE);

        /* Disable Edma Transfer. */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS,
                             EDMA3_I2C_RX_CHA_NUM,
                             EDMA3_TRIG_MODE_EVENT);

        complFlag = 0;
    }
}

/*
** EDMA Error Interrupt Service Routine(ISR).
** This function also disables the DMA feature in I2C for Transmit/Receive
** depending on the event.
*/
static void I2CEdmaErrIsr(void)
{
    volatile  unsigned int pendingIrqs = 0;

    /* Get status of the error interrupt */
    pendingIrqs = EDMA3ErrIntrHighStatusGet(SOC_EDMA30CC_0_REGS);

    /* Checking for Errors due to I2C Transmit event. */
    if((pendingIrqs & (0x01 << (EDMA3_I2C_TX_CHA_NUM  - 32))))
    {
        /* Clear the secondary and missed I2C EDMA events. */
        EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, EDMA3_I2C_TX_CHA_NUM);  
    
        /* Disable the I2C Transmit event. */
        I2CDMATxEventDisable(I2C_INST_BASE);

        /* Disable Edma Transfer */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS,
                             EDMA3_I2C_TX_CHA_NUM,
                             EDMA3_TRIG_MODE_EVENT);
    }

    /* Checking for Errors due to I2C Receive event. */
    else if((pendingIrqs & (0x01 << (EDMA3_I2C_RX_CHA_NUM - 32))))
    {
        /* Clear the secondary and missed I2C EDMA events. */
        EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, EDMA3_I2C_RX_CHA_NUM);  

        /* Disable the I2C Receive event. */
        I2CDMARxEventDisable(I2C_INST_BASE);

        /* Disable Edma Transfer */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS,
                             EDMA3_I2C_RX_CHA_NUM,
                             EDMA3_TRIG_MODE_EVENT);
    }
}

/***************************** End of file ***********************************/
