/**
 * \file  hsi2cEprom.c
 *
 * \brief Sample application for HSI2C. This application reads the specified
 *        number of bytes from EEPROM using HSI2C and displays them on the
 *        console of the host machine.
 *
 *        Application Configuration:
 *
 *            Modules Used:
 *                I2C0
 *                UART0
 *                Interrupt Controller
 *
 *            Configurable Parameters:
 *                Bus frequency    - 100kHz and 400kHz
 *                EEPROM memory address   - 0x0000 to 0x7FFF
 *                No of bytes to be read
 *
 *            Hard-coded configuration of other parameters:
 *                Addressing mode  - 7 bit
 *                I2C Instance     - 0
 *                Slave Address    - 0x50
 *
 *        Application Use Case:
 *        1.I2C controller is configured in Master mode.
 *        2.Master will first send address offset value to EEPROM, which
 *          indicates the address from which data read should start.
 *        3.Then master will read the data from EEPROM and display on console.
 *        4.The functionality is demonstrated in interrupt mode.
 *
 *        Running the Example:
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
#include "soc_AM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "cache.h"
#include "hsi2c.h"
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
#define I2C_INT_NUM                   (SYS_INT_I2C0INT)

/* Number of bytes to be read from EEPROM. */
#define NUM_BYTES_READ                (1000u)

/* Clock related definitions. */
#define I2C_SYS_CLK_48MHZ             (48000000u)
#define I2C_INTERNAL_CLK_12MHZ        (12000000u)
#define BUS_FREQ_MAX                  (400u)
#define BUS_FREQ_MIN                  (100u)

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
static void SetupI2CReception(unsigned int dataCountVal);
static void MMUConfigAndEnable(void);
static void I2CINTCConfigure(void);
static void EEPROMRead(void);
static void SetupI2C(void);
static void I2CIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DECLARATIONS
******************************************************************************/
volatile unsigned char dataFromSlave[NUM_BYTES_READ];
volatile unsigned int numBytesRead = 0;
volatile unsigned char dataToSlave[2];
volatile unsigned int addEeprom = 0;
volatile unsigned int complFlag = 1;
volatile unsigned int numOfBytes;
volatile unsigned int txCount;
volatile unsigned int rxCount;
volatile unsigned int busFreq;

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

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable the clock for I2C0. */
    I2C0ModuleClkConfig();

    /* Performing Pin Multiplexing for I2C0 instance. */
    I2CPinMuxSetup(0);

    ConsoleUtilsPrintf("StarterWare HSI2C EEPROM example application.\r\n");
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

    /* Configures INTC to receive I2C interrupts. */
    I2CINTCConfigure();

    /* Enable IRQ in CPSR */
    IntMasterIRQEnable();

    /* Configure the Bus Frequency, Slave address and enable the I2C. */
    SetupI2C();

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

        for(lIndex = 0; lIndex < numBytesRead; lIndex++)
        {
            /* Collect the most significant nibble from the read data. */
            nibbleData = ((dataFromSlave[lIndex] & 0xf0) >> 4);

            for(lIndex2 = 0; lIndex2 < 2; lIndex2++)
            {
                /* Check if the value is a numeric value. */
                if(nibbleData < 10)
                {
                    ConsoleUtilsPrintf("%c", (nibbleData + 0x30));
                }

                /* The value is a alpha-numeric value. */
                else
                {
                    ConsoleUtilsPrintf("%c", (nibbleData + 0x37));
                }

                /* Collect the least significant nibble from the read data. */
                nibbleData = dataFromSlave[lIndex] & 0x0f;
            }

            if(lIndex != (numBytesRead - 1))
            {
                ConsoleUtilsPrintf("%c", ',');
            }
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

    txCount = 0;
    rxCount = 0;

    SetupI2CReception(numBytesRead);
}

/*
** Configure the Interrupt Controller(INTC) to receive I2C interrupts.
*/
static void I2CINTCConfigure(void)
{
    /* Intialize the ARM Interrupt Controller(AINTC) */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(I2C_INT_NUM, I2CIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(I2C_INT_NUM, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(I2C_INT_NUM);
}

/*
** Transmits and Receives data over I2C bus. 
*/
static void SetupI2CReception(unsigned int dataCountVal)
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

    /* Transmit interrupt is enabled. */
    I2CMasterIntEnableEx(I2C_INST_BASE, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus. */
    I2CMasterStart(I2C_INST_BASE);

    while(I2CMasterBusBusy(I2C_INST_BASE) == 0);

    while(txCount != numOfBytes);

    complFlag = 1;

    /* Wait until I2C registers are ready to be accessed. */
    while(!(I2CMasterIntRawStatus(I2C_INST_BASE) & (I2C_INT_ADRR_READY_ACESS)));

    /** I2C as a Master-Receiver receives data from EEPROM. **/

    /* Data Count specifies the number of bytes to be received. */
    I2CSetDataCount(I2C_INST_BASE, dataCountVal);

    /* Reading the value of Data Count that was set above. */
    numOfBytes = I2CDataCountGet(I2C_INST_BASE);

    /* Clear status of all interrupts */
    I2CMasterIntClearEx(I2C_INST_BASE, 0x7FF);

    /* Configure I2C controller in Master Receiver mode. */
    I2CMasterControl(I2C_INST_BASE, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled. */
    I2CMasterIntEnableEx(I2C_INST_BASE, I2C_INT_RECV_READY |
                                        I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(I2C_INST_BASE);

    while(I2CMasterBusBusy(I2C_INST_BASE) == 0);

    while(complFlag);

    complFlag = 1;
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
static void I2CIsr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(I2C_INST_BASE);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(I2C_INST_BASE,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
                        
    if(status & I2C_INT_RECV_READY)
    {
        /* Receive data from data receive register */
        dataFromSlave[rxCount++] = I2CMasterDataGet(I2C_INST_BASE);

        /* Clear receive ready interrupt status */    
        I2CMasterIntClearEx(I2C_INST_BASE,  I2C_INT_RECV_READY);

        if(rxCount == numOfBytes)
        {
            /* Disable the receive ready interrupt */
            I2CMasterIntDisableEx(I2C_INST_BASE, I2C_INT_RECV_READY);
            /* Generate a STOP */
            I2CMasterStop(I2C_INST_BASE);
        }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(I2C_INST_BASE, dataToSlave[txCount++]);

        /* Clear Transmit interrupt status */
        I2CMasterIntClearEx(I2C_INST_BASE, I2C_INT_TRANSMIT_READY);         
                        
        if(txCount == numOfBytes)
        {
            /* Disable the transmit ready interrupt */
            I2CMasterIntDisableEx(I2C_INST_BASE, I2C_INT_TRANSMIT_READY);
        }
    }

    if (status & I2C_INT_STOP_CONDITION)
    {
        /* Disable transmit data ready and receive data read interupt */
        I2CMasterIntDisableEx(I2C_INST_BASE, I2C_INT_TRANSMIT_READY |
                                             I2C_INT_RECV_READY     |
                                             I2C_INT_STOP_CONDITION);
         complFlag = 0;
    }

    if(status & I2C_INT_NO_ACK)
    {
        I2CMasterIntDisableEx(I2C_INST_BASE, I2C_INT_TRANSMIT_READY  |
                                             I2C_INT_RECV_READY      |
                                             I2C_INT_NO_ACK          |
                                             I2C_INT_STOP_CONDITION);
        /* Generate a STOP */
        I2CMasterStop(I2C_INST_BASE);

        complFlag = 0;
    }
}

/***************************** End of file ***********************************/
