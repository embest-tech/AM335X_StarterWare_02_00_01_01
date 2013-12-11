/**
 * \file  i2cEdmaEEprom.c
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
 *            Configurable parameters:
 *                None.
 *
 *            Hard-coded configuration of other parameters:
 *                Bus frequency    - 100kHz
 *                Addressing mode  - 7bit
 *                I2C Instance     - 0
 *                Slave Address    - 0x50
 *                EEPROM memory address - 0x0000
 *                Number of bytes to be read - 50
 *
 *        Application Use Case:
 *        1.I2C controller is configured in Master mode.
 *        2.Master will first send the address offset value to EEPROM, which
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

#include "hsi2c.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "soc_AM335x.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/* I2C address of CAT24C256 e2prom */
#define  I2C_SLAVE_ADDR         (0x50)

/* Higher byte address (i.e A8-A15) */
#define  E2PROM_ADDR_MSB         0x00

/* Lower byte address (i.e A0-A7) */
#define  E2PROM_ADDR_LSB         0x00

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
void I2CIsr(void);
static void SetupI2C(void);
void I2CAINTCConfigure(void);
void CleanUpInterrupts(void);
static void E2promRead(unsigned char *data);
static void SetupI2CReception(unsigned int dcount);
/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int tCount;
volatile unsigned int rCount;
volatile unsigned int flag = 1;
volatile unsigned int numOfBytes;
volatile unsigned char dataToSlave[2];
volatile unsigned char dataFromSlave[50];
/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

int main(void)
{
    unsigned char dataRead[50];
    unsigned char temp;
    unsigned int i;

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable IRQ in CPSR */
    IntMasterIRQEnable();

    /* Configures AINTC to generate interrupt */
    I2CAINTCConfigure();

    /*
    ** Configures I2C to Master mode to generate start
    ** condition on I2C bus and to transmit data at a
    ** speed of 100khz
    */
    SetupI2C();

    /*
    ** Read data from a selected address of
    ** e2prom
    */
    E2promRead(dataRead);


    for(i = 0; i < 50; i++)
    {
         /* Collecting the Most Significant Nibble of the data byte. */
         temp = ((dataRead[i] & 0xF0) >> 4);

         if(temp < 10)
         {
              ConsoleUtilsPrintf("%c", (temp + 0x30));
         }
         else
         {
              ConsoleUtilsPrintf("%c", (temp + 0x37));
         } 

         /* Collecting the Least Significant Nibble of the data byte. */
         temp = (dataRead[i] & 0x0F);

         if(temp < 10)
         {
              ConsoleUtilsPrintf("%c", (temp + 0x30));
         }
         else
         {
              ConsoleUtilsPrintf("%c", (temp + 0x37));
         }
         
         ConsoleUtilsPrintf("%c", ',');
    }


    while(1);
}


 /* 
 ** Reads data from a specific address of e2prom
 */
static void E2promRead(unsigned char *data)
{
    unsigned int i;

    dataToSlave[0] = E2PROM_ADDR_MSB;
    dataToSlave[1] = E2PROM_ADDR_LSB;

    tCount = 0;
    rCount = 0;
    SetupI2CReception(50);

    for (i = 0; i < 50; i++ )
    {
         data[i] = dataFromSlave[i];
    }
}

static void SetupI2C(void)
{
    /* Enable the clock for I2C0 */
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(0);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_0_REGS);

    /* Disable auto Idle functionality */
    I2CAutoIdleDisable(SOC_I2C_0_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_0_REGS, 48000000, 12000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, I2C_SLAVE_ADDR);

    /* Bring I2C out of reset */
    I2CMasterEnable(SOC_I2C_0_REGS);
}

/* Configures AINTC to generate interrupt */
void I2CAINTCConfigure(void)
{
    /* Intialize the ARM Interrupt Controller(AINTC) */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_I2C0INT, I2CIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_I2C0INT);
}

/*
** Receives data over I2C bus 
*/
static void SetupI2CReception(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_0_REGS, 0x02);

    numOfBytes = I2CDataCountGet(SOC_I2C_0_REGS);

    /* Clear status of all interrupts */
    CleanUpInterrupts();

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(tCount != numOfBytes);

    flag = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(SOC_I2C_0_REGS) & (I2C_INT_ADRR_READY_ACESS)));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(SOC_I2C_0_REGS, dcount);

    numOfBytes = I2CDataCountGet(SOC_I2C_0_REGS);

    CleanUpInterrupts();

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY |
                                          I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(flag);

    flag = 1;
}


/* Clear status of all interrupts */
void CleanUpInterrupts(void)
{
    I2CMasterIntEnableEx(SOC_I2C_0_REGS, 0x7FF);
    I2CMasterIntClearEx(SOC_I2C_0_REGS,  0x7FF);
    I2CMasterIntDisableEx(SOC_I2C_0_REGS, 0x7FF);
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
void I2CIsr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(SOC_I2C_0_REGS);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(SOC_I2C_0_REGS,
	                    (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
						
    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
        dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_0_REGS);

	/* Clear receive ready interrupt status */	
        I2CMasterIntClearEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY);
		
         if(rCount == numOfBytes)
         {
              /* Disable the receive ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
              /* Generate a STOP */
              I2CMasterStop(SOC_I2C_0_REGS);
			  
         }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[tCount++]);

        /* Clear Transmit interrupt status */
	I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);		 
						
         if(tCount == numOfBytes)
         {
              /* Disable the transmit ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);
         }

    }
        
    if (status & I2C_INT_STOP_CONDITION)
    {
      	 /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
					       I2C_INT_STOP_CONDITION);
         flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY  |
                                               I2C_INT_RECV_READY      |
                                               I2C_INT_NO_ACK          |
                                               I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(SOC_I2C_0_REGS);

         flag = 0;
    }
}

