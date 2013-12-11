/**
 * \file     pmRtc.c
 *
 * \brief    This file contains the API's for RTC which is to be used by the
 *			 Power management application
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

#include "soc_AM335x.h"
#include "hsi2c.h"
#include "interrupt.h"
#include "beaglebone.h"
#include "clock.h"
#include "demoI2c.h"

/******************************************************************************
**                      MACROS
*******************************************************************************/


/*	System clock fed to I2C module - 48Mhz	*/
#define  I2C_SYSTEM_CLOCK					(48000000u)

/*	Internal clock used by I2C module - 12Mhz	*/
#define  I2C_INTERNAL_CLOCK					(12000000u)

/*	I2C bus speed or frequency - 100Khz	*/
#define	 I2C_OUTPUT_CLOCK					(100000u)

/******************************************************************************
**              GLOBAL VARIABLES
******************************************************************************/

volatile unsigned char dataFromSlave[I2C_INSTANCE][2];
volatile unsigned char dataToSlave[I2C_INSTANCE][3];
volatile unsigned int tCount[I2C_INSTANCE];
volatile unsigned int rCount[I2C_INSTANCE];
volatile unsigned int flag[I2C_INSTANCE] = {1, 1, 1};
volatile unsigned int numOfBytes[I2C_INSTANCE];

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
void I2C0Isr(void);
void I2C1Isr(void);
void pmSetupI2C(void);
void I2CAintcConfigure(void);
void SetupI2CTransmit(unsigned int dcount, unsigned int instNum);
void SetupReception(unsigned int dcount, unsigned int instNum);
void cleanupInterrupts(unsigned int instNum);
unsigned int getI2CAddr(unsigned int instNum);

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Get I2C base address of the instance passed
*/
unsigned int getI2CAddr(unsigned int instNum)
{
	if(I2C_0 == instNum)
		return SOC_I2C_0_REGS;
	if(I2C_1 == instNum)
		return SOC_I2C_1_REGS;
	if(I2C_2 == instNum)
		return SOC_I2C_2_REGS;

	return 0;
}

/*
** Clear the I2C interrupts for the given instance
*/
void cleanupInterrupts(unsigned int instNum)
{
    unsigned int instAddr;
    instAddr = getI2CAddr(instNum);

    I2CMasterIntEnableEx(instAddr, 0x7FF);
    I2CMasterIntClearEx(instAddr,  0x7FF);
    I2CMasterIntDisableEx(instAddr, 0x7FF);
}

/*
** Configure I2C0 on which the PMIC is interfaced
*/
void I2CInit(unsigned int instNum)
{
    unsigned int instAddr;

    instAddr = getI2CAddr(instNum);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(instAddr);

    I2CSoftReset(instAddr);

    /* Disable auto Idle functionality */
    I2CAutoIdleDisable(instAddr);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(instAddr, I2C_SYSTEM_CLOCK, I2C_INTERNAL_CLOCK,
						    I2C_OUTPUT_CLOCK);

    I2COwnAddressSet(instAddr, 0, I2C_OWN_ADDR_0);

    /* Bring I2C module out of reset */
    I2CMasterEnable(instAddr);

    while(!I2CSystemStatusGet(instAddr));
}

/*
** Configures AINTC to get PMIC interrupt
*/
void I2CIntRegister(unsigned int instNum)
{
	if(I2C_0 == instNum)
		IntRegister(SYS_INT_I2C0INT, I2C0Isr);
}

/*
** I2C0 Interrupt Service Routine. This function will read and write
** data through I2C bus.
*/
void I2C0Isr(void)
{
    unsigned int status = 0;

    status = I2CMasterIntStatus(SOC_I2C_0_REGS);

    I2CMasterIntClearEx(SOC_I2C_0_REGS,
	                    (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));

    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
        dataFromSlave[I2C_0][rCount[I2C_0]++] = I2CMasterDataGet(SOC_I2C_0_REGS);
	I2CMasterIntClearEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY);

         if(rCount[I2C_0] == numOfBytes[I2C_0])
         {
              I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
              /* Generate a STOP */
              I2CMasterStop(SOC_I2C_0_REGS);

         }
    }
    if (status & I2C_INT_TRANSMIT_READY)
    {
         /* Put data to data transmit register of i2c */

        I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[I2C_0][tCount[I2C_0]++]);
	I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

         if(tCount[I2C_0] == numOfBytes[I2C_0])
         {
	      I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

         }
    }

    if (status & I2C_INT_STOP_CONDITION)
    {
	 /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
					       I2C_INT_STOP_CONDITION);
         flag[I2C_0] = 0;
    }

    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY  |
                                                 I2C_INT_RECV_READY      |
                                                 I2C_INT_NO_ACK          |
                                                 I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(SOC_I2C_0_REGS);

         flag[I2C_0] = 0;
    }
}


/*
** Transmits data over I2C0 bus
*/
void SetupI2CTransmit(unsigned int dcount, unsigned int instNum)
{
    unsigned int instAddr;

    instAddr = getI2CAddr(instNum);

    I2CSetDataCount(instAddr, dcount);

    numOfBytes[instNum] = I2CDataCountGet(instAddr);

    cleanupInterrupts(instNum);

    I2CMasterControl(instAddr, I2C_CFG_MST_TX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(instAddr, I2C_INT_TRANSMIT_READY |
	                                      I2C_INT_STOP_CONDITION);
    I2CMasterStart(instAddr);

    while(I2CMasterBusBusy(instAddr) == 0);

    while(flag[instNum]);

    while(I2CMasterBusy(instAddr));

    while(0 == (I2CMasterIntRawStatus(instAddr) & I2C_INT_ADRR_READY_ACESS));

    flag[instNum] = 1;
}

/*
** Receives data over I2C0 bus
*/
void SetupReception(unsigned int dcount, unsigned int instNum)
{
    unsigned int instAddr;

    instAddr = getI2CAddr(instNum);

    I2CSetDataCount(instAddr, 1);

    numOfBytes[instNum] = I2CDataCountGet(instAddr);

    cleanupInterrupts(instNum);

    I2CMasterControl(instAddr, I2C_CFG_MST_TX );

    I2CMasterIntEnableEx(instAddr, I2C_INT_TRANSMIT_READY);

    I2CMasterStart(instAddr);

    while(I2CMasterBusBusy(instAddr) == 0);

    while(tCount[instNum] != 0x01);

    while(0 == (I2CMasterIntRawStatus(instAddr) & I2C_INT_ADRR_READY_ACESS));

    I2CSetDataCount(instAddr, dcount);

    numOfBytes[instNum] = I2CDataCountGet(instAddr);

    cleanupInterrupts(instNum);

    I2CMasterControl(instAddr, I2C_CFG_MST_RX);

    I2CMasterIntEnableEx(instAddr, I2C_INT_RECV_READY |
	                           I2C_INT_STOP_CONDITION );

    I2CMasterStart(instAddr);

    while(I2CMasterBusBusy(instAddr) == 0);

    while(flag[instNum]);

    flag[instNum] = 1;
}
