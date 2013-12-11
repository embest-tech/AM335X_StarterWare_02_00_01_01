/**
 * \file   cpld.c
 *
 * \brief  This file contains functions which read/write from the cpld
 *         to get the EVM profile that is configured at present
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
#include "hw_control_AM335x.h"
#include "hw_cm_per.h"
#include "hw_cm_wkup.h"
#include "hw_types.h"
#include "hsi2c.h"
#include "evmAM335x.h"

/******************************************************************************
	**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void cleanupInterrupts(void);
static void I2C0PinMux(void);
/******************************************************************************
**                      MACROS
*******************************************************************************/

#define CPLD_I2C_ADDR        0x35
#define CPLD_I2C_BASE        SOC_I2C_0_REGS
#define CPLD_CTRLREG_OFFSET    0x10

/**
 * \brief   Reads data from the CPLD register offset
 *
 * \return  none
 *
 * \note    interrupts are not used for simplicity. In case interrupts are
 *          desired, it should be handled by the application
 */
void CPLDI2CRead(unsigned char *data, unsigned int length, unsigned char offset)
{
    unsigned int i = 0;

    /* First send the register offset - TX operation */
    I2CSetDataCount(CPLD_I2C_BASE, 1);

    cleanupInterrupts();

    I2CMasterControl(CPLD_I2C_BASE, I2C_CFG_MST_TX);

    I2CMasterStart(CPLD_I2C_BASE);

    /* Wait for the START to actually occur on the bus */
    while (0 == I2CMasterBusBusy(CPLD_I2C_BASE));

    /* Wait for the Tx register to be empty */
    while (0 == I2CMasterIntRawStatusEx(CPLD_I2C_BASE, I2C_INT_TRANSMIT_READY));

    /* Push offset out and tell CPLD from where we intend to read the data */
    I2CMasterDataPut(CPLD_I2C_BASE, offset);

    I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

    while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_ADRR_READY_ACESS));

    I2CSetDataCount(SOC_I2C_0_REGS, length);

    cleanupInterrupts();

    /* Now that we have sent the register offset, start a RX operation*/
    I2CMasterControl(CPLD_I2C_BASE, I2C_CFG_MST_RX);

    /* Repeated start condition */
    I2CMasterStart(CPLD_I2C_BASE);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while (length--)
    {
        while (0 == I2CMasterIntRawStatusEx(CPLD_I2C_BASE, I2C_INT_RECV_READY));
        data[i] = (unsigned char)I2CMasterDataGet(CPLD_I2C_BASE);
        I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
    }

    I2CMasterStop(CPLD_I2C_BASE);

    while(0 == (I2CMasterIntRawStatus(SOC_I2C_0_REGS) & I2C_INT_STOP_CONDITION));

    I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_STOP_CONDITION);

}

/* Clear the status of all interrupts */
static void cleanupInterrupts(void)
{
    I2CMasterIntClearEx(SOC_I2C_0_REGS,  0x7FF);
}

/**
 * \brief   Configures the I2C Instance
 *
 * \return  none
 */
void CPLDI2CSetup(void)
{
    /* Configuring system clocks for I2C0 instance. */
    I2C0ModuleClkConfig();

    /* Performing Pin Multiplexing for I2C0. */
    I2C0PinMux();

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(CPLD_I2C_BASE);

    /* Auto Idle functionality is Disabled */
    I2CAutoIdleDisable(CPLD_I2C_BASE);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(CPLD_I2C_BASE, 48000000, 12000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(CPLD_I2C_BASE, CPLD_I2C_ADDR);

    /* Disable all I2C interrupts */
    I2CMasterIntDisableEx(CPLD_I2C_BASE, 0xFFFFFFFF);

    /* Bring I2C module out of reset */
    I2CMasterEnable(CPLD_I2C_BASE);

    while(!I2CSystemStatusGet(SOC_I2C_0_REGS));
}

/**
 * \brief   Reads the profile configured for the EVM from CPLD
 *
 * \return  profile number
 */
unsigned int EVMProfileGet(void)
{
    unsigned int data = 0;

    CPLDI2CSetup();

    CPLDI2CRead((unsigned char*)&data, 1, CPLD_CTRLREG_OFFSET);

    return (data & 0x07);
}

static void I2C0PinMux(void)
{
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SDA)  =
         (CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_RXACTIVE  |
          CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_SLEWCTRL  |
          CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_PUTYPESEL   );

    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SCL)  =
         (CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_RXACTIVE  |
          CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_SLEWCTRL  |
          CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_PUTYPESEL );
} 
