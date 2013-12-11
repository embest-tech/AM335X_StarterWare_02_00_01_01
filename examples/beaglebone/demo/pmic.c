/**
 *  \file   pmic.c
 *
 *  \brief  PMIC APIs
 *
 *   This file contains API's to access PMIC registers
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
#include "interrupt.h"
#include "soc_AM335x.h"
#include "pmic.h"
#include "demoI2c.h"
#include "demoCfg.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/


/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
extern volatile unsigned char dataFromSlave[I2C_INSTANCE][2];
extern volatile unsigned char dataToSlave[I2C_INSTANCE][3];
extern volatile unsigned int  tCount[I2C_INSTANCE];
extern volatile unsigned int  rCount[I2C_INSTANCE];
extern volatile unsigned int  flag[I2C_INSTANCE];
extern volatile unsigned int  numOfBytes[I2C_INSTANCE];

extern void SetupI2CTransmit(unsigned int dcount, unsigned int instNum);
extern void SetupReception(unsigned int dcount, unsigned int instNum);


/**
 *  \brief            - Generic function that can write a TPS65217 PMIC
 *                      register or bit field regardless of protection
 *                      level.
 *
 * \param prot_level  - Register password protection.
 *                      use PROT_LEVEL_NONE, PROT_LEVEL_1, or PROT_LEVEL_2
 * \param regOffset:  - Register address to write.
 *
 * \param dest_val    - Value to write.
 *
 * \param mask        - Bit mask (8 bits) to be applied.  Function will only
 *                      change bits that are set in the bit mask.
 *
 * \return:            None.
 */
void TPS65217RegWrite(unsigned char port_level, unsigned char regOffset,
                      unsigned char dest_val, unsigned char mask)
{
    unsigned char read_val;
    unsigned xor_reg;

    dataToSlave[I2C_0][0] = regOffset;
    tCount[I2C_0] = 0;
    rCount[I2C_0] = 0;

    if(mask != MASK_ALL_BITS)
    {
         SetupReception(1, I2C_0);

         read_val = dataFromSlave[I2C_0][0];
         read_val &= (~mask);
         read_val |= (dest_val & mask);
         dest_val = read_val;
    }

    if(port_level > 0)
    {
         xor_reg = regOffset ^ PASSWORD_UNLOCK;

         dataToSlave[I2C_0][0] = PASSWORD;
         dataToSlave[I2C_0][1] = xor_reg;
         tCount[I2C_0] = 0;

         SetupI2CTransmit(2, I2C_0);
    }

    dataToSlave[I2C_0][0] = regOffset;
    dataToSlave[I2C_0][1] = dest_val;
    tCount[I2C_0] = 0;

    SetupI2CTransmit(2, I2C_0);

    if(port_level == PROT_LEVEL_2)
    {
         dataToSlave[I2C_0][0] = PASSWORD;
         dataToSlave[I2C_0][1] = xor_reg;
         tCount[I2C_0] = 0;

         SetupI2CTransmit(2, I2C_0);

         dataToSlave[I2C_0][0] = regOffset;
         dataToSlave[I2C_0][1] = dest_val;
         tCount[I2C_0] = 0;

         SetupI2CTransmit(2, I2C_0);
    }
}

/**
 *  \brief              - Controls output voltage setting for the DCDC1,
 *                        DCDC2, or DCDC3 control registers in the PMIC.
 *
 * \param  dc_cntrl_reg   DCDC Control Register address.
 *                        Must be DEFDCDC1, DEFDCDC2, or DEFDCDC3.
 *
 * \param  volt_sel       Register value to set.  See PMIC TRM for value set.
 *
 * \return:               None.
 */
void TPS65217VoltageUpdate(unsigned char dc_cntrl_reg, unsigned char volt_sel)
{
    /* set voltage level */
    TPS65217RegWrite(PROT_LEVEL_2, dc_cntrl_reg, volt_sel, MASK_ALL_BITS);

    /* set GO bit to initiate voltage transition */
    TPS65217RegWrite(PROT_LEVEL_2, DEFSLEW, DCDC_GO, DCDC_GO);
}


void configVddOpVoltage(void)
{
    /* Configure PMIC slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_TPS65217_I2C_SLAVE_ADDR);

    /* Increase USB current limit to 1300mA */
    TPS65217RegWrite(PROT_LEVEL_NONE, POWER_PATH, USB_INPUT_CUR_LIMIT_1300MA,
                     USB_INPUT_CUR_LIMIT_MASK);

    /* Set LDO3, LDO4 output voltage to 3.3V */
    TPS65217RegWrite(PROT_LEVEL_2, DEFLS1, LDO_VOLTAGE_OUT_3_3, LDO_MASK);

    TPS65217RegWrite(PROT_LEVEL_2, DEFLS2, LDO_VOLTAGE_OUT_3_3, LDO_MASK);
}

/*
** set VDD1_OP voltage value
*/
void setVdd1OpVoltage(unsigned int opVolSelector)
{
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);
    TPS65217VoltageUpdate(DEFDCDC2, opVolSelector);
}
