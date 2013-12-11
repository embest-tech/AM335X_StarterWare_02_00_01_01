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
#include "hw_tps65910.h"

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


/*
** Configure VDD1 for various parameters such as 
** 		-	Multiplier
** 		-	Maximum Load Current
** 		-	Time step - voltage change per us(micro sec)
** 		-	Supply state (on (high/low power mode), off)
*/
void configureVdd1(unsigned int opVolMultiplier, unsigned maxLoadCurrent,
							unsigned int timeStep, unsigned int supplyState)
{
    dataToSlave[I2C_0][0] = VDD1_REG;
    dataToSlave[I2C_0][1] = (
						(opVolMultiplier << PMIC_VDD1_REG_VGAIN_SEL_SHIFT) |
						(maxLoadCurrent << PMIC_VDD1_REG_ILMAX_SHIFT) |
						(timeStep << PMIC_VDD1_REG_TSTEP_SHIFT) |
						(supplyState << PMIC_VDD1_REG_ST_SHIFT)
					 );
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;	
    SetupI2CTransmit(2, I2C_0);
}


/*
** Select the VDD1 value. VDD1_OP_REG or VDD1_SR_REG.
*/
void selectVdd1Source(unsigned int vdd1Source)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = VDD1_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	vdd1Source = (dataFromSlave[I2C_0][0] & (~PMIC_VDD1_OP_REG_CMD)) | 
							(vdd1Source << PMIC_VDD1_OP_REG_CMD_SHIFT);
	
	/*	Write reg value	*/
    dataToSlave[I2C_0][0] = VDD1_OP_REG;
    dataToSlave[I2C_0][1] = vdd1Source;
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}


/*
** set VDD1_OP voltage value
*/
void setVdd1OpVoltage(unsigned int opVolSelector)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = VDD1_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	opVolSelector = (dataFromSlave[I2C_0][0] & (~PMIC_VDD1_OP_REG_SEL)) | 
							(opVolSelector << PMIC_VDD1_OP_REG_SEL_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = VDD1_OP_REG;
    dataToSlave[I2C_0][1] = opVolSelector;
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;	
    SetupI2CTransmit(2, I2C_0);	
	
	/*	Read reg value to verify	*/
    dataToSlave[I2C_0][0] = VDD1_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);
	
	while((dataFromSlave[I2C_0][0] & PMIC_VDD1_OP_REG_SEL) != (opVolSelector << PMIC_VDD1_OP_REG_SEL_SHIFT));

}


/*
** get VDD1_OP voltage value
*/
unsigned int Vdd1OpVoltageGet(void)
{
    unsigned int Opp;

    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);
     /*	Read reg value to verify */
    dataToSlave[I2C_0][0] = VDD1_OP_REG;
    dataFromSlave[I2C_0][0] = 0; // clear receive buffer
    dataFromSlave[I2C_0][1] = 0;
    rCount[I2C_0] = 0;
    tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

    Opp = dataFromSlave[I2C_0][0];

    return Opp;
}




/*
** set VDD1_SR voltage value
*/
void setVdd1SrVoltage(unsigned int opVolSelector)
{
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = VDD1_SR_REG;
    dataToSlave[I2C_0][1] = opVolSelector;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}

/*#################*/
/*
** Configure vdd2 for various parameters such as 
** 		-	Multiplier
** 		-	Maximum Load Current
** 		-	Time step - voltage change per us(micro sec)
** 		-	Supply state (on (high/low power mode), off)
*/
void configureVdd2(unsigned int opVolMultiplier, unsigned maxLoadCurrent,
							unsigned int timeStep, unsigned int supplyState)
{
    dataToSlave[I2C_0][0] = VDD2_REG;
    dataToSlave[I2C_0][1] = (
						(opVolMultiplier << PMIC_VDD2_REG_VGAIN_SEL_SHIFT) |
						(maxLoadCurrent << PMIC_VDD2_REG_ILMAX_SHIFT) |
						(timeStep << PMIC_VDD2_REG_TSTEP_SHIFT) |
						(supplyState << PMIC_VDD2_REG_ST_SHIFT)
					 );
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;	
    SetupI2CTransmit(2, I2C_0);
}


/*
** Select the VDD2 value. VDD2_OP_REG or VDD2_SR_REG.
*/
void selectVdd2Source(unsigned int vdd1Source)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = VDD2_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	vdd1Source = (dataFromSlave[I2C_0][0] & (~PMIC_VDD2_OP_REG_CMD)) | 
							(vdd1Source << PMIC_VDD2_OP_REG_CMD_SHIFT);
	
	/*	Write reg value	*/
    dataToSlave[I2C_0][0] = VDD2_OP_REG;
    dataToSlave[I2C_0][1] = vdd1Source;
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}


/*
** set VDD2_OP voltage value
*/
void setVdd2OpVoltage(unsigned int opVolSelector)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = VDD2_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	opVolSelector = (dataFromSlave[I2C_0][0] & (~PMIC_VDD2_OP_REG_SEL)) | 
							(opVolSelector << PMIC_VDD2_OP_REG_SEL_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = VDD2_OP_REG;
    dataToSlave[I2C_0][1] = opVolSelector;
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;	
    SetupI2CTransmit(2, I2C_0);	
	
	/*	Read reg value to verify	*/
    dataToSlave[I2C_0][0] = VDD2_OP_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);
	
	while((dataFromSlave[I2C_0][0] & PMIC_VDD2_OP_REG_SEL) != (opVolSelector << PMIC_VDD2_OP_REG_SEL_SHIFT));

}


/*
** set VDD2_SR voltage value
*/
void setVdd2SrVoltage(unsigned int opVolSelector)
{
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = VDD2_SR_REG;
    dataToSlave[I2C_0][1] = opVolSelector;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}

/*#################*/
/*
** Store data in backup registers
*/
void storeBackupValue(unsigned int backupReg, unsigned char value)
{
	dataToSlave[I2C_0][0] = backupReg;
    dataToSlave[I2C_0][1] = value;

    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);		
}


/*
** Get data from backup registers
*/
unsigned char readBackupValue(unsigned int backupReg)
{
    dataToSlave[I2C_0][0] = backupReg;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	return dataFromSlave[I2C_0][0];
}


/*
** Configure VPLL settings (voltage & supply state)
*/
void configureVpll(unsigned int selectVoltage, unsigned int supplyState)
{
    dataToSlave[I2C_0][0] = VPLL_REG;
    dataToSlave[I2C_0][1] = (
						(selectVoltage << PMIC_VPLL_REG_SEL_SHIFT) |
						(supplyState << PMIC_VPLL_REG_ST_SHIFT)
					 );
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);
}


/*
** Enable thermal shutdown
*/
void enableThermalShutDown(void)
{
	unsigned char regVal = 0;

	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = THERM_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	regVal = (dataFromSlave[I2C_0][0] & PMIC_THERM_REG_THERM_STATE) | 
							(PMIC_THERM_REG_THERM_STATE_ENABLE << PMIC_THERM_REG_THERM_STATE_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = THERM_REG;
    dataToSlave[I2C_0][1] = regVal;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}


/*
** Disable thermal shutdown
*/
void disableThermalShutDown(void)
{
	unsigned char regVal = 0;

	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = THERM_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	regVal = (dataFromSlave[I2C_0][0] & PMIC_THERM_REG_THERM_STATE) | 
			(PMIC_THERM_REG_THERM_STATE_DISABLE << PMIC_THERM_REG_THERM_STATE_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = THERM_REG;
    dataToSlave[I2C_0][1] = regVal;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}

/*
** Set Hot die threshold
*/
void setHotdieDetThreshold(unsigned int thresholdValue)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = THERM_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	thresholdValue = (dataFromSlave[I2C_0][0] & PMIC_THERM_REG_THERM_HDSEL) | 
							(thresholdValue << PMIC_THERM_REG_THERM_HDSEL_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = THERM_REG;
    dataToSlave[I2C_0][1] = thresholdValue;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);	
}


/*
** Enable thermal shutdown
*/
unsigned char getThermalStatus(unsigned int statusMask)
{
	rCount[I2C_0] = 0;
    dataToSlave[I2C_0][0] = THERM_REG;
    SetupReception(1, I2C_0);
	return ((dataFromSlave[I2C_0][0] & PMIC_THERM_REG_THERM_TS) >> PMIC_THERM_REG_THERM_TS_SHIFT);
}


/*
** Select I2C interface whether SR I2C or Control I2C
*/
void selectI2CInstance(unsigned int i2cInstance)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = DEVCTRL_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	i2cInstance = (dataFromSlave[I2C_0][0] & PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL) | 
							(i2cInstance << PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL_SHIFT);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = DEVCTRL_REG;
    dataToSlave[I2C_0][1] = i2cInstance;
    tCount[I2C_0] = 0;
	rCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);
}

/*
** Get PMIC interrupt status
*/
tBoolean getInterruptStatus(unsigned char intMask)
{
	rCount[I2C_0] = 0;
    dataToSlave[I2C_0][0] = INT_STS_REG;
    SetupReception(1, I2C_0);
	return ((dataFromSlave[I2C_0][0] & intMask) == intMask);
}


/*
** Clear PMIC interrupt status
*/
void clearInterrupt(unsigned char intMask)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = INT_STS_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	intMask = (dataFromSlave[I2C_0][0] | intMask);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = INT_STS_REG;
    dataToSlave[I2C_0][1] = intMask;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);
}

/*
** Enable PMIC Interrupt
*/
void enableInterrupt(unsigned char intMask)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = INT_MSK_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	intMask = (dataFromSlave[I2C_0][0] & (!intMask));
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = INT_MSK_REG;
    dataToSlave[I2C_0][1] = intMask;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);
}


/*
** Disable PMIC Interrupt
*/
void disableInterrupt(unsigned char intMask)
{
	/*	Read reg value	*/
    dataToSlave[I2C_0][0] = INT_MSK_REG;
	dataFromSlave[I2C_0][0] = 0; // clear receive buffer
	dataFromSlave[I2C_0][1] = 0;
	rCount[I2C_0] = 0;
	tCount[I2C_0] = 0;	
    SetupReception(1, I2C_0);

	/*	Modify reg value	*/
	intMask = (dataFromSlave[I2C_0][0] | intMask);
	
	/*	Write reg value	*/
	dataToSlave[I2C_0][0] = INT_MSK_REG;
    dataToSlave[I2C_0][1] = intMask;
    tCount[I2C_0] = 0;
    SetupI2CTransmit(2, I2C_0);
}

/*
** Configure PMIC for operating VDD volatges
*/
void configVddOpVoltage(void)
{
    /* Configure PMIC slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);

    /* Using Control Interface of PMIC for communication with I2C0 */
    selectI2CInstance(PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL_CTL_I2C);

    /* Configure vdd1- need to validate these parameters   */
    configureVdd1(PMIC_VDD1_REG_VGAIN_SEL_X1, PMIC_VDD1_REG_ILMAX_1_5_A,
                PMIC_VDD1_REG_TSTEP_12_5, PMIC_VDD1_REG_ST_ON_HI_POW);

    /* Select the source for VDD1 control */
    selectVdd1Source(PMIC_VDD1_OP_REG_CMD_OP);

    /* Configure vdd2 - need to validate these parameters */
    configureVdd2(PMIC_VDD2_REG_VGAIN_SEL_X1, PMIC_VDD2_REG_ILMAX_1_5_A,
                PMIC_VDD2_REG_TSTEP_12_5, PMIC_VDD2_REG_ST_ON_HI_POW);

    /* Select the source for VDD2 control */
    selectVdd2Source(PMIC_VDD2_OP_REG_CMD_OP);
}
