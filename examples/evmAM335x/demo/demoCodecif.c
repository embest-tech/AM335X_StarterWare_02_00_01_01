/**
 * \file  demoCodecif.c
 *
 * \brief Functions to configure the codec trough i2c or other interfaces.
 *
 * Currently only one interface type is allowed. If another interface to be
 * used, this need enhancement.
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

#include "soc_AM335x.h"
#include "interrupt.h"
#include "hsi2c.h"
#include "demoCodecif.h"
#include "demoI2c.h"


/******************************************************************************
**              GLOBAL FUNCTION DECLARATIONS
******************************************************************************/	
extern volatile unsigned char dataFromSlave[I2C_INSTANCE][2];
extern volatile unsigned char dataToSlave[I2C_INSTANCE][3];
extern volatile unsigned int  tCount[I2C_INSTANCE];
extern volatile unsigned int  rCount[I2C_INSTANCE];
extern volatile unsigned int  flag[I2C_INSTANCE];
extern volatile unsigned int  numOfBytes[I2C_INSTANCE];


/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void I2CCodecSendBlocking(unsigned int baseAddr, unsigned int dataCnt);
static void I2CCodecRcvBlocking(unsigned int baseAddr, unsigned int dataCnt);

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/

/*
** Function to send data through i2c
*/
static void I2CCodecSendBlocking(unsigned int baseAddr, unsigned int dataCnt)
{
    flag[I2C_1] = 1;
	tCount[I2C_1] = 0;
 
    I2CSetDataCount(baseAddr, dataCnt);

    I2CMasterControl(baseAddr, I2C_CFG_MST_TX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(baseAddr, I2C_INT_TRANSMIT_READY | I2C_INT_STOP_CONDITION);

    I2CMasterStart(baseAddr);
   
    /* Wait till the data is sent */ 
    while(flag[I2C_1]);
}

/*
** Function to receive data from the Codec through I2C bus
*/
static void I2CCodecRcvBlocking(unsigned int baseAddr, unsigned int dataCnt)
{
    flag[I2C_1] = 1;
	rCount[I2C_1] = 0;
    
    I2CSetDataCount(baseAddr, dataCnt);

    I2CMasterControl(baseAddr, I2C_CFG_MST_RX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(baseAddr, I2C_INT_RECV_READY | I2C_INT_STOP_CONDITION);

    I2CMasterStart(baseAddr);

    /* Wait till data is received fully */
    while(flag[I2C_1]);
}


/*
** Writes a codec register with the given data value
*/
void CodecRegWrite(unsigned int baseAddr, unsigned char regAddr,
                   unsigned char regData)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address and data */
    dataToSlave[I2C_1][0] = regAddr;
    dataToSlave[I2C_1][1] = regData;
	
	I2CCodecSendBlocking(baseAddr, 2);
#endif
}

/*
** Clears codec register bits specified in the bit mask
*/
void CodecRegBitClr(unsigned int baseAddr, unsigned char regAddr,    
                    unsigned char bitMask)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    dataToSlave[I2C_1][0] = regAddr;
    I2CCodecSendBlocking(baseAddr, 1);

    /* Receive the register contents in dataToSlave */
	I2CCodecRcvBlocking(baseAddr, 1);

    dataToSlave[I2C_1][1] =  dataFromSlave[I2C_1][0] & ~bitMask;
    dataToSlave[I2C_1][0] = regAddr;
   
    I2CCodecSendBlocking(baseAddr, 2);

#endif
}

/***************************** End Of File ***********************************/
