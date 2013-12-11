/**
 * \file   demoTempSensor.c
 *
 * \brief  This application invokes APIs of HSI2C to control
 *         Temperature Sensor.
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

#include "hsi2c.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "demoI2c.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define     CONFIG_DATA(Resolution, Fault, POL, SD, TM, OS )   SD |    \
                                                               TM << 1 | \
                                                               OS << 7 |  \
                                                               POL << 2 |  \
                                                               Fault << 3 | \
                                                               Resolution << 5;


/* I2C address of TMP275 temperature sensor */
#define  I2C_TEMP_SENS_SLAVE_ADDR         (0x48)


/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void TemperatureCalc(unsigned char *data, int *result);
static unsigned int DecimalValGet(short int val);
static void TemperatureRegRead(unsigned char *data, unsigned char ptrReg);
static void ConfigureTempSensor(unsigned char configData, unsigned char ptrReg);

extern void SetupI2CTransmit(unsigned int dcount, unsigned int instNum);
extern void SetupReception(unsigned int dcount, unsigned int instNum);

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
unsigned char temperature[2];
int result[2];
unsigned char ptrReg;
	
/******************************************************************************
**              GLOBAL FUNCTION DECLARATIONS
******************************************************************************/	
extern volatile unsigned char dataFromSlave[I2C_INSTANCE][2];
extern volatile unsigned char dataToSlave[I2C_INSTANCE][3];
extern volatile unsigned int  tCount[I2C_INSTANCE];
extern volatile unsigned int  rCount[I2C_INSTANCE];

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

void initTempSensor(void)
{
    unsigned char configData;
    
    /*
    **  configures temperature sensor to 12bit resolution
    **  active low polarity of alert pin ,generate the 1 consecutive fault,
    **  compare mode of operation.
    */  
    configData = CONFIG_DATA(3, 0, 1, 0, 0,0);

    /* 
    ** Configure pointer register to select configuration
    ** register
    */
    ptrReg = 1;
	
    ConfigureTempSensor(configData, ptrReg);
}

void updateTemperature(unsigned int *intVal, unsigned int *floatVal)
{
	/* 
	** Configure pointer register to read temperature
	** register
	*/
	ptrReg = 0;
    
	TemperatureRegRead(temperature, ptrReg);
    
	TemperatureCalc(temperature, result);
    
	*intVal = result[0];
	*floatVal = result[1];
}

/* It returns the temperature in degree */
static void TemperatureCalc(unsigned char *data, int *result)
{
    short int val1;
    short int val2;
    int temp;

    val1 = data[0] >> 4;

    val2 = (short int)data[1] << 4;

    val1 = val1 | val2;

    if(val1 & 0x800)
    {
         result[1] = DecimalValGet(val1);
    
         val1 = ~val1;

         val1 = val1 + 0x01;

         val1 = val1 & 0x0fff;

         temp = (short int)val1 >> 4;

         temp = temp * (-1);

         result[0] = temp;
    }
    else
    {
         result[1] = DecimalValGet(val1);

         temp = (short int)val1 >> 4;

         result[0] = temp; 
    }
}

static unsigned int DecimalValGet(short int val)
{
    unsigned int i = 0;
    unsigned int sum = 0;

    while(i < 4)
    {
         sum +=(((val >> i) & 0x01) * 5000) / (1 << (3 - i));
         i++;
    }

    return sum;
}

/*
** This function selects Configuratio register of 
** temperature sensor and configures with
** input data
*/  
static void ConfigureTempSensor(unsigned char configData, unsigned char ptrReg)
{
    /* Reg offset of temperature sensor */
    dataToSlave[I2C_1][0] = ptrReg;

    /* Data to be written to reg Offset */
    dataToSlave[I2C_1][1] = configData;
	
	I2CMasterSlaveAddrSet(SOC_I2C_1_REGS, I2C_TEMP_SENS_SLAVE_ADDR);

    tCount[I2C_1] = 0;
    SetupI2CTransmit(2, I2C_1);
}

 /* Reads temperature value */
static void TemperatureRegRead(unsigned char *data, unsigned char ptrReg)
{
    unsigned int i;
    rCount[I2C_1] = 0;
    tCount[I2C_1] = 0;
	
	I2CMasterSlaveAddrSet(SOC_I2C_1_REGS, I2C_TEMP_SENS_SLAVE_ADDR);

    /* Reg offset of temperature sensor */
    dataToSlave[I2C_1][0] = ptrReg;
    SetupReception(2, I2C_1);
  
    for(i = 2; i ; i--)
    {
        *data = dataFromSlave[I2C_1][i-1];
         data++;
    }
}

/***************************** End Of File ************************************/
