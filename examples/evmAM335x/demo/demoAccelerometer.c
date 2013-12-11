/**
 * \file   demoAccelerometer.c
 *
 * \brief  This application invokes APIs of HSI2C
 *         to control Accelerometer.
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
#include "gpio_v2.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "demoI2c.h"
#include "demoMain.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
/* I2C address of LIS331DLH accelerometer*/
#define  I2C_ACCLERO_SLAVE_ADDR       (0x18)

#define  CTRL_REG1                    (0x20)
#define  CTRL_REG2                    (0x21)
#define  CTRL_REG3                    (0x22)
#define  CTRL_REG4                    (0x23)
#define  CTRL_REG5                    (0x24)
#define  CTRL_REG6                    (0x27)
#define  OUT_X_L_DATA                 (0x28)
#define  OUT_X_H_DATA                 (0x29)
#define  OUT_Y_L_DATA                 (0x2A)
#define  OUT_Y_H_DATA                 (0x2B)
#define  OUT_Z_L_DATA                 (0x2C)
#define  OUT_Z_H_DATA                 (0x2D)

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void SetupAccTransmit(unsigned int dcount);
static void SetupAccReception(unsigned int dcount);

static void AccelerometerRegRead(unsigned char regOffset,
                                 unsigned char* data);
static void AccelerometerRegWrite(unsigned char regOffset,
                                  unsigned char data);
static int binarySearch(unsigned int, unsigned int);

extern void cleanupInterrupts(unsigned int instNum);
/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
extern volatile unsigned char dataFromSlave[I2C_INSTANCE][2];
extern volatile unsigned char dataToSlave[I2C_INSTANCE][3];
extern volatile unsigned int  tCount[I2C_INSTANCE];
extern volatile unsigned int  rCount[I2C_INSTANCE];
extern volatile unsigned int  numOfBytes[I2C_INSTANCE];
extern volatile unsigned int  flag[I2C_INSTANCE];

int x_direction = 1, y_direction = 1, z_direction = 1;
int Index;

unsigned int cosine [] = {10000, 99985, 99939, 99863, 99756, 99619, 99452, 99255,
                          99027, 98769, 98481, 98163, 97815, 97437, 97437,
                          96593, 96126, 95630, 95106, 94552, 93969, 93358,
                          92718, 92050, 91355, 90631, 89879, 89101, 88295,
                          87462, 85717, 84805, 83867, 82904, 81915, 80902,
                          79864, 78801, 77715, 76604, 75471, 74314, 73135,
                          71934, 69466, 68200, 66913, 65606, 64279, 62932,
                          61566, 60182, 58779, 57358, 55919, 54464, 52992,
                          51504, 48481, 46947, 45399, 43837, 42262, 40674,
                          39073, 37461, 35837, 34202, 32557, 30902, 29237,
                          27564, 25882, 24192, 22495, 20791, 19081, 17365,
                          15643, 13917, 12187, 10453, 8716,  6976,  5234,
                          3490,  1745, 0};

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

void initAccelerometer()
{
    volatile unsigned int delay = 0xffff;
	
	I2CMasterSlaveAddrSet(SOC_I2C_1_REGS, I2C_ACCLERO_SLAVE_ADDR);

    /*
    ** Configure Accelerometer in normal mode with ODR
    ** (output data rate) of 100hz and enable x,y and
    ** z axis
    */
    AccelerometerRegWrite(CTRL_REG1, 0x2f);

    /* Internal filter is bypassed */
    AccelerometerRegWrite(CTRL_REG2, 0x00);

    /*
    ** Enable Block data update,littel
    ** endian selected and full scale
    ** is configured to 2g.
    */
    AccelerometerRegWrite(CTRL_REG4, 0x80);

    /* sleep to wakeup function is disabled */
    AccelerometerRegWrite(CTRL_REG5, 0x00);

    /* power on delay */
    while(delay--);
}

void getAcceleroPos(int* tiltDir, int* tiltDeg)
{
    unsigned char x_high;
    unsigned char y_high;
    unsigned char z_high;
    unsigned char x_low;
    unsigned char y_low;
    unsigned char z_low;
    int x_axis;
    int y_axis;
    int z_axis;
	
	I2CMasterSlaveAddrSet(SOC_I2C_1_REGS, I2C_ACCLERO_SLAVE_ADDR);

	 AccelerometerRegRead(OUT_X_L_DATA, &x_low);
	 AccelerometerRegRead(OUT_X_H_DATA, &x_high);
	 AccelerometerRegRead(OUT_Y_L_DATA, &y_low);
	 AccelerometerRegRead(OUT_Y_H_DATA, &y_high);
	 AccelerometerRegRead(OUT_Z_L_DATA, &z_low);
	 AccelerometerRegRead(OUT_Z_H_DATA, &z_high);

	 x_axis = x_low |(unsigned short int)x_high << 8;
	 y_axis = y_low |(unsigned short int)y_high << 8;
	 z_axis = z_low |(unsigned short int)z_high << 8;

	 x_axis = (x_axis >> 4) & 0xfff;
	 y_axis = (y_axis >> 4) & 0xfff;
	 z_axis = (z_axis >> 4) & 0xfff;
	 
	 if(x_axis & 0x800)
	 {
		  x_axis = ~x_axis;
		  x_axis =  x_axis + 1;
		  x_axis =  x_axis & 0xfff;
		  x_direction = -1;
	 }

	 if(y_axis & 0x800)
	 {
		  y_axis = ~y_axis;
		  y_axis =  y_axis + 1;
		  y_axis =  y_axis & 0xfff;
		  y_direction = -1;
	 }
	
	 if(z_axis & 0x800)
	 {
		  z_axis = ~z_axis;
		  z_axis = z_axis + 1;
		  z_axis = z_axis & 0xfff;
		  z_direction = -1;
	 } 

	 z_axis = (z_axis * 100000) / 1024; 

	 Index = binarySearch(z_axis, 90);
	 
	 if( Index != 0)
	 {

		  if( y_axis > x_axis)
		  {
			   if(y_direction == -1)
			   {
					y_direction = 1;
					*tiltDir = RIGHT_TILT;
			   }
			   else
			   {
					*tiltDir = LEFT_TILT;
			   }
		  }
		  else
		  {
				if(x_direction == -1)
				{
					x_direction = 1;
					*tiltDir = FORWARD_TILT;
				}
				else
				{
					*tiltDir = BACK_TILT;
				}
		  }
	}
	 
	if(((y_direction == 1) && (z_direction == -1)) || 
	  ((y_direction == -1) && (z_direction == -1)))
	{
		  Index = 180 - Index;
	}
	
	x_direction = 1;
	y_direction = 1;
	z_direction = 1; 

	*tiltDeg = Index;
}

/*
** This function searches for the fraction in the cosine lookup table.
** The return value is the index where fraction is found. This is nothing
** but the degree by which device is tilted i.e (cosine-inverse(z_axis/g)
** where g = acceleration due to gravity.
** Here, value - This is the ratio of axis value(X, Y or Z) to the acceleration
**               due to gravity.
**       N - Number of elements present in the Cosine look-up table
*/

static int binarySearch(unsigned int value, unsigned int N)
{
    int mid = 0;
    int high;
    int low;

    low = 0;
    high = N - 1;

    while(low <= high)
    {
         mid = (low + high) / 2;

         if(value < cosine[mid])
         {
              low = mid + 1;
         }
         else if(value > cosine[mid])
         {
              high = mid - 1;
         }
         else
         {
              return mid;
         }
    }

    return mid;
}

static void AccelerometerRegWrite(unsigned char regOffset,
                                  unsigned char data)
{
    dataToSlave[I2C_1][0] = regOffset;
    dataToSlave[I2C_1][1] = data;

    tCount[I2C_1] = 0;
    SetupAccTransmit(2);
}

static void AccelerometerRegRead(unsigned char regOffset,
                                 unsigned char* data)
{
    tCount[I2C_1] = 0;
    rCount[I2C_1] = 0;

    dataToSlave[I2C_1][0] = regOffset;
        
    SetupAccReception(1);

    *data = dataFromSlave[I2C_1][0];
} 


/*
** Transmits data over I2C bus 
*/
static void SetupAccTransmit(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_1_REGS, dcount);

    numOfBytes[I2C_1] = I2CDataCountGet(SOC_I2C_1_REGS);

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_TX | I2C_CFG_STOP);

    /* Transmit and Stop condition interrupt is enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                         I2C_INT_STOP_CONDITION );

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(flag[I2C_1]);

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(SOC_I2C_1_REGS) & (I2C_INT_ADRR_READY_ACESS)));

    flag[I2C_1] = 1;
}

/*
** Receives data over I2C bus 
*/
static void SetupAccReception(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_1_REGS, 0x01);

    numOfBytes[I2C_1] = I2CDataCountGet(SOC_I2C_1_REGS);

    /* Clear status all interrupts */
    cleanupInterrupts(I2C_1);

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(I2CMasterBusBusy(SOC_I2C_1_REGS) == 0);

    while(tCount[I2C_1] != numOfBytes[I2C_1]);

    flag[I2C_1] = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(SOC_I2C_1_REGS) & (I2C_INT_ADRR_READY_ACESS)));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(SOC_I2C_1_REGS, dcount);

    numOfBytes[I2C_1] = I2CDataCountGet(SOC_I2C_1_REGS);

    cleanupInterrupts(I2C_1);

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS,  I2C_INT_RECV_READY |
                                          I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(I2CMasterBusBusy(SOC_I2C_1_REGS) == 0);

    while(flag[I2C_1]);

    flag[I2C_1] = 1;
}

