/**
 * \file   hsi2cTempSensor.c
 *
 * \brief  This application invokes APIs of HSI2C to control Temperature
 *         Sensor to measure the surrounding temperature. The measured
 *         temperature will be displayed on console. Application continuously
 *         reads the temperature and the read value is compared with the
 *         previous value. The change in the temperature will be measured and
 *         displayed on the console.
 *
 *         Application configuration:
 *
 *             Modules Used:
 *                 I2C1
 *                 UART0
 *                 Interrupt Controller
 *
 *             Configurable Parameters:
 *                 None.
 *
 *             Hard-coded configuration of other parameters:
 *                 Bus frequency   - 100kHz
 *                 Addressing mode - 7bit
 *                 I2C Instances   - 1
 *                 Slave Address   - 0x48
 *
 *         Application Usecase:
 *         1. I2C controller will be configured in master mode.
 *         2. Master will configure the temperature sensor (part no: TMP275)
 *            by writing configuration values to sensor's registers.
 *         3. Temperature sensor will measure the surrounding temperature.
 *         4. Master will read the temperature value from sensor and
 *            displays on the console.
 *         5. If there is any change in the temperature, then the temperature
 *            measured will be updated and displayed on the console.
 *
 *         Running the example:
 *         1. On executing the example application the surrounding temperature
 *            is displayed on the console.
 *         2. Whenever there is a change in the temperature, the updated value will
 *            be displayed on the console.
 *
 *         Limitations:
 *         None.
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
#include "consoleUtils.h"
#include "evmAM335x.h"

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
#define  I2C_SLAVE_ADDR         (0x48)


/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
void I2CIsr(void);
static void SetupI2C(void);
static void I2CAintcConfigure(void);
static void cleanupInterrupts(void);
static void TemperatureCalc(unsigned char *data, int *result);
static unsigned int DecimalValGet(short int val);
static void SetupI2CTransmit(unsigned int dcount);
static void TemperatureRegRead(unsigned char *data, unsigned char ptrReg);
static void ConfigureTempSensor(unsigned char configData, unsigned char ptrReg);
static void SetupI2CReception(unsigned int dcount);


/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
volatile unsigned char dataFromSlave[2];
volatile unsigned char dataToSlave[3];
volatile unsigned int numOfBytes;
volatile unsigned int flag = 1;
volatile unsigned int tCount;
volatile unsigned int rCount;

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

int main(void)
{
    int result[2];
    int prevResult[2];
    unsigned char temperature[2];
    unsigned char ptrReg;
    unsigned char configData;

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable IRQ in CPSR */
    IntMasterIRQEnable();

    /* Configures AINTC to generate interrupt */
    I2CAintcConfigure();

    /*
    ** Configures HSI2C to Master mode to generate start
    ** condition on HSI2C bus and to transmit data at a
    ** speed of 100khz
    */
    SetupI2C();

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

     /* 
     ** Configure pointer register to read temperature
     ** register
     */
     ptrReg = 0;

     TemperatureRegRead(temperature, ptrReg);

     TemperatureCalc(temperature, result);

     ConsoleUtilsPrintf("Temperature Measured: %d.%d", result[0], result[1]);

     while(1)
     {
          ConsoleUtilsPrintf("\r");
          /* 
          ** Configure pointer register to read temperature
          ** register
          */
          ptrReg = 0;

          TemperatureRegRead(temperature, ptrReg);

          prevResult[0] = result[0];
          prevResult[1] = result[1];

          TemperatureCalc(temperature, result);

          prevResult[0] = prevResult[0] - result[0];
          prevResult[1] = prevResult[1] - result[1];
      
 
          if(prevResult[0] != 0 || prevResult[1] != 0)
          {
              ConsoleUtilsPrintf("Temperature Measured: %d.%d", result[0], result[1]);
          }
     }
}

/* It returns the temperature in degree */
static void TemperatureCalc(unsigned char *data, int *result)
{
    short int val1;
    short int val2;
    int temp;

    val1 = data[0] >> 4;

    val2 = (unsigned short int)data[1] << 4;

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
    dataToSlave[0] = ptrReg;

    /* Data to be written to reg Offset */
    dataToSlave[1] = configData;

    tCount = 0;
    SetupI2CTransmit(2);
}

 /* Reads temperature value */
static void TemperatureRegRead(unsigned char *data, unsigned char ptrReg)
{
    unsigned int i;
    rCount = 0;
    tCount = 0;

    /* Reg offset of temperature sensor */
    dataToSlave[0] = ptrReg;
    SetupI2CReception(2);
  
    for(i = 2; i ; i--)
    {
        *data = dataFromSlave[i-1];
         data++;
    }
}

 /*
 ** Configures i2c bus frequency and slave address.
 ** It also enable the clock for i2c module and
 ** does pinmuxing for the i2c.
 */
static void SetupI2C(void)
{
    I2C1ModuleClkConfig();

    I2CPinMuxSetup(1);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_1_REGS);

    /* 
    ** Upon reset Auto Idel is enabled.
    ** Hence it is disabled after reset
    */
    I2CAutoIdleDisable(SOC_I2C_1_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_1_REGS, 48000000, 12000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_1_REGS, I2C_SLAVE_ADDR);

    /* Bring I2C module out of reset */
    I2CMasterEnable(SOC_I2C_1_REGS);
}

/*
** Transmits data over I2C bus 
*/
static void SetupI2CTransmit(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transferred */
    I2CSetDataCount(SOC_I2C_1_REGS, dcount);

    numOfBytes = I2CDataCountGet(SOC_I2C_1_REGS);


    cleanupInterrupts();

    /* 
    ** Configure I2C controller in Master Transmitter mode.A stop
    ** condition will be generated after data count number of
    ** bytes are transferred.
    */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_TX | I2C_CFG_STOP);

    /* Transmit and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                         I2C_INT_STOP_CONDITION );

    /* Generated Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(flag);

    /* Wait untill I2C registers are ready to access */
    while(0 == (I2CMasterIntRawStatus(SOC_I2C_1_REGS) & I2C_INT_ADRR_READY_ACESS));

    flag = 1;
}

/*
** Receives data over I2C bus 
*/
static void SetupI2CReception(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_1_REGS, 1);

    numOfBytes = I2CDataCountGet(SOC_I2C_1_REGS);

    cleanupInterrupts();

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(tCount != numOfBytes);

    /* Wait untill I2C registers are ready to access */
    while(0 == (I2CMasterIntRawStatus(SOC_I2C_1_REGS) & I2C_INT_ADRR_READY_ACESS));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(SOC_I2C_1_REGS, dcount);

    numOfBytes = I2CDataCountGet(SOC_I2C_1_REGS);

    cleanupInterrupts();

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(SOC_I2C_1_REGS, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, I2C_INT_RECV_READY |
                                         I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_1_REGS);

    while(flag);

    flag = 1;
}

/* Configures AINTC to generate interrupt */
static void I2CAintcConfigure(void)
{
    /* Intialize the ARM Interrupt Controller(AINTC) */
    IntAINTCInit();

    /* Registering I2C0 ISR in AINTC */
    IntRegister(SYS_INT_I2C1INT, I2CIsr);
    
    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_I2C1INT);
}

static void cleanupInterrupts(void)
{
    I2CMasterIntEnableEx(SOC_I2C_1_REGS, 0x7FF);
    I2CMasterIntClearEx(SOC_I2C_1_REGS,  0x7FF);
    I2CMasterIntDisableEx(SOC_I2C_1_REGS, 0x7FF);
}


/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
void I2CIsr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(SOC_I2C_1_REGS);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(SOC_I2C_1_REGS,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));

    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
         dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_1_REGS);

         /* Clear receive ready interrupt status */
         I2CMasterIntClearEx(SOC_I2C_1_REGS,  I2C_INT_RECV_READY);

         if(rCount == numOfBytes)
         {
              /* Disable the receive ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_RECV_READY);

              /* Generate a STOP */
              I2CMasterStop(SOC_I2C_1_REGS);

         }

             
    }
    if (status & I2C_INT_TRANSMIT_READY)
    {
         /* Put data to data transmit register of i2c */
         I2CMasterDataPut(SOC_I2C_1_REGS, dataToSlave[tCount++]);

         /* Clear Transmit interrupt status */
         I2CMasterIntClearEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);

         if(tCount == numOfBytes)
         {
              /* Disable the transmit ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);

         }

    }
  
    if (status & I2C_INT_STOP_CONDITION)
    {
           /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
                           I2C_INT_STOP_CONDITION);
         flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY  |
                                               I2C_INT_RECV_READY      |
                                               I2C_INT_NO_ACK          |
                                               I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(SOC_I2C_1_REGS);

         flag = 0;
    }
}
/***************************** End Of File ************************************/
