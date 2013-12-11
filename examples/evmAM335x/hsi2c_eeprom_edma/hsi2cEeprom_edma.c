/**
 * \file  i2cEdmaEEprom.c
 *
 * \brief Sample application for HSI2C using EDMA for data transfer.
 *        This application reads the specified number of bytes from
 *        EEPROM using HSI2C and displays them on the console of the
 *        host machine. The EDMA will be used for data transfer
 *        between I2C registers and DDR memory.
 *
 *        Application Configuration:
 *
 *            Modules Used:
 *                I2C0
 *                UART0
 *                Interrupt Controller
 *                EDMA
 *
 *            Configurable parameters:
 *                None.
 *
 *            Hard-coded configuration of other parameters:
 *                Bus frequency     - 100kHz
 *                Addressing mode   - 7bit
 *                I2C Instance      - 0
 *                Slave Address     - 0x50
 *                EEPROM memory address - 0x0000
 *                No of bytes to be read - 50
 *
 *        Application Use Case:
 *        1.I2C controller is configured in Master mode.
 *        2.Master will first send the address offset value to EEPROM, which
 *          indicates the address from which data read should start.
 *        3.Then master will read the data from EEPROM and display on console.
 *        4.The functionality is demonstrated in DMA mode.
 *
 *        Running the example:
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

#include "edma_event.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "consoleUtils.h"
#include "interrupt.h"
#include "hsi2c.h"
#include "edma.h"


/*********************************************************************************
**                    INTERNAL MACRO DEFINITION
*********************************************************************************/
/* Address of CAT24C256 E2PROM */
#define I2C_SLAVE_ADDR             (0x50u)

/* Higher byte address(i.eA8-A15) */
#define EPROM_ADDR0_MSB            (0x00u)

/* Lower byte address (i.e A0-A7) */
#define EPROM_ADDR0_LSB            (0x00u)

/*******************************************************************************
**                   INTERNAL FUNCTION PROTOTYPE
*******************************************************************************/
void I2CEdmaIsr(void);
void I2CEdmaErrIsr(void);
static void SetupI2C(void);
static void SetupEdma(void);
static void EpromRead(void);
static void CleanUpInterrupts(void);
static void SetupI2CReception(int dcount);
static void RegisterEdma3Interrupts(void);
static void I2CEdmaReceiveConfig(unsigned int address);
static void I2CEdmaTransmitConfig(unsigned int address); 

/*******************************************************************************
**                   INTERNAL VARIABLE DEFINITION
*******************************************************************************/
volatile int error = 0;
unsigned char destBuff[50];
volatile unsigned int flag = 1;
unsigned char srcBuff[2]  = {EPROM_ADDR0_MSB, EPROM_ADDR0_LSB};

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

int main(void)
{
    unsigned int i;
    unsigned char temp;

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    SetupEdma();

    /*
    ** Configures I2C to Master mode to generate start codition
    ** on I2C bus and to transmit data at a bus speed of  100khz
    */
    SetupI2C();

    EpromRead();

    for(i = 0; i < 50; i++)
    {
         /* Collecting the Most Significant Nibble of the data byte. */
         temp = ((destBuff[i] & 0xF0) >> 4);

         if(temp < 10)
         {
              ConsoleUtilsPrintf("%c", (temp + 0x30));
         }
         else
         {
              ConsoleUtilsPrintf("%c", (temp + 0x37));
         } 

         /* Collecting the Least Significant Nibble of the data byte. */
         temp = (destBuff[i] & 0x0F);

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

/* Read from Eprom */
static void EpromRead(void)
{
    I2CEdmaTransmitConfig((unsigned int)srcBuff);
    SetupI2CReception(50);
}

/*
** Configures Edma to transmit 1 byte from 
** i2c transmit register to destination buffer.
*/
static void I2CEdmaReceiveConfig(unsigned int address)
{
    EDMA3CCPaRAMEntry paramSet;
  
    paramSet.destAddr  = address;
    paramSet.srcAddr   = (SOC_I2C_0_REGS + I2C_DATA);

    /*
    ** I2C generates one EDMA event whenever I2CXSR is empty.There is space
    ** for only one byte of data in I2CXSR.There is no fifo.Hence per event
    ** one bytes needs to be transfered.Thus EDMA is configured in ASYNC mode
    ** with acount = 1, bcount = total_numbytes, ccount = 1.BSRC index should
    ** be 1 since memory pointer needs to incremented one after every byte 
    ** transfer by EDMA.BDST index should be zero since the destination address
    ** is in constant adrressing mode(hardware register).
    **
    */
    paramSet.srcBIdx    = 0x00;
    paramSet.srcCIdx    = 0x00;
    paramSet.destBIdx   = 0x01;
    paramSet.destCIdx   = 0x00;
    paramSet.aCnt       = 0x01;
    paramSet.bCnt       = 0x32;             
    paramSet.cCnt       = 0x01;
    paramSet.bCntReload = 0x00;
    paramSet.linkAddr   = 0xffff;
    paramSet.opt        = 0x00;

    /* Program the TCC */
    paramSet.opt |= ((EDMA3_CHA_I2C0_RX << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Transmission complition interrupt enable */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);
 
    /* configure PaRAM Set */ 
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS,  EDMA3_CHA_I2C0_RX, &paramSet);

    /* Enable the transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS,  EDMA3_CHA_I2C0_RX, EDMA3_TRIG_MODE_EVENT);
}


/*
** Enables Module Clock for EDMA Channel and Transfer Controller.
** Registers Edma Interrupt.
*/
static void SetupEdma(void)
{
    volatile unsigned int evtQ = 0;

    /* Enable the module clock for EDMA */
    EDMAModuleClkConfig();

    /* Intialize the Edma */
    EDMA3Init(SOC_EDMA30CC_0_REGS, evtQ);

    /* Register required edma interrupts */
    RegisterEdma3Interrupts();

    /* Request DMA Channel and TCC */ 
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA ,
                        EDMA3_CHA_I2C0_RX , EDMA3_CHA_I2C0_RX , evtQ);

    /* Request DMA Channel and TCC */ 
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA ,
                        EDMA3_CHA_I2C0_TX , EDMA3_CHA_I2C0_TX , evtQ);
}

static void I2CEdmaTransmitConfig(unsigned int address) 
{
    EDMA3CCPaRAMEntry paramSet;
    paramSet.srcAddr    = (unsigned int)address;
    paramSet.destAddr   = (SOC_I2C_0_REGS + I2C_DATA);
    paramSet.srcBIdx    = 0x01;
    paramSet.srcCIdx    = 0x00;
    paramSet.destBIdx   = 0x00;
    paramSet.destCIdx   = 0x00;
    paramSet.aCnt       = 0x01;
    paramSet.bCnt       = 0x02;              
    paramSet.cCnt       = 0x01;
    paramSet.bCntReload = 0x00;
    paramSet.linkAddr   = 0xffff;
    paramSet.opt        = 0;

    /* Program the TCC */
    paramSet.opt |= (( EDMA3_CHA_I2C0_TX << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Transmission complition interrupt enable */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* configure PaRAM Set */ 
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_I2C0_TX , &paramSet);

    /* Enable the transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_I2C0_TX , EDMA3_TRIG_MODE_EVENT);
}

/*
** Registers EDMA Completion and Error Interrupt
**
*/
static void RegisterEdma3Interrupts(void)
{
    /* Intialize ARM interrupt controller */
    IntAINTCInit();

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /*
    ** Registers the I2CIsr in the Interrupt Vector Table of
    ** AINTC.
    */
    IntRegister(SYS_INT_EDMACOMPINT , I2CEdmaIsr);

     /* Set priority for system interrupt in AINTC */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the EDMA CC0 system interrupt in AINTC.*/
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /*
    ** Registers the I2CEdmaIsr in the Interrupt Vector Table of
    ** AINTC.
    */
    IntRegister(SYS_INT_EDMAERRINT, I2CEdmaErrIsr);

    /* Set priority for system interrupt in AINTC */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the EDMA CCERR system interrupt AINTC.*/
    IntSystemEnable(SYS_INT_EDMAERRINT);
}

/*
** Clears the pending interrupt in EDMA,Disables the Transmit
** Event generated by I2C and Disables EDMA Transfer
*/
void I2CEdmaIsr(void)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int val;


    /* Get status of the interrupt */
    pendingIrqs = EDMA3IntrStatusHighGet(SOC_EDMA30CC_0_REGS);

    val = (0x01 << (EDMA3_CHA_I2C0_TX - 32));

    if((pendingIrqs & val))
    {
         /* clear the pending interrupt */
         EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, EDMA3_CHA_I2C0_TX);

         /* Disable the transmit event */
         I2CDMATxEventDisable(SOC_I2C_0_REGS);

         /* Disable Edma Transfer */
         EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , EDMA3_CHA_I2C0_TX , EDMA3_TRIG_MODE_EVENT);
   
         flag = 0;
    }

    val = (0x01 << (EDMA3_CHA_I2C0_RX - 32));
    
    if((pendingIrqs & val))
    {
         /* clear the pending interrupt */
         EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, EDMA3_CHA_I2C0_RX);

         /* Disable the transmit event */
         I2CDMARxEventDisable(SOC_I2C_0_REGS);

         /* Disable Edma Transfer */
         EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , EDMA3_CHA_I2C0_RX , EDMA3_TRIG_MODE_EVENT);
   
         flag = 0;
    }
}

/*
** Clears the pending error interrupt in EDMA,Disables the Transmit
** Event generated by I2C and Disables EDMA Transfer
*/
void I2CEdmaErrIsr(void)
{
    volatile  unsigned int pendingIrqs;

    /* Get status of the error interrupt */
    pendingIrqs = EDMA3ErrIntrHighStatusGet(SOC_EDMA30CC_0_REGS);

    if((pendingIrqs & (0x01 << (EDMA3_CHA_I2C0_TX  - 32))))
    {
         /* clear the pending error interrupt */
         EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS,  EDMA3_CHA_I2C0_TX);  
    
         /* Disable the transmit event */
         I2CDMATxEventDisable(SOC_I2C_0_REGS);

         /* Disable Edma Transfer */
         EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , EDMA3_CHA_I2C0_TX ,
                             EDMA3_TRIG_MODE_EVENT);

         flag = 0;

         error = -1;
    }
    else if((pendingIrqs & (0x01 << (EDMA3_CHA_I2C0_RX - 32))))
    {
         /* clear the pending error interrupt */
         EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS,  EDMA3_CHA_I2C0_RX);  
    
         /* Disable the transmit event */
         I2CDMARxEventDisable(SOC_I2C_0_REGS);

         /* Disable Edma Transfer */
         EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , EDMA3_CHA_I2C0_RX ,
                             EDMA3_TRIG_MODE_EVENT);

         flag = 0;

         error = -1;
     }
}

/*
** Configures I2C to communicate with EEPROM
** at 100kbps
*/
static void SetupI2C(void)
{
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(0);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_0_REGS);
   
    /* Disables the auto idle functionality */
    I2CAutoIdleDisable(SOC_I2C_0_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_0_REGS, 48000000, 12000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, I2C_SLAVE_ADDR);

    /* Bring i2c out of reset */
    I2CMasterEnable(SOC_I2C_0_REGS);
}

/*
** Reads data from selected address 
*/
static void SetupI2CReception(int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_0_REGS, 2);

    /* clear all interrupt status */
    CleanUpInterrupts();

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);

    /*I2C Transmit Event is enabled */ 
    I2CDMATxEventEnable(SOC_I2C_0_REGS);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);

     /*
     ** Wait for the START to be reflected on the bus.
     ** This can be checked by waiting for BUS BUSY condition set.
     */
    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(flag);

    flag = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(SOC_I2C_0_REGS) & (I2C_INT_ADRR_READY_ACESS)));

    /* Configure the paramset for reception */
    I2CEdmaReceiveConfig((unsigned int)destBuff);

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(SOC_I2C_0_REGS, dcount);

    /* Clear all interrupt status */ 
    CleanUpInterrupts();

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_RX);

    /* I2C Receive Event is Enable */
    I2CDMARxEventEnable(SOC_I2C_0_REGS);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);
    
    /*
    ** Wait for the START to be reflected on the bus.
    ** This can be checked by waiting for BUS BUSY condition set.
    */
    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(flag);

    flag = 1;

    /* Generate stop condition */
    I2CMasterStop(SOC_I2C_0_REGS);

}

/* Clear status of all interrupts */
static void CleanUpInterrupts(void)
{
    I2CMasterIntEnableEx(SOC_I2C_0_REGS, 0x7FF);
    I2CMasterIntClearEx(SOC_I2C_0_REGS,  0x7FF);
    I2CMasterIntDisableEx(SOC_I2C_0_REGS, 0x7FF);
}

/******************************* End of file ********************************/
