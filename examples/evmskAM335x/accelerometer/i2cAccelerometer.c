/**
 * \file   i2cAccelerometer.c
 *
 * \brief  This application demonstrates invoking APIs of HSI2C to control
 *         Accelerometer. An accelerometer is an electromechanical device
 *         that will measure acceleration forces. This application measures
 *         the angle of tilt of the board with respect to earth and displays
 *         on the console.
 *
 *         Application Configuration:
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
 *                 Slave Address   - 0x18
 *
 *         Application UseCase:
 *         1. I2C controller will be configured in master mode.
 *         2. Master will configure the accelerometer(part no: LIS331DLH) by
 *            writing configuration values to accelerometer's registers.
 *         3. Master will read the angle of tilt from accelerometer
 *            using I2C interface and displays on the console.
 *
 *         Running the example:
 *         1. On executing the application, as the EVM board is tilted
 *            its angle of tilt is displayed on the the console.
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
#include "evmskAM335x.h"
#include "delay.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define  I2C_INSTANCE                  SOC_I2C_0_REGS

#define  I2C_SYSTEM_CLOCK              48000000

#define  I2C_INTERNAL_CLOCK            12000000

#define  I2C_OUTPUT_CLOCK              100000

#define  I2C_CLEAR_ALL_INTERRUPTS      0x7FF 

/* I2C address of LIS331DLH accelerometer*/
#define  I2C_SLAVE_ADDR               (0x18)
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

#define  ENABLE_XYZ_AXIS              (0x07)
#define  OUTPUT_DAT_RATE              (0x08)
#define  NORMAL_POWER_MODE            (0x20)
#define  FULL_SCALE_2g                (0x00)
#define  BLOCK_DATA_UPDATE            (0x80)

#define  CONFIG_CNTL_REG1             (ENABLE_XYZ_AXIS       |  \
                                       OUTPUT_DAT_RATE       |   \
                                       NORMAL_POWER_MODE)

#define  CONFIG_CNTL_REG4             (FULL_SCALE_2g    |    \
                                       BLOCK_DATA_UPDATE)   

#define  INTERNAL_FILTER_BYPASS       (0x00) 

#define  DISABLE_SLEEP_TO_WAKEUP      (0x00)  

#define  START_ADDR_DDR               (0x80000000)

#define  START_ADDR_DEV               (0x44000000)

#define  START_ADDR_OCMC              (0x40300000)

#define  NUM_SECTIONS_DDR             (512)

#define  NUM_SECTIONS_DEV             (960)

#define  NUM_SECTIONS_OCMC            (1)

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void I2CIsr(void);
static void SetupI2C(void);
static void I2CAintcConfigure(void);
static void cleanupInterrupts(void);
static void SetupI2CTransmit(unsigned int dcount);
static void SetupI2CReception(unsigned int dcount);
static void AccelerometerRegRead(unsigned char regOffset, unsigned char* data);
static void AccelerometerRegWrite(unsigned char regOffset, unsigned char data);
static int  BinarySearch(unsigned int, unsigned int);
/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
volatile unsigned char dataFromSlave[2];
volatile unsigned char dataToSlave[3];
volatile unsigned int gpioIsrFlag = 0;
volatile unsigned int numOfBytes;
volatile unsigned int flag = 1;
volatile unsigned int tCount;
volatile unsigned int rCount;
unsigned char x_low;
unsigned char y_low;
unsigned char z_low;
unsigned char x_high;
unsigned char y_high;
unsigned char z_high;

 /* This array is used as cosine lookup table.
 ** Each index of this array represnts the degree(0 to 90). The value at 
 ** the corresponding index represents the cosine value of the degree.
 */
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


/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif
/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

/*
** Function to setup MMU. This function Maps three regions ( 1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes 
    ** specified here are, 
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given. 
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

int main()
{
    int x_axis;
    int y_axis;
    int z_axis;
    int Index;
    int x_direction = 1, y_direction = 1, z_direction = 1;
    char c;

    MMUConfigAndEnable();

    CacheEnable(CACHE_ALL);

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Configures AINTC to generate interrupt */
    I2CAintcConfigure();

    DelayTimerSetup();

    /* Enable IRQ in CPSR */
    IntMasterIRQEnable();

    /*
    ** Configures HSI2C to Master mode to generate start
    ** condition on HSI2C bus and to transmit data at a
    ** speed of 100khz
    */
    SetupI2C();

    /*
    ** Configure Accelerometer in normal mode with ODR
    ** (output data rate) of 100hz and enable x,y and
    ** z axis
    */
    AccelerometerRegWrite(CTRL_REG1, CONFIG_CNTL_REG1);

    /* Internal filter is bypassed */
    AccelerometerRegWrite(CTRL_REG2, INTERNAL_FILTER_BYPASS);

    /*
    ** Enable Block data update,littel
    ** endian selected and full scale
    ** is configured to 2g.
    */
    AccelerometerRegWrite(CTRL_REG4, CONFIG_CNTL_REG4);

    /* sleep to wakeup function is disabled */
    AccelerometerRegWrite(CTRL_REG5, DISABLE_SLEEP_TO_WAKEUP);

    /* power on delay */
    delay(25);

    do
    {
         /* Read the accelerations in x,y and z directions */
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

        
         /* 
         **If condition is true, then acceleration is along negative x-axis.
         **Else it is along positive x-axis.
         */
         if(x_axis & 0x800)
         {
              x_axis = ~x_axis;
              x_axis =  x_axis + 1;
              x_axis =  x_axis & 0xfff;
              x_direction = -1;
         }
         
         /* 
         **If condition is true, then acceleration is along negative y-axis.
         **Else it is along positive y-axis.
         */
         if(y_axis & 0x800)
         {
              y_axis = ~y_axis;
              y_axis =  y_axis + 1;
              y_axis =  y_axis & 0xfff;
              y_direction = -1;
         }
      

         /* 
         **If condition is true, then acceleration is along negative z-axis.
         **Else it is along positive z-axis.
         */
         if(z_axis & 0x800)
         {
              z_axis = ~z_axis;
              z_axis = z_axis + 1;
              z_axis = z_axis & 0xfff;
              z_direction = -1;
         } 

         /* 
         **cosine-inverse(z_axis/g) gives the degree by which
         **the device is tilted. 1g is eqvivalent to 1024 digital
         **value.
         */
         z_axis = (z_axis * 100000) / 1024; 

         /*
         **(z_axis/g) results in a fraction. The function 'BinarySearch()'
         **searches for the fraction in the cosine lookup table. The return
         **value is the index where fraction is found.This is nothing
         **but the degree by which device is tilted i.e (cosine-inverse(z_axis/g).
         */
         Index = BinarySearch(z_axis, 90);

         if( Index != 0)
         {
   
              if( y_axis > x_axis)
              {
                   if(y_direction == -1)
                   {
                        y_direction = 1;
                        ConsoleUtilsPrintf("Tilted Left ");
                   }
                   else
                   {
                         ConsoleUtilsPrintf("Tilted Right ");
                   }
              }
              else
              {
                    if(x_direction == -1)
                    {
                          ConsoleUtilsPrintf("Tilted Backward ");
                    }
                    else
                    {
                          ConsoleUtilsPrintf("Tilted forward ");
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

        ConsoleUtilsPrintf("%d degree \n", Index);

        ConsoleUtilsPrintf("To proceed for one more iteration enter 'y':");

        ConsoleUtilsScanf("%c", &c);

        ConsoleUtilsPrintf("\n");

   }while(c == 'y');

   while(1);
}

/*
**The function 'BinarySearch()' searches for the fraction in the cosine lookup table.
**The return value is the index where fraction is found.This is nothing
** but the degree by which device is tilted i.e (cosine-inverse(z_axis/g).
*/
static int BinarySearch(unsigned int z, unsigned int n)
{
    int low, high, mid = 0;

    low = 0;
    high = n - 1;

    while(low <= high)
    {
         mid = (low + high) / 2;

         if(z < cosine[mid])
         {
              low = mid + 1;
         }
         else if(z > cosine[mid])
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

static void AccelerometerRegWrite(unsigned char regOffset, unsigned char data)
{
    dataToSlave[0] = regOffset;
    dataToSlave[1] = data;

    tCount = 0;
    SetupI2CTransmit(2);
}

static void AccelerometerRegRead(unsigned char regOffset, unsigned char* data)
{
    tCount = 0;
    rCount = 0;

    dataToSlave[0] = regOffset;
        
    SetupI2CReception(1);

    *data = dataFromSlave[0];
} 


 /*
 ** Configures i2c bus frequency and slave address.
 ** It also enable the clock for i2c module and
 ** does pinmuxing for the i2c.
 */
static void SetupI2C(void)
{
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(0);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(I2C_INSTANCE);

    /* Auto Idle functionality is dsabled */
    I2CAutoIdleDisable(I2C_INSTANCE);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(I2C_INSTANCE, I2C_SYSTEM_CLOCK, I2C_INTERNAL_CLOCK,
                        I2C_OUTPUT_CLOCK);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(I2C_INSTANCE, I2C_SLAVE_ADDR);

    /* Bring I2C module out of reset */
    I2CMasterEnable(I2C_INSTANCE);
}

/*
** Transmits data over I2C bus 
*/
static void SetupI2CTransmit(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(I2C_INSTANCE, dcount);

    numOfBytes = I2CDataCountGet(I2C_INSTANCE);

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(I2C_INSTANCE, I2C_CFG_MST_TX | I2C_CFG_STOP);

    /* Transmit and Stop condition interrupt is enabled */
    I2CMasterIntEnableEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY |
                                         I2C_INT_STOP_CONDITION );

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(I2C_INSTANCE);

    while(flag);

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(I2C_INSTANCE) & (I2C_INT_ADRR_READY_ACESS)));

    flag = 1;
}

/*
** Receives data over I2C bus 
*/
static void SetupI2CReception(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(I2C_INSTANCE, 0x01);

    numOfBytes = I2CDataCountGet(I2C_INSTANCE);

    /* Clear status all interrupts */
    cleanupInterrupts();

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(I2C_INSTANCE, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(I2C_INSTANCE);

    while(I2CMasterBusBusy(I2C_INSTANCE) == 0);

    while(tCount != numOfBytes);

    flag = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(I2C_INSTANCE) & (I2C_INT_ADRR_READY_ACESS)));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(I2C_INSTANCE, dcount);

    numOfBytes = I2CDataCountGet(I2C_INSTANCE);

    cleanupInterrupts();

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(I2C_INSTANCE, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(I2C_INSTANCE,  I2C_INT_RECV_READY |
                                          I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(I2C_INSTANCE);

    while(I2CMasterBusBusy(I2C_INSTANCE) == 0);

    while(flag);

    flag = 1;
}

/* Configures AINTC to generate interrupt */
void I2CAintcConfigure(void)
{
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_I2C0INT, I2CIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_I2C0INT);
}


void cleanupInterrupts(void)
{
    I2CMasterIntClearEx(I2C_INSTANCE, I2C_CLEAR_ALL_INTERRUPTS);
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
void I2CIsr(void)
{
    unsigned int status = 0;

    status = I2CMasterIntStatus(I2C_INSTANCE);

    I2CMasterIntClearEx(I2C_INSTANCE,
	                    (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
						
    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
         dataFromSlave[rCount++] = I2CMasterDataGet(I2C_INSTANCE);
	 I2CMasterIntClearEx(I2C_INSTANCE,  I2C_INT_RECV_READY);
		
         if(rCount == numOfBytes)
         {
              I2CMasterIntDisableEx(I2C_INSTANCE, I2C_INT_RECV_READY);
              /* Generate a STOP */
              I2CMasterStop(I2C_INSTANCE);
			  
         }

             
    }
    if (status & I2C_INT_TRANSMIT_READY)
    {
         /* Put data to data transmit register of i2c */
		 
        I2CMasterDataPut(I2C_INSTANCE, dataToSlave[tCount++]);
	I2CMasterIntClearEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY);		 
						
         if(tCount == numOfBytes)
         {
	      I2CMasterIntDisableEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY);
			
         }

    }
  
        
    if (status & I2C_INT_STOP_CONDITION)
    {
      	 /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
			                       I2C_INT_STOP_CONDITION);
         flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(I2C_INSTANCE, I2C_INT_TRANSMIT_READY  |
                                               I2C_INT_RECV_READY      |
                                               I2C_INT_NO_ACK          |
                                               I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(I2C_INSTANCE);

         flag = 0;
    }
}

