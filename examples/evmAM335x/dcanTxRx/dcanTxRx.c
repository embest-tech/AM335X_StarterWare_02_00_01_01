/**
 * \file   dcanTxRx.c
 *
 * \brief  The application will transmit and receive single/multiple CAN
 *         frames and display that message on the UART console.
 *
 *         Application Configuration:
 *
 *             Modules Used:
 *                 DCAN1
 *                 UART0
 *
 *             Configurable Parameter:
 *                 1) CAN frame data: 1 - 8 bytes
 *
 *             Hard-coded configuration of other parameters:
 *                 1) CAN communication using two nodes
 *                 2) CAN frame support - Data frames
 *                 3) Configured to Transmit/Receive CAN frames
 *                 4) Operating Speed: 1Mbps
 *
 *         Application Use Case:
 *             The application performs transmission/reception of CAN frames
 *             between two CAN nodes. On execution of the example the user can
 *             either transmit/receive a single CAN frame or multiple CAN
 *             frames. If a single CAN frame is selected then user can transmit
 *             a CAN frame of data size ranging from 1-8 bytes. Upon selection
 *             of multiple CAN frames the application will transmit CAN frames
 *             intended by the user.
 *
 *         Running the example:
 *             Before execution of the example please connect the EVMs back to
 *             back using straight cables for DB9 connectors and on execution
 *             the user is asked if single/multiple data frames need to be
 *             transmitted/received. The CAN data frame is transmitted to
 *             other board and is again recived back by the sending board.
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

/* Include the necessary header files */
#include "consoleUtils.h"
#include "dcan_frame.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "hw_types.h"
#include "dcan.h"

/******************************************************************************
**                       INTERNAL MACRO DEFINITIONS
******************************************************************************/
#define CAN_DATA_BYTES_MAX_SIZE           (8u)
#define TRANSMIT_MULTIPLE_MSG             (2u)
#define TRANSMIT_REMOTE_FRAME             (3u)
#define DCAN_NO_INT_PENDING               (0x00000000u)
#define TRANSMIT_SINGLE_MSG               (1u)
#define DCAN_ERROR_OCCURED                (0x8000u)
#define ENTER_KEY_PRESSED                 (0xD)
#define CAN_RX_MSG_ID                     (0u)
#define DCAN_BIT_RATE                     (1000000u)
#define DCAN_IN_CLK                       (24000000u)
/* 
** This macro has to be commented if a CAN frame with extended ID 
** has to be transmitted.
*/
#define DCAN_STD_ID

/******************************************************************************
**                       INTERNAL FUNCTION PROTOTYPES                     
******************************************************************************/
static void MsgTransfer(unsigned int flag);
static void DCANAintcConfigure(void);
static void StartDCANTransfer(void);
static void ConfigureDCAN(void);
static void DCANParityIsr(void);
static void DCANIsr0(void);

/******************************************************************************
**                       GLOBAL VARIABLE DEFINITIONS                   
******************************************************************************/
/* CAN frame details */
#ifdef DCAN_STD_ID
static unsigned int txflag = (CAN_DATA_FRAME | CAN_MSG_DIR_TX);
static unsigned int canId = 0x01;
#else
static unsigned int txflag = (CAN_EXT_FRAME | CAN_DATA_FRAME | CAN_MSG_DIR_TX);
static unsigned int canId = 0x2000;
#endif
static unsigned int rxflag = (CAN_DATA_FRAME | CAN_MSG_DIR_RX);
static volatile unsigned int isrTxFlag = 1;
static volatile unsigned int isrRxFlag = 1;
static unsigned int index1 = 0;
static unsigned int index3 = 0;
static unsigned int value = 0;
static unsigned int data[2];
can_frame entry;

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
******************************************************************************/
int main(void)
{
    unsigned int index = 0;
    unsigned int input;

    /* Enable the DCAN1 module clock */
    DCANModuleClkConfig();

    /* Perform the pinmux for DCAN1 */
    DCANPinMuxSetUp(0);

    /* Initialize the DCAN message RAM */
    DCANMsgRAMInit(0);

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable the processor IRQ */
    IntMasterIRQEnable();

    /* Register the DCAN interrupts */
    DCANAintcConfigure();

    /* Perform the DCAN configuration */
    ConfigureDCAN();

    index = CAN_NUM_OF_MSG_OBJS;

    while(index--)
    {
        /* Invalidate all message objects in the message RAM */
        CANInValidateMsgObject(SOC_DCAN_1_REGS, index, DCAN_IF2_REG);
    }

    index = 1;

    while(index)
    {
        ConsoleUtilsPrintf("Please Input\n1) Transmit a single data frame\n");
        ConsoleUtilsPrintf("2) Transmit 'n' data frames\n\n");

        ConsoleUtilsPrintf("User Input = ");
        ConsoleUtilsScanf("%d", &input);
        ConsoleUtilsPrintf("\n\n");

        switch(input)
        {
            case 1:MsgTransfer(TRANSMIT_SINGLE_MSG);
                   index = 0;
                   break;
            case 2:MsgTransfer(TRANSMIT_MULTIPLE_MSG);
                   index = 0;
                   break;
            default:ConsoleUtilsPrintf("Invalid option selected!!\n");
                   break;
        }
    }

    /* Terminating while loop */
    while(1);
}

/*
** This function is used to send a single CAN message frame 
** or multiple messages.
*/
static void MsgTransfer(unsigned int flag)
{
    unsigned char *ptr;
    unsigned int index2 = 0, index4 = 0;
    unsigned int data1[10] = {0x30303030, 0x31313131, 0x32323232, 0x33333333, 
                              0x34343434, 0x35353535, 0x36363636, 0x37373737, 
                              0x38383838, 0x39393939};
    unsigned char buffer[9];

    ptr = (unsigned char *) &data;

    index3 = 1;

    entry.flag = rxflag;
    entry.id = CAN_RX_MSG_ID;

    /*
    ** Configure a receive message object to accept CAN
    ** frames with standard ID.
    */
    CANMsgObjectConfig(SOC_DCAN_1_REGS, &entry);

    entry.flag = (CAN_EXT_FRAME | CAN_MSG_DIR_RX | CAN_DATA_FRAME);
    entry.id = CAN_RX_MSG_ID;

    /*
    ** Configure a receive message object to accept CAN
    ** frames with extended ID.
    */
    CANMsgObjectConfig(SOC_DCAN_1_REGS, &entry);

    if(flag == TRANSMIT_SINGLE_MSG)
    {

        ConsoleUtilsPrintf("Please input data not exceeding 8 characters\n");

        ConsoleUtilsGets((char *) buffer, (CAN_DATA_BYTES_MAX_SIZE + 1));

        index1 = 0;
        while(buffer[index1] != '\0')
        {
            ptr[index1] = buffer[index1];
            index1++;
        }

        ConsoleUtilsPrintf("\r\n");

        /* Populate the can_frame structure with the CAN frame information */
        entry.id = canId;
        entry.flag = txflag;
        entry.dlc = index1;
        entry.data = (unsigned int*)data;

        value = 1;

        /* Configure a transmit message object */
        CANMsgObjectConfig(SOC_DCAN_1_REGS, &entry);

        /* Start the CAN transfer */
        StartDCANTransfer();
    }

    if(flag == TRANSMIT_MULTIPLE_MSG)
    {
        index1 = CAN_DATA_BYTES_MAX_SIZE;

        ConsoleUtilsPrintf("Please enter the number of messages to be");
        ConsoleUtilsPrintf(" transmitted:");
        ConsoleUtilsScanf("%d", &value);
        ConsoleUtilsPrintf("\n");

        entry.id = canId;
        entry.flag = txflag;
        entry.dlc = CAN_DATA_BYTES_MAX_SIZE;

        for(index2 = 0; index2 < value; index2++)
        {
            data[0] = data1[index2 % 10];
            data[1] = data1[index2 % 10];

            entry.data = (unsigned int*)data;

            isrTxFlag = 1; 
            isrRxFlag = 1; 

            /* Configure a transmit message object */
            CANMsgObjectConfig(SOC_DCAN_1_REGS, &entry);

            if(index4 == 0)
            {
                /* Start the CAN transfer */
                StartDCANTransfer();

                index4 = 1;
            }

            ConsoleUtilsPrintf("%d%c", (index2 + 1), ':');

            while(isrTxFlag || isrRxFlag);
 
            index3++;
        }
    }
}

/*
** This function will start the communication on the bus and enable
** the DCAN interrupt lines.
*/
static void StartDCANTransfer(void)
{
    /* Start the CAN transfer */
    DCANNormalModeSet(SOC_DCAN_1_REGS);

    /* Enable the error interrupts */
    DCANIntEnable(SOC_DCAN_1_REGS, DCAN_ERROR_INT);

    /* Enable the interrupt line 0 of DCAN module */
    DCANIntLineEnable(SOC_DCAN_1_REGS, DCAN_INT_LINE0);
}

/*
** This function will configure DCAN with the required parameters.
*/
static void ConfigureDCAN(void)
{
    /* Reset the DCAN module */
    DCANReset(SOC_DCAN_1_REGS);

    /* Enter the Initialization mode of CAN controller */
    DCANInitModeSet(SOC_DCAN_1_REGS);

    /* Enable the write access to the DCAN configuration registers */
    DCANConfigRegWriteAccessControl(SOC_DCAN_1_REGS, DCAN_CONF_REG_WR_ACCESS_ENABLE);

    /* Configure the bit timing values for CAN communication */
    CANSetBitTiming(SOC_DCAN_1_REGS, DCAN_IN_CLK, DCAN_BIT_RATE); 

    /* Disable the write access to the DCAN configuration registers */
    DCANConfigRegWriteAccessControl(SOC_DCAN_1_REGS, DCAN_CONF_REG_WR_ACCESS_DISABLE);
}

/*
** DCAN Parity error interrupt handler.
*/
static void DCANParityIsr(void)
{
    unsigned int errVal;
    unsigned int wrdNum;
    unsigned int msgNum;
    
    if(DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT) == 
                           DCAN_ERROR_OCCURED)
    {
        /* Check the status of DCAN Status and error register */
        errVal = DCANErrAndStatusRegInfoGet(SOC_DCAN_1_REGS);

        if(errVal & DCAN_PARITY_ERR_DETECTED)    
        {
            /* Read the word number where parity error got detected */
            wrdNum = DCANParityErrCdRegStatusGet(SOC_DCAN_1_REGS, 
                                                 DCAN_PARITY_ERR_WRD_NUM);

            /* Read the message number where parity error got detected */
            msgNum = DCANParityErrCdRegStatusGet(SOC_DCAN_1_REGS, 
                                                 DCAN_PARITY_ERR_MSG_NUM);

            ConsoleUtilsPrintf("\nParity error has occured in message number ");
            ConsoleUtilsPrintf("%d", msgNum);
            ConsoleUtilsPrintf(" and word number ");
            ConsoleUtilsPrintf("%d\n", wrdNum);
       }
    }
}

/*
** DCAN Isr for Interrupt line 0.
*/
static void DCANIsr0(void)
{
    unsigned int errVal;
    unsigned int data[2];
    unsigned char *dataPtr;
    unsigned int index = 0;
    unsigned int msgNum;

    while(DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT))
    {
        if(DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT) == 
                               DCAN_ERROR_OCCURED)
        {
            /* Check the status of DCAN Status and error register */
            errVal = DCANErrAndStatusRegInfoGet(SOC_DCAN_1_REGS);       

            if(errVal & DCAN_MOD_IN_BUS_OFF_STATE)
            {
                ConsoleUtilsPrintf("**DCAN is in Bus-off state**\n");

                /*
                ** This feature will automatically get the CAN bus to bus-on
                ** state once the error counters are below the error warning
                ** limit.
                */
                DCANAutoBusOnControl(SOC_DCAN_1_REGS, DCAN_AUTO_BUS_ON_ENABLE);
            }

            if(errVal & DCAN_ERR_WARN_STATE_RCHD)
            {
                ConsoleUtilsPrintf("Atleast one of the error counters have");
                ConsoleUtilsPrintf(" reached the error warning limit\n");
            }
        }
    
        if((DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT) != 
            DCAN_NO_INT_PENDING) && 
          ((DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT) != 
            DCAN_ERROR_OCCURED)))
        {
            /* Get the number of the message object which caused the interrupt */
            msgNum = DCANIntRegStatusGet(SOC_DCAN_1_REGS, DCAN_INT_LINE0_STAT);
    
            /* Interrupt handling for transmit objects */
            if(msgNum < (CAN_NUM_OF_MSG_OBJS/2))
            {
                /* Clear the Interrupt pending status */
                CANClrIntPndStat(SOC_DCAN_1_REGS, msgNum, DCAN_IF1_REG);

                isrTxFlag = 0;

                if(value == index3)
                {
                    /* Disable the transmit interrupt of the message object */
                    CANTxIntDisable(SOC_DCAN_1_REGS, msgNum, DCAN_IF1_REG);

                    /* Invalidate the transmit message object */
                    CANInValidateMsgObject(SOC_DCAN_1_REGS, msgNum, DCAN_IF1_REG);
                }
            }

            if((msgNum >= (CAN_NUM_OF_MSG_OBJS/2)) && (msgNum < CAN_NUM_OF_MSG_OBJS))
            {
                /* Read a received message from message RAM to interface register */
                CANReadMsgObjData(SOC_DCAN_1_REGS, msgNum, (unsigned int*) data, DCAN_IF2_REG);
    
                /* Clear the Interrupt pending status */
                CANClrIntPndStat(SOC_DCAN_1_REGS, msgNum, DCAN_IF2_REG);

                dataPtr = (unsigned char*) data;

                ConsoleUtilsPrintf("Data received = ");

                index1 = (DCANIFMsgCtlStatusGet(SOC_DCAN_1_REGS, DCAN_IF2_REG) &
                                                DCAN_DAT_LEN_CODE_READ);

                /* Print the received data bytes on the UART console */
                for(index = 0; index < index1; index++)
                {
                    ConsoleUtilsPrintf("%c", *dataPtr++);
                }

                ConsoleUtilsPutChar('\r');
                ConsoleUtilsPutChar('\n');

                isrRxFlag = 0;

                if(value == index3)
                {
                    /* Disable the receive interrupt of the message object */
                    CANRxIntDisable(SOC_DCAN_1_REGS, msgNum, DCAN_IF2_REG);

                    /* Invalidate the receive message object */
                    CANInValidateMsgObject(SOC_DCAN_1_REGS, msgNum, DCAN_IF2_REG);
                }
           }
       }
   }
}

/* Interrupt mapping to AINTC and registering CAN ISR */
static void DCANAintcConfigure(void)
{
    /* Set up the ARM interrupt controller */
    IntAINTCInit();

    /* Register the DCAN Interrupt handler for interrupt line 0*/
    IntRegister(SYS_INT_DCAN1_INT0, DCANIsr0);

    /* Assign priority to the interrupt */
    IntPrioritySet(SYS_INT_DCAN1_INT0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupts in AINTC */
    IntSystemEnable(SYS_INT_DCAN1_INT0);

    /* Register the DCAN Interrupt handler for parity interrupt */
    IntRegister(SYS_INT_DCAN1_PARITY, DCANParityIsr);

    /* Assign priority to the interrupt */
    IntPrioritySet(SYS_INT_DCAN1_PARITY, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupts in AINTC */
    IntSystemEnable(SYS_INT_DCAN1_PARITY);
}
