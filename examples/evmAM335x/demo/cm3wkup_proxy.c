/**
 * \file     cm3wkup_proxy.c
 *
 * \brief    This file contains the function prototypes for inter-processor 
			 communication between A8 and M3.
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
 
#include <string.h>
#include "hw_types.h"
#include "hw_prm_mpu.h"
#include "hw_prm_per.h"
#include "cm3wkup_proxy.h"
#include "hw_control_AM335x.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "cm3image.h"
#include "hw_prm_wkup.h"
#include "consoleUtils.h"
#include "clock.h"
#include "mailbox.h"
#include "demoCfg.h"

/********************** MACROS ***************************/

#define PM_STATUS_SHIFT			16

/* Number of CLK_M_OSC clocks to be seen before exiting deep sleep mode */
#define  PM_DEEP_SLEEP_COUNT                    (0x6A75)

/******************************************************************************
**              EXTERNAL FUNCTIONS
******************************************************************************/
extern void romRestoreLocation(void);

/******************************************************************************
**                       GLOBAL VARIABLE DEFINITIONS
******************************************************************************/

/*  Flag to indicate M3 event is received   */
volatile unsigned int isM3IntReceived = 0;

/* DS0 data - don't cares are not defined */
deepSleepData ds0Data = {
    .dsDataBits.cmdID = PM_CMD_DS0_MODE,

    .dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
    .dsDataBits.moscState = PM_MOSC_STATE_OFF,
    .dsDataBits.deepSleepCount = PM_DEEP_SLEEP_COUNT,

    /* Value is reserved */
    .dsDataBits.vddMpuVal = 0,

    .dsDataBits.pdMpuState = PM_MPU_POWERSTATE_OFF,
    .dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
    .dsDataBits.pdMpul1RetState = PM_MPU_L1_RETSTATE_OFF,
    .dsDataBits.pdMpul2RetState = PM_MPU_L2_RETSTATE_OFF,
    .dsDataBits.pdMpuRamOnState = PM_MPU_RAM_ONSTATE_OFF,

    .dsDataBits.pdPerState = PM_PER_POWERSTATE_RET,
    .dsDataBits.pdPerIcssMemRetState = PM_PER_ICSS_RAM_RETSTATE_OFF,
    .dsDataBits.pdPerMemRetState = PM_PER_MEM_RETSTATE_OFF,
    .dsDataBits.pdPerOcmcRetState = PM_PER_OCMC_RAM_RETSTATE_RET,
    .dsDataBits.pdPerIcssMemOnState = PM_PER_ICSS_RAM_ONSTATE_OFF,
    .dsDataBits.pdPerMemOnState = PM_PER_MEM_ONSTATE_OFF,
    .dsDataBits.pdPerOcmcOnState = PM_PER_OCMC_RAM_ONSTATE_OFF,

    .dsDataBits.wakeSources = WAKE_SOURCE_TSC,
    .dsDataBits.reserved = 0,
};

/*      DS1 data - don't cares are not defined  */
deepSleepData ds1Data = {
    .dsDataBits.cmdID = PM_CMD_DS1_MODE,

    .dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
    .dsDataBits.moscState = PM_MOSC_STATE_OFF,
    .dsDataBits.deepSleepCount = PM_DEEP_SLEEP_COUNT,

    /* Value is reserved */
    .dsDataBits.vddMpuVal = 0,

    .dsDataBits.pdMpuState = PM_MPU_POWERSTATE_OFF,
    .dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
    .dsDataBits.pdMpul1RetState = PM_MPU_L1_RETSTATE_OFF,
    .dsDataBits.pdMpul2RetState = PM_MPU_L2_RETSTATE_OFF,
    .dsDataBits.pdMpuRamOnState = PM_MPU_RAM_ONSTATE_OFF,

    .dsDataBits.pdPerState = PM_PER_POWERSTATE_ON,
    .dsDataBits.pdPerIcssMemRetState = PM_PER_ICSS_RAM_RETSTATE_OFF,
    .dsDataBits.pdPerMemRetState = PM_PER_MEM_RETSTATE_OFF,
    .dsDataBits.pdPerOcmcRetState = PM_PER_OCMC_RAM_RETSTATE_OFF,
    .dsDataBits.pdPerIcssMemOnState = PM_PER_ICSS_RAM_ONSTATE_ON,
    .dsDataBits.pdPerMemOnState = PM_PER_MEM_ONSTATE_ON,
    .dsDataBits.pdPerOcmcOnState = PM_PER_OCMC_RAM_ONSTATE_ON,

    .dsDataBits.wakeSources = WAKE_SOURCE_TSC,
    .dsDataBits.reserved = 0,
};

/* Standby data - don't cares are not defined */
deepSleepData standbyData = {
    .dsDataBits.cmdID = PM_CMD_STANDBY_MODE,
    .dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
    .dsDataBits.moscState = PM_MOSC_STATE_ON,
    .dsDataBits.deepSleepCount = PM_DEEP_SLEEP_COUNT,

    /* Value is reserved */
    .dsDataBits.vddMpuVal = 0,

    .dsDataBits.pdMpuState = PM_MPU_POWERSTATE_OFF,
    .dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
    .dsDataBits.pdMpul1RetState = PM_MPU_L1_RETSTATE_OFF,
    .dsDataBits.pdMpul2RetState = PM_MPU_L2_RETSTATE_OFF,
    .dsDataBits.pdMpuRamOnState = PM_MPU_RAM_ONSTATE_OFF,

    .dsDataBits.pdPerState = PM_PER_POWERSTATE_ON,
    .dsDataBits.pdPerIcssMemRetState = PM_PER_ICSS_RAM_RETSTATE_OFF,
    .dsDataBits.pdPerMemRetState = PM_PER_MEM_RETSTATE_OFF,
    .dsDataBits.pdPerOcmcRetState = PM_PER_OCMC_RAM_RETSTATE_OFF,
    .dsDataBits.pdPerIcssMemOnState = PM_PER_ICSS_RAM_ONSTATE_ON,
    .dsDataBits.pdPerMemOnState = PM_PER_MEM_ONSTATE_ON,
    .dsDataBits.pdPerOcmcOnState = PM_PER_OCMC_RAM_ONSTATE_ON,

    .dsDataBits.wakeSources = WAKE_SOURCE_MPU,
    .dsDataBits.reserved = 0,
};

/*      Command to reset CM3 state machine      */
deepSleepData dsDataM3reset = {
    .dsDataBits.cmdID = 0xE
};

/**************************************************************************
  API FUNCTION DEFINITIONS
***************************************************************************/


/**
 *  \brief   This function configures the deep sleep data in to IPC registers
 *
 * \param     pmDsDataVar	structure variable containing deep sleep data
 *
 * \return 	  None
 */
  
void configIPCRegs(deepSleepData pmDsDataVar)
{

	/*	Command ID	*/
	HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(1)) = pmDsDataVar.dsParams.short1;
		
	if((PM_CMD_RTC_MODE == pmDsDataVar.dsDataBits.cmdID) || 
			(PM_CMD_RTC_FAST_MODE == pmDsDataVar.dsDataBits.cmdID))
	{
		/*	RTC time out value	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(2)) = pmDsDataVar.dsParams.byte1;
	}
	else
	{
		/*	Resume address	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(0)) = pmDsDataVar.dsParams.word0;
		/*	deep sleep data	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(2)) = pmDsDataVar.dsParams.word1;
		/*	deep sleep data	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(3)) = pmDsDataVar.dsParams.word2;	
	}
}


/**
 *  \brief   This function reads teh trace data from CM3
 *
 * \param     None
 *
 * \return 	  trace		trace data indicating the state of CM3
 */
 
unsigned int readCM3Trace(void)
{
	return HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(4));
}


/**
 *  \brief   This function returns the status of the last sent command
 *
 * \param     None
 *
 * \return 	  status 	status of the last sent command
 */
 
unsigned short readCmdStatus(void)
{
	return ((HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(1))) >> PM_STATUS_SHIFT);
}

/**
 * \brief   This function extracts the firmware version from IPC message register
 *
 * \param   None
 *
 * \return  CM3 firmware version
 */
unsigned short readCM3FWVersion(void)
{
    return (HWREGH(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(2)));
}

/*
** Clear CM3 event and re-enable the event
*/
void CM3EventsClear(void)
{
    /* Clear M3_TXEV event */
    HWREG(SOC_CONTROL_REGS + CONTROL_M3_TXEV_EOI) |=
                                    CONTROL_M3_TXEV_EOI_M3_TXEV_EOI;

    /* Re-arm M3_TXEV event */
    HWREG(SOC_CONTROL_REGS + CONTROL_M3_TXEV_EOI) &=
                                 (~CONTROL_M3_TXEV_EOI_M3_TXEV_EOI);
}

/*
** CM3 ISR handler
*/
static void CM3Isr(void)
{
    isM3IntReceived = 1;
    CM3EventsClear();
}

/*
** Register interrupt handler for CM3 Event
*/
void CM3IntRegister(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_M3_TXEV, CM3Isr);
}

/*
** Load CM3 image into its memory and release CM3 from reset
*/
void CM3LoadAndRun(void)
{
    /* Load CM3 SW */
    memcpy((void *)(CM3_UMEM_START_ADDR), (const void *)cm3image,
           sizeof(cm3image));

    /* Release CM3 from reset */
    HWREG(SOC_PRM_WKUP_REGS + PRM_WKUP_RM_WKUP_RSTCTRL) &=
          (~PRM_WKUP_RM_WKUP_RSTCTRL_WKUP_M3_LRST);
}

/*
** Wait for ACK from CM3
*/
void waitForM3Txevent(void)
{
    /* wait until CM3 TX Event is generated */
    while(!isM3IntReceived);

    while(PM_IN_PROGRESS == readCmdStatus())
    {
        CM3EventsClear();
    }

    switch(readCmdStatus())
    {
       case PM_CMD_PASS:
       case PM_WAIT4OK:
           break;

       case PM_CMD_FAIL:
       default:

           /* Command failed or invalid status */
           ConsoleUtilsPrintf("\n\n ACK Failed \r\n");

           while(1);
    }

    CM3EventsClear();

    /* Reset interrupt flag */
    isM3IntReceived = 0;
}

/*
** Initialize the Mailbox
**  - Enable clock and reset mailbox
*/
void initializeMailbox(unsigned int baseAdd)
{
    /* Enable Mailbox clock */
    enableModuleClock(CLK_MAILBOX0);

    /* Reset Mailbox */
    MBresetMailbox(baseAdd);

    /* Clear new message status */
    MBclrNewMsgStatus(baseAdd, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);

    /* Enable new message interrupt */
    MBenableNewMsgInt(baseAdd, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);

    /* Configure idle mode */
    MBconfigIdleMode(MAILBOX_0_BASE_ADDR,
                     (MAILBOX_SYSCONFIG_SIDLEMODE_FORCEIDLE <<
                                            MAILBOX_SYSCONFIG_SIDLEMODE_SHIFT));
}

/*
** Initializes Mailbox
*/
void MailBoxInit(void)
{
    initializeMailbox(MAILBOX_0_BASE_ADDR);
}

/*
** Generate Mailbox interrupt to CM3 by writing a dummy vlaue to mailbox reg
*/
void generateMailboxInt(unsigned int baseAdd)
{
    /* Write to Mailbox register */
    MBsendMessage(baseAdd, MAILBOX_QUEUE_0, 0x12345678u);
}

/*
** Clear mail box messages
*/
void clearMailboxMsg(void)
{
    unsigned int tamp;

    /* Read the message back */
    MBgetMessage(MAILBOX_0_BASE_ADDR, MAILBOX_QUEUE_0, &tamp);

    /* Clear new message status */
    MBclrNewMsgStatus(MAILBOX_0_BASE_ADDR, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);
}

/*
** MPU and CM3 Sync
*/
void syncCm3(void)
{
    /* Generate mailbox interrupt to CM3 */
    generateMailboxInt(MAILBOX_0_BASE_ADDR);

    /* Wait for ACK from CM3 */
    waitForM3Txevent();

    /* Clear the message in mailbox */
    clearMailboxMsg();
}
