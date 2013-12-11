/**
 * \file    ehrpwm_haptics.c
 *
 * \brief   This is a sample application file demonstrating the use of
 *          a EHRPWM to rotate the haptics motor.
 *
 *          Application Configuration:
 *
 *              Modules Used:
 *                  PWMSS2
 *                  Interrupt Controller
 *
 *              Configurable parameters:
 *                  None.
 *
 *              Hard-coded configuration of other parameters:
 *                  Instance - 2
 *                  Time base counter mode - UP
 *                  Duty Cycle - 50%
 *                  Synchronization - disabled
 *                  Dead-band       - disabled
 *                  PWM-Chopper     - disabled
 *                  Trip-zone       - disabled
 *                  High Resolution PWM - disabled
 *
 *            Application Usecase:
 *            1. Hapticis device "Pico-Haptics 304-100" is connected on the
 *               board.
 *            2. PWM output will be generated with 50% duty cycle and with
 *               a frequency of 19.5 KHz.
 *            3. Haptics motor will be driven by this PWM output.
 *
 *            Running the example:
 *            1. Load the example application on the target and run.
 *            2. When the application is executed haptics motor will rotate.
 *
 *            Limitations:
 *            1. This application demonstrates generation of PWM B-Channel
 *               output only, because haptics motor is connected to PWMB output
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
#include "hw_types.h"
#include "ehrpwm.h"
#include "evmAM335x.h"
#include "uartStdio.h"


#define CLOCK_DIV_VAL                 (10)
#define SOC_EHRPWM_2_MODULE_FREQ      (100000000)

/****************************************************************************/
/*              LOCAL FUNCTION PROTOTYPES                                   */
/****************************************************************************/
static void EHRPWMConfigure(void);
/****************************************************************************/
/*             LOCAL FUNCTION DEFINITIONS                                   */
/****************************************************************************/

int main(void)
{
    PWMSSModuleClkConfig(2);

    EPWM2PinMuxSetup();

    /* Enable Clock for EHRPWM in PWM sub system */
    EHRPWMClockEnable(SOC_PWMSS2_REGS);

    /* Enable Timer Base Module Clock in control module */
    PWMSSTBClkEnable(2);

    /* EHRPWM is configured to generate PWM waveform on EPWMBxB Pin*/
    EHRPWMConfigure();

    while(1);
}

static void EHRPWMConfigure(void)
{
  
    /* TimeBase configuration */
    /* Configure the clock frequency */
    EHRPWMTimebaseClkConfig(SOC_EPWM_2_REGS,
                            SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL,
                            SOC_EHRPWM_2_MODULE_FREQ);

    /* Configure the period of the output waveform */
    EHRPWMPWMOpFreqSet(SOC_EPWM_2_REGS,
                       SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL,
                       (unsigned int)(SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL)/0xFF,
                       (unsigned int)EHRPWM_COUNT_UP,
                       (bool)EHRPWM_SHADOW_WRITE_DISABLE);

    /* Disable synchronization*/
    EHRPWMTimebaseSyncDisable(SOC_EPWM_2_REGS);

    /* Disable syncout*/
    EHRPWMSyncOutModeSet(SOC_EPWM_2_REGS, EHRPWM_SYNCOUT_DISABLE);

    /* Configure the emulation behaviour*/
    EHRPWMTBEmulationModeSet(SOC_EPWM_2_REGS, EHRPWM_STOP_AFTER_NEXT_TB_INCREMENT);

    /* Configure Counter compare cub-module */
    /* Load Compare A value */
    EHRPWMLoadCMPA(SOC_EPWM_2_REGS,
                   50,
                   (bool)EHRPWM_SHADOW_WRITE_DISABLE,
                   (unsigned int)EHRPWM_COMPA_NO_LOAD,
                   (bool)EHRPWM_CMPCTL_OVERWR_SH_FL);

    /* Load Compare B value */
    EHRPWMLoadCMPB(SOC_EPWM_2_REGS,
                   200,
                   (bool)EHRPWM_SHADOW_WRITE_DISABLE,
                   (unsigned int) EHRPWM_COMPB_NO_LOAD,
                   (bool)EHRPWM_CMPCTL_OVERWR_SH_FL);

    /* Configure Action qualifier */
    /* Toggle when CTR = CMPA */
    EHRPWMConfigureAQActionOnB(SOC_EPWM_2_REGS,
                               EHRPWM_AQCTLB_ZRO_DONOTHING,
                               EHRPWM_AQCTLB_PRD_DONOTHING,
                               EHRPWM_AQCTLB_CAU_EPWMXBTOGGLE,
                               EHRPWM_AQCTLB_CAD_DONOTHING,
                               EHRPWM_AQCTLB_CBU_DONOTHING,
                               EHRPWM_AQCTLB_CBD_DONOTHING,
                               EHRPWM_AQSFRC_ACTSFB_DONOTHING);

    /* Bypass dead band sub-module */
    EHRPWMDBOutput(SOC_EPWM_2_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);

    /* Disable Chopper sub-module */
    EHRPWMChopperDisable(SOC_EPWM_2_REGS);

    /* Disable trip events */
    EHRPWMTZTripEventDisable(SOC_EPWM_2_REGS,(bool)EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EPWM_2_REGS,(bool)EHRPWM_TZ_CYCLEBYCYCLE);

    /* Event trigger */
    /* Generate interrupt every 3rd occurance of the event */
    EHRPWMETIntPrescale(SOC_EPWM_2_REGS, EHRPWM_ETPS_INTPRD_THIRDEVENT);
    /* Generate event when CTR = CMPB */
    EHRPWMETIntSourceSelect(SOC_EPWM_2_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC);

    /* Disable High resolution capability */
    EHRPWMHRDisable(SOC_EPWM_2_REGS);

}
