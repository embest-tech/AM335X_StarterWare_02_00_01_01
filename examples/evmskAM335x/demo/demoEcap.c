/**
 * \file   demoEcap.c
 *
 * \brief  This file uses the PWM feature of ECAP to control the
 *         backlight of LCD.
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
#include "evmAM335x.h"
#include "ecap.h"
#include "ehrpwm.h"
#include "delay.h"

/******************************************************************************
**              INTERNAL FUNCTION PROTOTYPES
******************************************************************************/

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/
/*
** Enables Backlight for Ecap
*/
void EcapBkLightEnable(void)
{
    /* Start the counter */
    ECAPCounterControl(SOC_ECAP_2_REGS, ECAP_COUNTER_FREE_RUNNING);

    /* Configure ECAP to generate PWM waveform with 90% duty cycle */
    ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 3000, 3300);
}

/*
** Disables Backlight for Ecap
*/
void EcapBkLightDisable(void)
{
    /* Start the counter */
    ECAPCounterControl(SOC_ECAP_2_REGS, ECAP_COUNTER_FREE_RUNNING);

    /* Configure ECAP to generate PWM waveform with 0% duty cycle */
    ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 0, 3300);
}

/*
** Dims Backlight for Ecap
*/
void EcapBkLightDim(void)
{
    /* Start the counter */
    ECAPCounterControl(SOC_ECAP_2_REGS, ECAP_COUNTER_FREE_RUNNING);

    /* Configure ECAP to generate PWM waveform */
    ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 1500, 3300);
}

/*
** Varies backlight for Ecap
*/
void EcapBkLightVary(void)
{
    unsigned int cnt = 0;

    /* Set Counter */
    ECAPCounterConfig(SOC_ECAP_2_REGS, 0);

    while(cnt <= 2500) 
    {
        cnt++;

        /* Configure ECAP to generate PWM waveform with duty cycle */
        ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 3000 - cnt, 3300);

        delay(1);
    }
	
	while(cnt > 0) 
    {
        cnt--;

        /* Configure ECAP to generate PWM waveform with duty cycle */
        ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 3000 - cnt, 3300);

        delay(1); 
    }

    /* Configure ECAP to generate PWM waveform with 0% duty cycle */
    ECAPAPWMCaptureConfig(SOC_ECAP_2_REGS, 3000, 3300);
}

/*
** Initializes ECAP
*/
void EcapInit(void)
{
    /* Enable Interface and Module Clocks for PWMSS */
    PWMSSModuleClkConfig(2);

    /* Enable Clock */
    ECAPClockEnable(SOC_PWMSS2_REGS);

    /* Configure ECAP to Generate PWM wave form */
    ECAPOperatingModeSelect(SOC_ECAP_2_REGS, ECAP_APWM_MODE);

    /* Set Counter */
    ECAPCounterConfig(SOC_ECAP_2_REGS, 0);

    /* Configure ECAP_PWM_OUT pin as high */
    ECAPAPWMPolarityConfig(SOC_ECAP_2_REGS, ECAP_APWM_ACTIVE_LOW);
}


