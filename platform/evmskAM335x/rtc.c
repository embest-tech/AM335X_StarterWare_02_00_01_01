/**
 * \file   rtc.c
 *
 * \brief  This file contains functions which does the platform specific
 *         configurations for RTC.
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
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
*/


#include "hw_cm_wkup.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "hw_cm_per.h"
#include "hw_cm_rtc.h"
#include "hw_types.h"

/**
 * \brief  This API returns a unique number which identifies itself
 *         with the RTC IP in AM335X SoC.
 *
 * \param  None
 *
 * \return This returns a number '2' which is unique to RTC IP in AM335X.
 */

unsigned int RtcVersionGet(void)
{
    return 2;
}

/*
** This function enables the system L3 and system L4_WKUP clocks.
** This also enables the clocks for the RTC module.
*/

void RTCModuleClkConfig(void)
{
    /* Performing System Clock configurations for RTC. */

    /* Writing to MODULEMODE field of CM_RTC_RTC_CLKCTRL register. */
    HWREG(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL) |=
          CM_RTC_RTC_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_RTC_RTC_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL) &
           CM_RTC_RTC_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_RTC_CLKSTCTRL register. */
    HWREG(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL) |=
          CM_RTC_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_RTC_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL) &
           CM_RTC_CLKSTCTRL_CLKTRCTRL));

    /* Checking the RTC related registers for desired values being set. */

    /*
    ** Waiting for IDLEST field in CM_RTC_RTC_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_RTC_RTC_CLKCTRL_IDLEST_FUNC <<
           CM_RTC_RTC_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL) &
           CM_RTC_RTC_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_RTC_GCLK field in CM_RTC_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_RTC_CLKSTCTRL_CLKACTIVITY_L4_RTC_GCLK !=
          (HWREG(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL) &
           CM_RTC_CLKSTCTRL_CLKACTIVITY_L4_RTC_GCLK));

    /*
    ** Waiting for CLKACTIVITY_RTC_32KCLK field in CM_RTC_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_RTC_CLKSTCTRL_CLKACTIVITY_RTC_32KCLK !=
          (HWREG(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL) &
           CM_RTC_CLKSTCTRL_CLKACTIVITY_RTC_32KCLK));
}

/******************************* End of file ********************************/
