/**
 * \file   mcasp.c
 *
 * \brief  This file contains functions which configure McASP pins
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
#include "hw_control_AM335x.h"
#include "hw_types.h"
#include "evmAM335x.h"
#include "hw_cm_per.h"

#define MCASP_SEL_MODE            0x04

/**
 * \brief   This function selects the McASP instance 1 pins
 *          
 * \param   None
 *
 * \return  TRUE/FALSE.
 *
 * \note    This muxing depends on the profile in which the EVM is configured.
 */
unsigned int McASP1PinMuxSetup(void)
{
    unsigned int profile;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch (profile)
    {
        case 0:
        case 3:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_MII1_COL) = 
                          CONTROL_CONF_MII1_RXERR_CONF_MII1_RXERR_RXACTIVE 
                          | MCASP_SEL_MODE;            
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_MII1_CRS) = 
                          CONTROL_CONF_MII1_RXERR_CONF_MII1_RXERR_RXACTIVE
                          | MCASP_SEL_MODE;            
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_MII1_RXERR) =
                          CONTROL_CONF_MII1_RXERR_CONF_MII1_RXERR_RXACTIVE
                          | MCASP_SEL_MODE;            
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_RMII1_REFCLK) = 
                          CONTROL_CONF_RMII1_REFCLK_CONF_RMII1_REFCLK_RXACTIVE 
                          | MCASP_SEL_MODE;            
            status = TRUE;
        break; 

        default:
        break;
    }
    
    return status;
}

/**
 * \brief   This function enables McASP clocks
 *          
 * \param   None
 *
 * \return  None.
 *
 */
void McASP1ModuleClkConfig(void)
{
    HWREG(SOC_PRCM_REGS + CM_PER_MCASP1_CLKCTRL) =
                             CM_PER_MCASP1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_MCASP1_CLKCTRL) &
      CM_PER_MCASP1_CLKCTRL_MODULEMODE) != CM_PER_MCASP1_CLKCTRL_MODULEMODE_ENABLE);


    /*
    ** Waiting for IDLEST field in CM_PER_MCASP1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_MCASP1_CLKCTRL_IDLEST_FUNC <<
           CM_PER_MCASP1_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_MCASP1_CLKCTRL) &
           CM_PER_MCASP1_CLKCTRL_IDLEST));

}

/****************************** End Of File *********************************/
