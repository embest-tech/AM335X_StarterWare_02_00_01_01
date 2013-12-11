/**
 * \file   hsi2c.c
 *
 * \brief  This file contains functions which configure the hsi2c
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


#include "hw_control_AM335x.h"
#include "hw_cm_wkup.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "hw_cm_per.h"
#include "hw_types.h"

void I2CPinMuxSetup(unsigned int instance)
{

    if(instance == 0)
    {
         HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SDA)  =
                (CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_RXACTIVE  |
                 CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_SLEWCTRL  | 
                 CONTROL_CONF_I2C0_SDA_CONF_I2C0_SDA_PUTYPESEL   );

         HWREG(SOC_CONTROL_REGS + CONTROL_CONF_I2C0_SCL)  =
                (CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_RXACTIVE  |
                 CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_SLEWCTRL  | 
                 CONTROL_CONF_I2C0_SCL_CONF_I2C0_SCL_PUTYPESEL );

    } 
    else if(instance == 1)
    {
                               /* I2C_SCLK */   
        HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_D1)  = 
             (CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_PUTYPESEL |
              CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_RXACTIVE  |
              CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_SLEWCTRL  |
              CONTROL_CONF_MUXMODE(2));                     
                              /* I2C_SDA */
        HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_CS0) = 
             (CONTROL_CONF_SPI0_CS0_CONF_SPI0_CS0_PUTYPESEL |
              CONTROL_CONF_SPI0_CS0_CONF_SPI0_CS0_RXACTIVE  |
              CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_SLEWCTRL    |
              CONTROL_CONF_MUXMODE(2));

    }
}

/**
 * \brief   This function will configure the required clocks for I2C1 instance.
 *
 * \return  None.
 *
 */
void I2C1ModuleClkConfig(void)
{
    HWREG(SOC_PRCM_REGS + CM_PER_I2C1_CLKCTRL) |= 
                             CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_I2C1_CLKCTRL) & 
      CM_PER_I2C1_CLKCTRL_MODULEMODE) != CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE);

    while(!(HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) & 
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK | 
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK)));
    
}


/*
** This function enables the system L3 and system L4_WKUP clocks.
** This also enables the clocks for I2C0 instance.
*/
void I2C0ModuleClkConfig(void)
{
   /* Writing to MODULEMODE field of CM_WKUP_I2C0_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
          CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_MODULEMODE));


    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));
    /*
    ** Waiting for CLKACTIVITY_I2C0_GFCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_I2C0_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_I2C0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_IDLEST));
}

