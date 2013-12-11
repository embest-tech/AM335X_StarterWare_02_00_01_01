/**
 * \file   dmtimer.c
 *
 * \brief  This file consists of functions which will configure the DMTIMER.
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
#include "hw_cm_per.h"
#include "hw_cm_dpll.h"
#include "hw_types.h"

/*
 *
 * Note: No pin-muxing is required for DMTimer instances 0,1,2,3. 
 *       Hence they can be used directly. Hence no pin-muxing function
 *       is used for these instances.
 */


/*
 * \brief This function will enable the module clock for DMTIMER2 instance.
 *
 * \return None.
 */
void DMTimer2ModuleClkConfig(void)
{
    /* Clear CLKSEL field of CM_DPLL_CLKSEL_TIMER2_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER2_CLK) &=
          ~(CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL);

    /* Writing to the CLKSEL field of CM_DPLL_CLKSEL_TIMER2_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER2_CLK) |=
          CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL_CLK_M_OSC;

    /* Waiting for the CLKSEL field to reflect the written value. */
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER2_CLK) &
           CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL) !=
           CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL_CLK_M_OSC);

    /* Writing to MODULEMODE field of CM_PER_TIMER2_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER2_CLKCTRL) |=
                             CM_PER_TIMER2_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for the MODULEMODE field to reflect the written value. */
    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER2_CLKCTRL) &
    CM_PER_TIMER2_CLKCTRL_MODULEMODE) != CM_PER_TIMER2_CLKCTRL_MODULEMODE_ENABLE);

    /* 
    ** Waiting for the CLKACTIVITY_TIMER2_GCLK field of CM_PER_L4LS_CLKSTCTRL 
    ** register to be set.
    */
    while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER2_GCLK)));

}

/*
 * \brief This function will enable the module clock for DMTIMER3 instance.
 *
 * \return None.
 */
void DMTimer3ModuleClkConfig(void)
{
    /* Clear CLKSEL field of CM_DPLL_CLKSEL_TIMER3_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER3_CLK) &=
          ~(CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL);

    /* Writing to the CLKSEL field of CM_DPLL_CLKSEL_TIMER3_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER3_CLK) |=
          CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL_CLK_M_OSC;

    /* Waiting for the CLKSEL field to reflect the written value. */
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER3_CLK) &
           CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL) !=
           CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL_CLK_M_OSC);

    /* Writing to MODULEMODE field of CM_PER_TIMER3_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER3_CLKCTRL) |=
                             CM_PER_TIMER3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for the MODULEMODE field to reflect the written value. */
    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER3_CLKCTRL) &
    CM_PER_TIMER3_CLKCTRL_MODULEMODE) != CM_PER_TIMER3_CLKCTRL_MODULEMODE_ENABLE);

    /*
    ** Waiting for the CLKACTIVITY_TIMER3_GCLK field of CM_PER_L4LS_CLKSTCTRL
    ** register to be set.
    */
    while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER3_GCLK)));

}

/*
 * \brief This function will enable the module clock for DMTIMER4 instance.
 *
 * \return None.
 */
void DMTimer4ModuleClkConfig(void)
{
    /* Clear CLKSEL field of CM_DPLL_CLKSEL_TIMER4_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER4_CLK) &=
          ~(CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL);

    /* Writing to the CLKSEL field of CM_DPLL_CLKSEL_TIMER4_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER4_CLK) |=
          CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL_CLK_M_OSC;

    /* Waiting for the CLKSEL field to reflect the written value. */
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER4_CLK) &
           CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL) !=
           CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL_CLK_M_OSC);

    /* Writing to MODULEMODE field of CM_PER_TIMER4_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER4_CLKCTRL) |=
                             CM_PER_TIMER4_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for the MODULEMODE field to reflect the written value. */
    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER4_CLKCTRL) &
    CM_PER_TIMER4_CLKCTRL_MODULEMODE) != CM_PER_TIMER4_CLKCTRL_MODULEMODE_ENABLE);

    /*
    ** Waiting for the CLKACTIVITY_TIMER4_GCLK field of CM_PER_L4LS_CLKSTCTRL
    ** register to be set.
    */
    while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER4_GCLK)));

}

/*
 * \brief This function will enable the module clock for DMTIMER6 instance.
 *
 * \return None.
 */
void DMTimer6ModuleClkConfig(void)
{
    /* Clear CLKSEL field of CM_DPLL_CLKSEL_TIMER6_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER6_CLK) &=
          ~(CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL);

    /* Writing to the CLKSEL field of CM_DPLL_CLKSEL_TIMER6_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER6_CLK) |=
          CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL_SEL2;

    /* Waiting for the CLKSEL field to reflect the written value. */
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER6_CLK) &
           CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL) !=
          CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL_SEL2);

    /* Writing to MODULEMODE field of CM_PER_TIMER6_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER6_CLKCTRL) |=
                             CM_PER_TIMER6_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for the MODULEMODE field to reflect the written value. */
    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER6_CLKCTRL) &
    CM_PER_TIMER6_CLKCTRL_MODULEMODE) != CM_PER_TIMER6_CLKCTRL_MODULEMODE_ENABLE);

    /*
    ** Waiting for the CLKACTIVITY_TIMER6_GCLK field of CM_PER_L4LS_CLKSTCTRL
    ** register to be set.
    */
    while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER6_GCLK)));

}

/*
 * \brief This function will enable the module clock for DMTIMER7 instance.
 *
 * \return None.
 */
void DMTimer7ModuleClkConfig(void)
{
    /* Clear CLKSEL field of CM_DPLL_CLKSEL_TIMER7_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER7_CLK) &=
          ~(CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL);

    /* Writing to the CLKSEL field of CM_DPLL_CLKSEL_TIMER7_CLK register. */
    HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER7_CLK) |=
          CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL_CLK_M_OSC;

    /* Waiting for the CLKSEL field to reflect the written value. */
    while((HWREG(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER7_CLK) &
           CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL) !=
           CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL_CLK_M_OSC);

    /* Writing to MODULEMODE field of CM_PER_TIMER7_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_TIMER7_CLKCTRL) |=
                             CM_PER_TIMER7_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for the MODULEMODE field to reflect the written value. */
    while((HWREG(SOC_CM_PER_REGS + CM_PER_TIMER7_CLKCTRL) &
    CM_PER_TIMER7_CLKCTRL_MODULEMODE) != CM_PER_TIMER7_CLKCTRL_MODULEMODE_ENABLE);

    /*
    ** Waiting for the CLKACTIVITY_TIMER7_GCLK field of CM_PER_L4LS_CLKSTCTRL
    ** register to be set.
    */
    while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER7_GCLK)));

}

/****************************** End of file *********************************/
