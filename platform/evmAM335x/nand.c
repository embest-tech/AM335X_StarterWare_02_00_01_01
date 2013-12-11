/**
 * \file   uart.c
 *
 * \brief  This file contains functions which does the platform specific
 *         configurations for UART.
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
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "hw_cm_per.h"
#include "evmAM335x.h"
#include "hw_types.h"

/**
 * \brief   This function selects the GPMC pins for NAND use. The GPMC pins
 *          are multiplexed with pins of other peripherals in the SoC
 *          
 * \return  TRUE/FALSE
 *
 * \note    This pin multiplexing depends on the profile in which the EVM
 *          is configured.
 */


unsigned int NANDPinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch (profile)
    {
        /* All profiles have the same setting. */
        case 0:
        case 1:
        case 4:
        case 5:
        case 6:
        case 7:
                /* GPMC_AD0 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(0)) =
                ( 0 << CONTROL_CONF_GPMC_AD0_CONF_GPMC_AD0_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD0_CONF_GPMC_AD0_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD0_CONF_GPMC_AD0_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD0_CONF_GPMC_AD0_RXACTIVE_SHIFT);

                /* GPMC_AD1 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(1)) =
                ( 0 << CONTROL_CONF_GPMC_AD1_CONF_GPMC_AD1_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD1_CONF_GPMC_AD1_PUDEN_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_AD1_CONF_GPMC_AD1_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD1_CONF_GPMC_AD1_RXACTIVE_SHIFT) ;
                /* GPMC_AD2 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(2)) =
                ( 0 << CONTROL_CONF_GPMC_AD2_CONF_GPMC_AD2_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD2_CONF_GPMC_AD2_PUDEN_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_AD2_CONF_GPMC_AD2_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD2_CONF_GPMC_AD2_RXACTIVE_SHIFT) ;
                /* GPMC_AD3 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(3)) =
                ( 0 << CONTROL_CONF_GPMC_AD3_CONF_GPMC_AD3_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD3_CONF_GPMC_AD3_PUDEN_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_AD3_CONF_GPMC_AD3_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD3_CONF_GPMC_AD3_RXACTIVE_SHIFT) ;
                /* GPMC_AD4 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(4)) =
                ( 0 << CONTROL_CONF_GPMC_AD4_CONF_GPMC_AD4_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD4_CONF_GPMC_AD4_PUDEN_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_AD4_CONF_GPMC_AD4_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD4_CONF_GPMC_AD4_RXACTIVE_SHIFT) ;
                /* GPMC_AD5 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(5)) =
                ( 0 << CONTROL_CONF_GPMC_AD5_CONF_GPMC_AD5_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD5_CONF_GPMC_AD5_PUDEN_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_AD5_CONF_GPMC_AD5_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD5_CONF_GPMC_AD5_RXACTIVE_SHIFT) ;
                /* GPMC_AD6 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(6)) =
                ( 0 << CONTROL_CONF_GPMC_AD6_CONF_GPMC_AD6_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD6_CONF_GPMC_AD6_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD6_CONF_GPMC_AD6_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD6_CONF_GPMC_AD6_RXACTIVE_SHIFT) ;
                /* GPMC_AD7 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(7)) =
                ( 0 << CONTROL_CONF_GPMC_AD7_CONF_GPMC_AD7_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD7_CONF_GPMC_AD7_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD7_CONF_GPMC_AD7_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD7_CONF_GPMC_AD7_RXACTIVE_SHIFT) ;
                /* GPMC_AD8 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(8)) =
                ( 0 << CONTROL_CONF_GPMC_AD8_CONF_GPMC_AD8_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD8_CONF_GPMC_AD8_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD8_CONF_GPMC_AD8_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD8_CONF_GPMC_AD8_RXACTIVE_SHIFT) ;
                /* GPMC_AD9 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(9)) =
                ( 0 << CONTROL_CONF_GPMC_AD9_CONF_GPMC_AD9_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD9_CONF_GPMC_AD9_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD9_CONF_GPMC_AD9_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD9_CONF_GPMC_AD9_RXACTIVE_SHIFT) ;
                /* GPMC_AD10 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(10)) =
                ( 0 << CONTROL_CONF_GPMC_AD10_CONF_GPMC_AD10_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD10_CONF_GPMC_AD10_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD10_CONF_GPMC_AD10_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD10_CONF_GPMC_AD10_RXACTIVE_SHIFT) ;
                /* GPMC_AD11 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(11)) =
                ( 0 << CONTROL_CONF_GPMC_AD11_CONF_GPMC_AD11_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD11_CONF_GPMC_AD11_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD11_CONF_GPMC_AD11_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD11_CONF_GPMC_AD11_RXACTIVE_SHIFT) ;
                /* GPMC_AD12 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(12)) =
                ( 0 << CONTROL_CONF_GPMC_AD12_CONF_GPMC_AD12_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD12_CONF_GPMC_AD12_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD12_CONF_GPMC_AD12_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD12_CONF_GPMC_AD12_RXACTIVE_SHIFT) ;

                /* GPMC_AD13 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(13)) =
                ( 0 << CONTROL_CONF_GPMC_AD13_CONF_GPMC_AD13_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD13_CONF_GPMC_AD13_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD13_CONF_GPMC_AD13_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD13_CONF_GPMC_AD13_RXACTIVE_SHIFT) ;
                /* GPMC_AD14 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(14)) =
                ( 0 << CONTROL_CONF_GPMC_AD14_CONF_GPMC_AD14_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD14_CONF_GPMC_AD14_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD14_CONF_GPMC_AD14_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD14_CONF_GPMC_AD14_RXACTIVE_SHIFT) ;
                /* GPMC_AD15 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(15)) =
                ( 0 << CONTROL_CONF_GPMC_AD15_CONF_GPMC_AD15_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD15_CONF_GPMC_AD15_PUDEN_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_AD15_CONF_GPMC_AD15_PUTYPESEL_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_AD15_CONF_GPMC_AD15_RXACTIVE_SHIFT) ;

                /* GPMC_WAIT0 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_WAIT0) =
                ( 0 << CONTROL_CONF_GPMC_WAIT0_CONF_GPMC_WAIT0_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_WAIT0_CONF_GPMC_WAIT0_PUDEN_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_WAIT0_CONF_GPMC_WAIT0_PUTYPESEL_SHIFT)|
                ( 1 << CONTROL_CONF_GPMC_WAIT0_CONF_GPMC_WAIT0_RXACTIVE_SHIFT);

                /* GPMC_WPN */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_WPN) =
                ( 0 << CONTROL_CONF_GPMC_WPN_CONF_GPMC_WPN_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_WPN_CONF_GPMC_WPN_PUDEN_SHIFT) |
                ( 1 << CONTROL_CONF_GPMC_WPN_CONF_GPMC_WPN_PUTYPESEL_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_WPN_CONF_GPMC_WPN_RXACTIVE_SHIFT);

                /* GPMC_CS0 */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_CSN(0)) =
                ( 0 << CONTROL_CONF_GPMC_CSN0_CONF_GPMC_CSN0_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_CSN0_CONF_GPMC_CSN0_PUDEN_SHIFT)|
                ( 1 << CONTROL_CONF_GPMC_CSN0_CONF_GPMC_CSN0_PUTYPESEL_SHIFT)|
                ( 0 << CONTROL_CONF_GPMC_CSN0_CONF_GPMC_CSN0_RXACTIVE_SHIFT);

                /* GPMC_ALE */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_ADVN_ALE) =
                ( 0 << CONTROL_CONF_GPMC_ADVN_ALE_CONF_GPMC_ADVN_ALE_MMODE_SHIFT
                ) |
                ( 0 << CONTROL_CONF_GPMC_ADVN_ALE_CONF_GPMC_ADVN_ALE_PUDEN_SHIFT
                )  |
                ( 1 <<
                CONTROL_CONF_GPMC_ADVN_ALE_CONF_GPMC_ADVN_ALE_PUTYPESEL_SHIFT) |
                ( 0 <<
                CONTROL_CONF_GPMC_ADVN_ALE_CONF_GPMC_ADVN_ALE_RXACTIVE_SHIFT);

                /* GPMC_BE0N_CLE */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_BE0N_CLE) =
                ( 0 << CONTROL_CONF_GPMC_BE0N_CLE_CONF_GPMC_BE0N_CLE_MMODE_SHIFT
                ) |
                ( 0 << CONTROL_CONF_GPMC_BE0N_CLE_CONF_GPMC_BE0N_CLE_PUDEN_SHIFT
                )  |
                ( 1 <<
                CONTROL_CONF_GPMC_BE0N_CLE_CONF_GPMC_BE0N_CLE_PUTYPESEL_SHIFT) |
                ( 0 <<
                CONTROL_CONF_GPMC_BE0N_CLE_CONF_GPMC_BE0N_CLE_RXACTIVE_SHIFT);

                /* GPMC_OEN_REN */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_OEN_REN) =
                ( 0 << CONTROL_CONF_GPMC_OEN_REN_CONF_GPMC_OEN_REN_MMODE_SHIFT
                ) |
                ( 0 << CONTROL_CONF_GPMC_OEN_REN_CONF_GPMC_OEN_REN_PUDEN_SHIFT
                )  |
                ( 1 <<
                CONTROL_CONF_GPMC_OEN_REN_CONF_GPMC_OEN_REN_PUTYPESEL_SHIFT) |
                ( 0 <<
                CONTROL_CONF_GPMC_OEN_REN_CONF_GPMC_OEN_REN_RXACTIVE_SHIFT);

                /* GPMC_WEN */
                HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_WEN) =
                ( 0 << CONTROL_CONF_GPMC_WEN_CONF_GPMC_WEN_MMODE_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_WEN_CONF_GPMC_WEN_PUDEN_SHIFT)  |
                ( 1 << CONTROL_CONF_GPMC_WEN_CONF_GPMC_WEN_PUTYPESEL_SHIFT) |
                ( 0 << CONTROL_CONF_GPMC_WEN_CONF_GPMC_WEN_RXACTIVE_SHIFT);
                 status = TRUE;

        break;

        case 2:
        case 3:
        break;

        default:
        break;
    }
    return status;
}

/*
** This function enables the system L3S and system L4_WKUP clocks.
** This also enables the clocks for UART0 instance.
*/

void GPMCClkConfig(void)
{
    HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) |=
                             CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
     CM_PER_L3S_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) |=
                             CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
     CM_PER_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
                             CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) &
                               CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) !=
                                   CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) |=
                             CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) &
        CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
                             CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
                              CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL) !=
                                CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_GPMC_CLKCTRL) |=
                             CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_GPMC_CLKCTRL) &
      CM_PER_GPMC_CLKCTRL_MODULEMODE) != CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_PRCM_REGS + CM_PER_ELM_CLKCTRL) |=
                             CM_PER_ELM_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_ELM_CLKCTRL) &
      CM_PER_ELM_CLKCTRL_MODULEMODE) != CM_PER_ELM_CLKCTRL_MODULEMODE_ENABLE);


    while(!(HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
            CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));

    while(!(HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
            CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

}

/****************************** End of file *********************************/
