/**
 *  \file   demoDvfs.c
 *
 *  \brief  Functions used to change the opp at which core operates.
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

#include "demoDvfs.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "hw_types.h"
#include "pmic.h"
#include "device.h"
#include "consoleUtils.h"
#include "demoCfg.h"

unsigned int mpuOpp;
unsigned int mpuFreq;
unsigned int mpuVdd1;

/* OPP table for mpu multiplier and pmic voltage select */
tOPPConfig oppTable[] =
{
    {MPUPLL_M_275_MHZ, PMIC_VOLT_SEL_1100MV},   /* OPP100 275Mhz - 1.1v */
    {MPUPLL_M_500_MHZ, PMIC_VOLT_SEL_1100MV},   /* OPP100 500Mhz - 1.1v */
    {MPUPLL_M_600_MHZ, PMIC_VOLT_SEL_1200MV},   /* OPP120 600Mhz - 1.2v */
    {MPUPLL_M_720_MHZ, PMIC_VOLT_SEL_1260MV},   /* OPP TURBO 720Mhz - 1.26v */
};

/* OPP to string table */
tValStr oppStrMap[] =
{
    {OPP_NONE, ""},          /* NONE */
    {OPP_50, "OPP50"},       /* OPP50 */
    {OPP_100, "OPP100"},     /* OPP100 */
    {OPP_120, "OPP120"},     /* OPP120 */
    {SR_TURBO, "SR TURBO"}   /* SR TURBO */
};

/* Freqency to string table */
tValStr freqStrMap[] =
{
    {MPUPLL_M_275_MHZ, "275"},    /* 275Mhz */
    {MPUPLL_M_500_MHZ, "500"},    /* 500Mhz */
    {MPUPLL_M_600_MHZ, "600"},    /* 600Mhz */
    {MPUPLL_M_720_MHZ, "720"},    /* 720Mhz */
    {0, ""}                       /* NULL */
};

/* Voltage to string table */
tValStr voltStrMap[] =
{
    {PMIC_VOLT_SEL_1100MV, "1.1"},  /* 1.1v */
    {PMIC_VOLT_SEL_1200MV, "1.2"},  /* 1.2v */
    {PMIC_VOLT_SEL_1260MV, "1.26"}, /* 1.26v */
    {0, ""}                         /* NULL */
};

/*
** This function determines the vdd1 voltage for OPP.
*/
unsigned int VddVoltageGet(unsigned int Opp)
{
    switch(Opp)
    {
        case OPP_50:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                /*
                ** Set vdd1 voltage to 1.1V for SoC ver 1.0.
                ** The minimum voltage level for OPP100.
                */
                return PMIC_VOLT_SEL_1100MV;
            }
            break;

        case OPP_100:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return PMIC_VOLT_SEL_1100MV;
            }
            break;

        case OPP_120:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return PMIC_VOLT_SEL_1200MV;
            }
            break;

        case SR_TURBO:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return PMIC_VOLT_SEL_1260MV;
            }
            break;

        default:
            break;
    }

    return false;
}

/*
** This function returns the frequency corresponding to an OPP.
*/
unsigned int FrequencyGet(unsigned int Opp)
{
    switch(Opp)
    {
        case OPP_50:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return MPUPLL_M_275_MHZ;
            }
            break;

        case OPP_100:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return MPUPLL_M_500_MHZ;
            }
            break;

        case OPP_120:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return MPUPLL_M_600_MHZ;
            }
            break;

        case SR_TURBO:
            if(DEVICE_VERSION_1_0 == deviceVersion)
            {
                return MPUPLL_M_720_MHZ;
            }
            break;

        default:
            break;
    }

    return false;
}

/*
** Configure the frequency and voltage to lowest OPP configuration of SoC.
*/
unsigned int ConfigMinimumOPP(void)
{
    unsigned int oppIdx;

    if(DEVICE_VERSION_1_0 == deviceVersion)
    {
        oppIdx = EFUSE_OPP100_275;
    }
    else
    {
        return OPP_NONE;
    }

    DemoMpuPllConfig(oppTable[oppIdx].pllMult);
    setVdd1OpVoltage(oppTable[oppIdx].pmicVolt);

    return OPP_50;
}

/*
** Configure the frequency and voltage to maximum OPP configuration of SoC.
*/
unsigned int ConfigMaximumOPP(void)
{
    unsigned int oppIdx;

    if(DEVICE_VERSION_1_0 == deviceVersion)
    {
        oppIdx = EFUSE_OPPTB_720;
    }
    else
    {
        return OPP_NONE;
    }

    setVdd1OpVoltage(oppTable[oppIdx].pmicVolt);
    DemoMpuPllConfig(oppTable[oppIdx].pllMult);

    return SR_TURBO;
}

/*
** Clock for cortex-A8 is fed from MPUPLL.
** This function configures MPUPLL to run the cortex-A8 at desired frequency.
*/
void DemoMpuPllConfig(unsigned int freq)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    /* Wait for DPLL to go in to bypass mode */
    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) &
                CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS));

    /* Clear the MULT and DIV field of DPLL_MPU register */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) &=
                      ~(CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT |
                        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV);

    /* Set the multiplier and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) |=
         ((freq << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT) |
         (MPUPLL_N << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU);

    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV;

    regVal = regVal | MPUPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU) = regVal;

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) &
                             CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK));
}

/*
** Print current OPP confiuration
*/
void PrintConfigDVFS(void)
{
    tValStr *tempValStr;

    /*
    ** Print string name of the OPP.
    */
    ConsoleUtilsPrintf("\n\r%s mode selected (vdd_mpu at ",
                       oppStrMap[mpuOpp].str);

    tempValStr = &voltStrMap[0];

    /* Enter a loop to search the matching voltage string. */
    while(tempValStr->val)
    {
        /*
        ** If a match is found, then print string name of the voltage.
        */
        if(tempValStr->val == mpuVdd1)
        {
            ConsoleUtilsPrintf("%sV,  ", tempValStr->str);
        }

        tempValStr++;
    }

    tempValStr = &freqStrMap[0];

    /* Enter a loop to search the matching frequency string. */
    while(tempValStr->val)
    {
        /*
        ** If a match is found, then print string name of the frequency.
        */
        if(tempValStr->val == mpuFreq)
        {
            ConsoleUtilsPrintf("%sMHz) \r\n", tempValStr->str);
        }

        tempValStr++;
    }
}
