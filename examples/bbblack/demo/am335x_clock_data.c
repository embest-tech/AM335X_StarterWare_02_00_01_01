/**
 * \file     am335x_clock.c
 *
 * \brief    This file contains the definitions of Clocks used in the application
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

#include "clock.h"
#include "soc_AM335x.h"
#include "hw_control_AM335x.h"

/* TEMP */
#define CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL_CLK32768 	CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL_SEL2
#define CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_DISP_PLL	CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_SEL1
#define CM_DPLL_CM_CPTS_RFT_CLKSEL_CLKSEL_SYSCLK1	CM_DPLL_CM_CPTS_RFT_CLKSEL_CLKSEL_SEL2
#define CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_PER_PLL	CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_SEL3
#define CM_DPLL_CM_MAC_CLKSEL_MII_CLK_SEL_SYSCLK2_DIV5	CM_DPLL_CM_MAC_CLKSEL_MII_CLK_SEL_SEL0
#define CM_MPU_MPU_CLKCTRL_MODULEMODE_DISABLE		CM_MPU_MPU_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_EMIF_CLKCTRL_MODULEMODE_DISABLE		CM_PER_EMIF_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_I2C1_CLKCTRL_MODULEMODE_DISABLE		CM_PER_I2C1_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_SPI0_CLKCTRL_MODULEMODE_DISABLE      CM_PER_SPI0_CLKCTRL_MODULEMODE_DISABLED
#define CM_WKUP_UART0_CLKCTRL_MODULEMODE_DISABLE    CM_WKUP_UART0_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_DISABLE  CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_DISABLED
#define CM_RTC_RTC_CLKCTRL_MODULEMODE_DISABLE       CM_RTC_RTC_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_DISABLE      CM_PER_L4LS_CLKCTRL_MODULEMODE_DISABLED
#define CM_WKUP_WDT1_CLKCTRL_MODULEMODE_DISABLE     CM_WKUP_WDT1_CLKCTRL_MODULEMODE_DISABLED
#define CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_DISABLE  CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_LCDC_CLKCTRL_MODULEMODE_DISABLE      CM_PER_LCDC_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_MCASP1_CLKCTRL_MODULEMODE_DISABLE    CM_PER_MCASP1_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_DISABLE   CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_L3_CLKCTRL_MODULEMODE_DISABLE   		CM_PER_L3_CLKCTRL_MODULEMODE_DISABLED
#define CM_PER_L4FW_CLKCTRL_MODULEMODE_DISABLE   	CM_PER_L4FW_CLKCTRL_MODULEMODE_DISABLED


/*	EVM	*/
#define EVM_15X15	1
//#define EVM_13X13	1

ClockDomain l4lsClkDomain;
Clock lcdcGclkClock;
Clock gpio3GclkClock;
Clock i2cGclkClock;
Clock spiGclkClock;
Clock uartGclkClock;
Clock rtc32kGclkClock;
Clock wdt1GclkClock;
Clock adcFclkClock;
Clock l4WkupGclkClock;
Clock uart0GclkClock;
Clock mcaspGclkClock;
Clock cpswCptsRftGclkClock;
Clock emifGclkClock;
Clock l4WkupAonGclkClock;
Clock mpuIclkClock;
Clock lcdcL3IclkClock;
Clock lcdcL4IclkClock;
Clock core100MClock;
Clock l3GclkClock;
Clock l4fwGclkClock;
Clock l3sGclkClock;
Clock cpsw125MhzGclkClock;
Clock clk24MhzGclkClock;
Clock cpsw5MhzGclkClock;
Clock i2c0GclkClock;
Clock gpio0GclkClock;
Clock timer1IclkClock;
ClockDomain wkupClkDomain;


#ifdef __cplusplus
extern "C" {
#endif

/*	STATIC CLOCK CONFIGURATION	*/

/************************* PLL's ********************************/
ADPLL dpllCore = {
	.isPLLConfigured	=	FALSE,
	.autoIdleCtrlReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_AUTOIDLE_DPLL_CORE),
	/*to have deterministic value for showing power numbers
		in the demo auto idle is disabled*/
	.autoIdleCtrlVal	=
		(CM_WKUP_CM_AUTOIDLE_DPLL_CORE_AUTO_DPLL_MODE_AUTO_CTL_DISABLE <<
							CM_WKUP_CM_AUTOIDLE_DPLL_CORE_AUTO_DPLL_MODE_SHIFT),
	.idleStatusReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_CORE),
	.adpllClkStatusMask =	CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK,
	.adpllClkStatusShift =	CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK_SHIFT,
	.adpllClkMNBypassStatusMask	 =	CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS,
	.adpllClkMNBypassStatusShift =	CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS_SHIFT,
	.adpllConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_CORE),
	/*.adpllBypassClkSource = NULL // MOSC is the only bypass clock, so no config is reqd */
	/* 1000Mhz as dpll o/p */
	.adpllMultiplierMask=	 CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT,
	.adpllMultiplier	=	{(1000 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT),
							 (1000 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT),
							 (1000 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT),
							 (100 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT)},
	.adpllDividerMask	=	CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV,
	.adpllDivider		=	{(23 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV_SHIFT)},
	.adpllModeReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_CORE),

	.adpllLowPowerMask	=	(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_LPMODE_EN |
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN),
	.adpllLowPowerNormalVal	=	((CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_LPMODE_EN_DISABLED << 	/* disable */
								CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_LPMODE_EN_SHIFT) |
							(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_DPLL_LOCK_MODE <<  	/* lock mode. confirm if this is corrent */
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_SHIFT)),

	.adpllLowPowerBypassVal = ((CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_LPMODE_EN_ENABLED << 	/* Enable */
								CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_LPMODE_EN_SHIFT) |
							   (CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_DPLL_LP_BYP_MODE <<  	/* bypass mode */
								CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_SHIFT)),

	.adpllRelockRampMask=	(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RELOCK_RAMP_EN |
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_DRIFTGUARD_EN |
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RAMP_RATE |
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RAMP_LEVEL),

	.adpllRelockRampVal	=	(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RELOCK_RAMP_EN | 		/* active logic is not clear */
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_DRIFTGUARD_EN	| 			/* drift guard is enabled */
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RAMP_RATE_REFCLKX2 |		/* ramp rate */
							(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RAMP_LEVEL_RAMP_ALGO1 <<	/* ramp algo1 */
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_RAMP_LEVEL_SHIFT)),
};


ADPLL dpllMpu = {
	.isPLLConfigured	=	FALSE,
	.autoIdleCtrlReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_AUTOIDLE_DPLL_MPU),
	.autoIdleCtrlVal	=
		(CM_WKUP_CM_AUTOIDLE_DPLL_MPU_AUTO_DPLL_MODE_AUTO_CTL_DISABLE <<
							CM_WKUP_CM_AUTOIDLE_DPLL_MPU_AUTO_DPLL_MODE_SHIFT),
	.idleStatusReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU),
	.adpllClkStatusMask =	CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK,
	.adpllClkStatusShift =	CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK_SHIFT,
	.adpllClkMNBypassStatusMask	 =	CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS,
	.adpllClkMNBypassStatusShift =	CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS_SHIFT,
	.adpllConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU),
	.adpllBypassClkMask	=	CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_BYP_CLKSEL,
	.adpllBypassClkSource = (CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_BYP_CLKSEL_SEL0 <<
							CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_BYP_CLKSEL_SHIFT),
	.adpllMultiplierMask=	 CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT,
	.adpllMultiplier	=	{(720 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT),
							(650 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT),
							(550 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT),
							(275 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT)},
	.adpllDividerMask	=	CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV,
	.adpllDivider		=	{(23 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT),
							(23 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT),
							(23 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT),
							(23 << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT)},
	.adpllModeReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU),

	.adpllLowPowerMask	=	(CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_LPMODE_EN |
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN),
	.adpllLowPowerNormalVal	=	((CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_LPMODE_EN_DISABLED << 	/* disable */
								CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_LPMODE_EN_SHIFT) |
							(CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_LOCK_MODE <<  	/* lock mode. confirm if this is corrent */
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_SHIFT)),

	.adpllLowPowerBypassVal = ((CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_LPMODE_EN_ENABLED << 	/* Enable */
								CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_LPMODE_EN_SHIFT) |
							   (CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_LP_BYP_MODE <<  	/* bypass mode */
								CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_SHIFT)),

	.adpllRelockRampMask=	(CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RELOCK_RAMP_EN |
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_DRIFTGUARD_EN |
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RAMP_RATE |
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RAMP_LEVEL),

	.adpllRelockRampVal	=	(CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RELOCK_RAMP_EN | 		/* active logic is not clear */
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_DRIFTGUARD_EN	| 			/* drift guard is enabled */
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RAMP_RATE_REFCLKX2 |		/* ramp rate */
							(CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RAMP_LEVEL_RAMP_ALGO1 <<	/* ramp algo1 */
							CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_RAMP_LEVEL_SHIFT)),

};


ADPLL dpllPer = {
	.isPLLConfigured	=	FALSE,
	.autoIdleCtrlReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_AUTOIDLE_DPLL_PER),
	.autoIdleCtrlVal	=
		(CM_WKUP_CM_AUTOIDLE_DPLL_PER_AUTO_DPLL_MODE_AUTO_CTL_DISABLE <<
							CM_WKUP_CM_AUTOIDLE_DPLL_PER_AUTO_DPLL_MODE_SHIFT),
	.idleStatusReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_PER),
	.adpllClkStatusMask =	CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK,
	.adpllClkStatusShift =	CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK_SHIFT,
	.adpllClkMNBypassStatusMask	 =	CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS,
	.adpllClkMNBypassStatusShift =	CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS_SHIFT,
	.adpllConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_PERIPH),
	.adpllSigmaDeltaDividerMask = CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_SD_DIV,
	.adpllSigmaDeltaDivider = {4,4,4,4},
	.adpllMultiplierMask=	 CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT,
	.adpllMultiplier	=	{(960 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT),
							 (960 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT),
							 (960 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT),
							 (480 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT)},
	.adpllDividerMask	=	CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV,
	.adpllDivider		=	{(23 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV_SHIFT)},
	.adpllModeReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_PER),

	.adpllLowPowerMask	=	CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN,
	.adpllLowPowerNormalVal	=	(CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_LOCK_MODE <<	/*	lock mode	*/
							CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_SHIFT),

	.adpllLowPowerBypassVal = (CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_LP_BYP_MODE <<  	/* bypass mode */
								CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_SHIFT),

};


ADPLL dpllDisp = {
	.isPLLConfigured	=	FALSE,
	.autoIdleCtrlReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_AUTOIDLE_DPLL_DISP),
	.autoIdleCtrlVal	=
		(CM_WKUP_CM_AUTOIDLE_DPLL_DISP_AUTO_DPLL_MODE_AUTO_CTL_DISABLE <<
							CM_WKUP_CM_AUTOIDLE_DPLL_DISP_AUTO_DPLL_MODE_SHIFT),
	.idleStatusReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DISP),
	.adpllClkStatusMask =	CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK,
	.adpllClkStatusShift =	CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK_SHIFT,
	.adpllClkMNBypassStatusMask	 =	CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS,
	.adpllClkMNBypassStatusShift =	CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS_SHIFT,
	.adpllConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DISP),
	.adpllBypassClkMask	=	CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_BYP_CLKSEL,
	.adpllBypassClkSource = (CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_BYP_CLKSEL_SEL0 <<
							CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_BYP_CLKSEL_SHIFT),
	/*	150Mhz of Pixel clock -> 600Mhz Disp PLL clock	*/
	/*	COnfigure PLL to 1200Mhz, so that CLKOUT will be 600Mhz	*/
	.adpllMultiplierMask=	 CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT,
	.adpllMultiplier	=	{(25 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT),
							 (25 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT),
							 (25 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT),
							 (25 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT)},
	.adpllDividerMask	=	CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV,
	.adpllDivider		=	{(3 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT),
							 (3 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT),
							 (3 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT),
							 (11 << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV_SHIFT)},
	.adpllModeReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DISP),

	.adpllLowPowerMask	=	(CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_LPMODE_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN),
	.adpllLowPowerNormalVal	=	((CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_LPMODE_EN_DISABLED << 	/* disable */
								CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_LPMODE_EN_SHIFT) |
							(CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_LOCK_MODE <<  	/* lock mode. confirm if this is corrent */
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_SHIFT)),

	.adpllLowPowerBypassVal = ((CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_LPMODE_EN_ENABLED << 	/* Enable */
								CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_LPMODE_EN_SHIFT) |
							   (CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_LP_BYP_MODE <<  	/* bypass mode */
								CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_SHIFT)),

	.adpllRelockRampMask=	(CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RELOCK_RAMP_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_DRIFTGUARD_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RAMP_RATE |
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RAMP_LEVEL),

	.adpllRelockRampVal	=	(CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RELOCK_RAMP_EN | 		/* active logic is not clear */
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_DRIFTGUARD_EN	| 			/* drift guard is enabled */
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RAMP_RATE_REFCLKX2 |		/* ramp rate */
							(CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RAMP_LEVEL_RAMP_ALGO1 <<	/* ramp algo1 */
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_RAMP_LEVEL_SHIFT)),

};


ADPLL dpllDDR = {
	.isPLLConfigured	=	FALSE,
	.autoIdleCtrlReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_AUTOIDLE_DPLL_DDR),
	.autoIdleCtrlVal	=
		(CM_WKUP_CM_AUTOIDLE_DPLL_DDR_AUTO_DPLL_MODE_AUTO_CTL_DISABLE <<
							CM_WKUP_CM_AUTOIDLE_DPLL_DDR_AUTO_DPLL_MODE_SHIFT),
	.idleStatusReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_DDR),
	.adpllClkStatusMask =	CM_WKUP_CM_IDLEST_DPLL_DDR_ST_DPLL_CLK,
	.adpllClkStatusShift =	CM_WKUP_CM_IDLEST_DPLL_DDR_ST_DPLL_CLK_SHIFT,
	.adpllClkMNBypassStatusMask	 =	CM_WKUP_CM_IDLEST_DPLL_DDR_ST_MN_BYPASS,
	.adpllClkMNBypassStatusShift =	CM_WKUP_CM_IDLEST_DPLL_DDR_ST_MN_BYPASS_SHIFT,
	.adpllConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_DDR),
	.adpllBypassClkMask	=	CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_BYP_CLKSEL,
	.adpllBypassClkSource = (CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_BYP_CLKSEL_SEL0 <<
							CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_BYP_CLKSEL_SHIFT),
	.adpllMultiplierMask=	 CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT,

#ifdef EVM_13X13
	.adpllMultiplier	=	{(166 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (166 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (166 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (100 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT)},
	.adpllDivider		=	{(23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT)},
#endif
#ifdef EVM_15X15
	.adpllMultiplier	=	{(200 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (200 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (200 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT),
							 (100 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_MULT_SHIFT)},
	.adpllDivider		=	{(23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT),
							 (23 << CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV_SHIFT)},
#endif

	.adpllDividerMask	=	CM_WKUP_CM_CLKSEL_DPLL_DDR_DPLL_DIV,
	.adpllModeReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_DDR),

	.adpllLowPowerMask	=	(CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_LPMODE_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN),

	.adpllLowPowerNormalVal	=	((CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_LPMODE_EN_DISABLED << 	/* disable */
								CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_LPMODE_EN_SHIFT) |
							(CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_DPLL_LOCK_MODE <<  	/* lock mode. confirm if this is corrent */
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_SHIFT)),

	.adpllLowPowerBypassVal = ((CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_LPMODE_EN_ENABLED << 	/* Enable */
								CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_LPMODE_EN_SHIFT) |
							   (CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_DPLL_LP_BYP_MODE <<  	/* bypass mode */
								CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_EN_SHIFT)),

	.adpllRelockRampMask=	(CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RELOCK_RAMP_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_DRIFTGUARD_EN |
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RAMP_RATE |
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RAMP_LEVEL),

	.adpllRelockRampVal	=	(CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RELOCK_RAMP_EN | 		/* active logic is not clear */
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_DRIFTGUARD_EN	| 			/* drift guard is enabled */
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RAMP_RATE_REFCLKX2 |		/* ramp rate */
							(CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RAMP_LEVEL_RAMP_ALGO1 <<	/* ramp algo1 */
							CM_WKUP_CM_CLKMODE_DPLL_DDR_DPLL_RAMP_LEVEL_SHIFT)),

};

/************************* CLOCK's ********************************/
/*	Crystal which oscillates at 24MHz - commented variables to be renamed and added to clock defn	*/
Clock sysClkInClock = {
	.clkName			=	"sysClkInClock",
	.clockSpeedHz 		= 	CLK_EXT_CRYSTAL_SPEED,
//	.clockCtrlReg 		=	(SOC_CONTROL_REGS + CONTROL_DEEPSLEEP_CTRL),
//	.enableValue		=	(~CONTROL_DEEPSLEEP_CTRL_DSENABLE),
	/* disable is effective only if A8 and M3 hits WFI*/
//	.disableValue		=	(CONTROL_DEEPSLEEP_CTRL_DSENABLE),
//	.enDisActiveLogic	=	CLK_ACTIVE_LOW_LOGIC,
	.activeChildCount	=	0,
};

/*	Crystal which oscillates at 32KHz	*/
Clock clk32768Clock = {
	.clkName			=	"clk32768Clock",
	.clockSpeedHz 		= 	CLK_CLOCK_SPEED_32768_HZ,
	.activeChildCount	=	0,
};

/*	RC oscillator which generates 32KHz	- commented variables to be renamed and added to clock defn */
Clock clkRC32KClock = {
	.clkName			=	"clkRC32KClock",
	.clockSpeedHz 		= 	CLK_CLOCK_SPEED_32768_HZ,
//	.clockCtrlReg		=	(SOC_CONTROL_REGS + CONTROL_RCOSC_CTRL),
//	.enableValue		=	((~CONTROL_RCOSC_CTRL_STOPOSC) <<
								//CONTROL_RCOSC_CTRL_STOPOSC_SHIFT),
//	.disableValue		=	(CONTROL_RCOSC_CTRL_STOPOSC <<
//								CONTROL_RCOSC_CTRL_STOPOSC_SHIFT),
//	.enDisActiveLogic	=	CLK_ACTIVE_LOW_LOGIC,
	.activeChildCount	=	0,
};


/*	DPLL PER clock	*/
Clock dpllPerClock = {
	.clkName			=	"dpllPerClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.adpllPtr			=	&dpllPer,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};


/*	Clock dervied from PER PLL (24kHz -> 32kHz)	*/
/*	This clock has control to operate at opp50 - Control module	*/
Clock clk32KhzClock = {
	.clkName			=	"clk32KhzClock",
	.clockSpeedHz 		= 	CLK_CLOCK_SPEED_32768_HZ, /* Clock chapter */
	.parentClock		=	&dpllPerClock,
	.OPPSupported		= 	CLK_OPP_100,
	.activeChildCount	=	0,
};

/*	External clock input - optional	*/
Clock tclkinClock = {
	.clkName			=	"tclkinClock",
	/*	TBD- to be configured if an ext clock is connected	*/
	.clockSpeedHz 		= 	0,
	.activeChildCount	=	0,
};

/*	Clock out 1	- NOT USED AS OF NOW - commented code to be moved
	to corresponding module	*/
Clock clkout1Clock = {
	.clkName			=	"clkout1Clock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
//	.clockCtrlReg		=	(SOC_CONTROL_REGS + CONTROL_STATUS ),
//	.enableValue		=	(CONTROL_STATUS_SYSBOOT0_CLKOUT1_ENABLE <<
//							CONTROL_STATUS_SYSBOOT0_SHIFT),
//	.disableValue		=	((~CONTROL_STATUS_SYSBOOT0_CLKOUT1_ENABLE) <<
//							CONTROL_STATUS_SYSBOOT0_SHIFT),
	.parentClock		=	&sysClkInClock,
	.activeChildCount	=	0,
};

/*	CUST_EFUSE_SYSCLK	*/

/*	CORE_CLK_OUT	*/

/*	DPLL CORE clock	*/
Clock dpllCoreClock = {
	.clkName			=	"dpllCoreClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.adpllPtr			=	&dpllCore,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};
/**************************************************************/

/*	dpll Core M4 clock divider	*/
ClockDivider dpllCoreM4ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M4_DPLL_CORE),
	.clkDividerMask		=	CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIV,
	.clkDividerValue 	= 	{10, 10, 10, 1},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask	= CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL,
	.clkoutAutoGateCtrl = (CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_CLK_ENABLE <<
							CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_GATE_CTRL_SHIFT),
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M4_DPLL_CORE_ST_HSDIVIDER_CLKOUT1,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M4_DPLL_CORE_ST_HSDIVIDER_CLKOUT1_SHIFT,
	.isPdCtrlValid		=	true,
	.clkAutoPDCtrlMask	=	CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_PWDN,
	.clkAutoPDCtrl		=	(CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_PWDN_ALWAYS_ACTIVE <<
							CM_WKUP_CM_DIV_M4_DPLL_CORE_HSDIVIDER_CLKOUT1_PWDN_SHIFT),
};

/*	core_pll_M4_clk	*/
Clock corePllM4Clock = {
	.clkName			=	"corePllM4Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&dpllCoreClock,
	.clkDivider 		= 	&dpllCoreM4ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};

/*	SYSCLK1	*/
Clock sysclk1Clock = {
	.clkName			=	"sysclk1Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&corePllM4Clock,
	.activeChildCount	=	0,
};

/****************************************************************/

/*	DPLL MPU clock	*/
Clock dpllMpuClock = {
	.clkName			=	"dpllMpuClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.adpllPtr			=	&dpllMpu,
	.activeChildCount	=	0,
	.muxInputSelMask	=	CONTROL_PLL_CLKINPULOW_CTRL_MPU_DPLL_CLKINPULOW_SEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CONTROL_REGS + CONTROL_PLL_CLKINPULOW_CTRL),
	.muxInputSelVal		=	CONTROL_PLL_CLKINPULOW_CTRL_MPU_DPLL_CLKINPULOW_SEL,
	.OPPSupported		=	CLK_OPP_100,
};

/*	DBGSYSCLK	*/


/*	SR_SYSCLK	*/

/*	mcasp_fclk	*/

/*	can_clk	*/


/*	DPLL DISP clock	*/
Clock dpllDispClock = {
	.clkName			=	"dpllDispClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.adpllPtr			=	&dpllDisp,
	.activeChildCount	=	0,
	.muxInputSelMask	=	CONTROL_PLL_CLKINPULOW_CTRL_DISP_PLL_CLKINPULOW_SEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CONTROL_REGS + CONTROL_PLL_CLKINPULOW_CTRL),
	.muxInputSelVal		=	CONTROL_PLL_CLKINPULOW_CTRL_DISP_PLL_CLKINPULOW_SEL,
	.OPPSupported		=	CLK_OPP_100,
};

/*	VTP_CLK	*/

/*	DPLL DISP clock	*/
Clock dpllDdrClock = {
	.clkName			=	"dpllDdrClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.adpllPtr			=	&dpllDDR,
	.activeChildCount	=	0,
	.muxInputSelMask	=	CONTROL_PLL_CLKINPULOW_CTRL_DDR_PLL_CLKINPULOW_SEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CONTROL_REGS + CONTROL_PLL_CLKINPULOW_CTRL),
	.muxInputSelVal		=	CONTROL_PLL_CLKINPULOW_CTRL_DDR_PLL_CLKINPULOW_SEL,
	.OPPSupported		=	CLK_OPP_100,
};

ClockDivider dpllDdrM2ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DDR),
	.clkDividerMask		=	CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIV,
	.clkDividerValue	= 	{1,1,1,1},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask	=	CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_GATE_CTRL,
	.clkoutAutoGateCtrl = CM_WKUP_CM_DIV_M2_DPLL_DDR_DPLL_CLKOUT_GATE_CTRL,
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M2_DPLL_DDR_ST_DPLL_CLKOUT,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M2_DPLL_DDR_ST_DPLL_CLKOUT_SHIFT,
};

/*	ddr_pll_clk	*/
Clock ddrPllM2Clock = {
	.clkName			=	"ddrPllM2Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&dpllDdrClock,
	.clkDivider 		= 	&dpllDdrM2ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};

/*	CLK_32KHZ_TIMER	*/
Clock clk32KhzTimerClock = {
	.clkName			=	"clk32KhzTimerClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32KhzClock,
	.activeChildCount	=	0,
};

/**********************************************************************/
MuxParentClockSel timer2ClkMux = {
	.parentClock		 = 	{
								&tclkinClock,
								&sysClkInClock,
								&clk32KhzTimerClock,
							},
};

MuxParentClockSel timer3ClkMux = {
	.parentClock		 = 	{
								&tclkinClock,
								&sysClkInClock,
								&clk32KhzTimerClock,
							},
};

MuxParentClockSel timer4ClkMux = {
	.parentClock		 = 	{
								&tclkinClock,
								&sysClkInClock,
								&clk32KhzTimerClock,
							},
};

MuxParentClockSel timer7ClkMux = {
	.parentClock		 = 	{
								&tclkinClock,
								&sysClkInClock,
								&clk32KhzTimerClock,
							},
};

/*	TIMER2_CLK	*/
Clock timer2ClkClock 	= {
	.clkName			=	"timer2ClkClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER2_CLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_TIMER2_CLK_CLKSEL_CLK_M_OSC, /*	MOSC	*/
	.activeChildCount	=	0,
};

/*	TIMER3_CLK	*/
Clock timer3ClkClock 	= {
	.clkName			=	"timer3ClkClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER3_CLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_TIMER3_CLK_CLKSEL_CLK_M_OSC, /*	MOSC	*/
	.activeChildCount	=	0,
};

/*	TIMER4_CLK	*/
Clock timer4ClkClock 	= {
	.clkName			=	"timer4ClkClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER4_CLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_TIMER4_CLK_CLKSEL_CLK_M_OSC, /*	MOSC	*/
	.activeChildCount	=	0,
};

/* TIMER6_CLK */
Clock timer6ClkClock    = {
    .clkName            = "timer6ClkClock",
    .clockSpeedHz       = CLK_EXT_CRYSTAL_SPEED,
    .parentClock        = &sysClkInClock,
    .muxInputSelMask    = CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL,
    .muxInputSelReg     =
         (unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER6_CLK),
    /* MOSC */
    .muxInputSelVal     = CM_DPLL_CLKSEL_TIMER6_CLK_CLKSEL_SEL2,
    .activeChildCount = 0,
};

/*	TIMER7_CLK	*/
Clock timer7ClkClock 	= {
	.clkName			=	"timer7ClkClock",
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER7_CLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_TIMER7_CLK_CLKSEL_CLK_M_OSC, /*	MOSC	*/
	.activeChildCount	=	0,
};


/*	timer2_gclk	*/
Clock timer2GclkClock = {
	.clkName			=	"timer2GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer2ClkClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER2_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER2_GCLK_SHIFT,
};

/*	timer3_gclk	*/
Clock timer3GclkClock = {
	.clkName			=	"timer3GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer3ClkClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER3_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER3_GCLK_SHIFT,
};

/*	timer4_gclk	*/
Clock timer4GclkClock = {
	.clkName			=	"timer4GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer4ClkClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER4_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER4_GCLK_SHIFT,
};

/* timer6_gclk */
Clock timer6GclkClock   = {
    .clkName            =   "timer6GclkClock",
    .clkDomainPtr       =   &l4lsClkDomain,
    .clockSpeedHz       =   CLK_EXT_CRYSTAL_SPEED,
    .parentClock        =   &timer6ClkClock,
    .activeChildCount   =   0,
    .clkGateStatusReg   =   (unsigned int *)
                            (SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
    .clockGateStatusMask =  CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER6_GCLK,
    .clockGateStatusShift = CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER6_GCLK_SHIFT,
};

/*	timer7_gclk	*/
Clock timer7GclkClock = {
	.clkName			=	"timer7GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer7ClkClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER7_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER7_GCLK_SHIFT,
};

ClockDomain l4lsClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_CAN_CLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_2_GDBCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_3_GDBCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_LCDC_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER2_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER3_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER4_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER5_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER6_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_TIMER7_GCLK |
								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK),
	.clkList			= 	{
								&timer2GclkClock,
								&timer3GclkClock,
								&timer4GclkClock,
								&timer7GclkClock,
								&lcdcGclkClock,
								&gpio3GclkClock,
								&i2cGclkClock,
								&spiGclkClock,
								&uartGclkClock,
                                &timer6GclkClock,
                            }
};



/*	timer2 functioanl clock	*/
Clock timer2FclkClock = {
	.clkName			=	"timer2FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer2GclkClock,
	.activeChildCount	=	0,
};


/*	timer3 functioanl clock	*/
Clock timer3FclkClock = {
	.clkName			=	"timer3FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer3GclkClock,
	.activeChildCount	=	0,
};


/*	timer4 functioanl clock	*/
Clock timer4FclkClock = {
	.clkName			=	"timer4FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer4GclkClock,
	.activeChildCount	=	0,
};

/* timer6 functioanl clock */
Clock timer6FclkClock = {
    .clkName            =   "timer6FclkClock",
    .isLeafClock        =   true,
    .clockSpeedHz       =   CLK_EXT_CRYSTAL_SPEED,
    .parentClock        =   &timer6GclkClock,
    .activeChildCount   =   0,
};

/*	timer7 functioanl clock	*/
Clock timer7FclkClock = {
	.clkName			=	"timer7FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&timer7GclkClock,
	.activeChildCount	=	0,
};


/*	Timer1 Gclk clock	*/
Clock timer1GclkClock = {
	.clkName			=	"timer1GclkClock",
	.isLeafClock		=	true,
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32768Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_TIMER1_GCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_TIMER1_GCLK_SHIFT,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_TIMER1MS_CLK),
	.muxInputSelMask	=	CM_DPLL_CLKSEL_TIMER1MS_CLK_CLKSEL,
	.muxInputSelVal		=	(CM_DPLL_CLKSEL_TIMER1MS_CLK_CLKSEL_SEL5 << CM_DPLL_CLKSEL_TIMER1MS_CLK_CLKSEL_SHIFT),

};



/*	timer1 functioanl clock	*/
Clock timer1FclkClock = {
	.clkName			=	"timer1FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&timer1GclkClock,
	.activeChildCount	=	0,
};

/*	TIMER1_GCLK	*/
/**************************************************************/

ClockDomain rtcClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL),
	.clkStateTransValue	=	(CM_RTC_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_RTC_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_RTC_CLKSTCTRL_CLKACTIVITY_L4_RTC_GCLK |
							CM_RTC_CLKSTCTRL_CLKACTIVITY_RTC_32KCLK),
	.clkList			= 	{
								&rtc32kGclkClock,
							}
};

/*	rtc_32kClk - gclk	*/
Clock rtc32kGclkClock = {
	.clkName			=	"rtc32kGclkClock",
	.clkDomainPtr		=	&rtcClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32KhzTimerClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL),
	.clockGateStatusMask =	CM_RTC_CLKSTCTRL_CLKACTIVITY_RTC_32KCLK,
	.clockGateStatusShift =	CM_RTC_CLKSTCTRL_CLKACTIVITY_RTC_32KCLK_SHIFT,
};

/*	rtc 32k functional clock	*/
Clock rtcFclkClock = {
	.clkName			=	"rtcFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&rtc32kGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/

/*	dpll per M2 divider	*/
ClockDivider dpllPerM2ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_PER),
	.clkDividerMask		=	CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIV,
	.clkDividerValue	= 	{5,5,5,5},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask = CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_GATE_CTRL,
	.clkoutAutoGateCtrl = CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_GATE_CTRL,
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M2_DPLL_PER_ST_DPLL_CLKOUT,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M2_DPLL_PER_ST_DPLL_CLKOUT_SHIFT,
};

/*	PER dPll M2 clock	*/
Clock clk192MhzClock = {
	.clkName			=	"clk192MhzClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ,
	.parentClock		=	&dpllPerClock,
	.clkDivider 		= 	&dpllPerM2ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};

/*	dpll disp M2 divider	*/
ClockDivider dpllDispM2ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_DISP),
	.clkDividerMask 	= 	CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIV,
	.clkDividerValue	= 	{1,1,1,1},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask = CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_GATE_CTRL,
	.clkoutAutoGateCtrl = CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_GATE_CTRL,
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M2_DPLL_DISP_ST_DPLL_CLKOUT,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M2_DPLL_DISP_ST_DPLL_CLKOUT_SHIFT,
};

/*	DISP dpll M2 clock	*/
Clock clkDispPllM2Clock = {
	.clkName			=	"clkDispPllM2Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_600_MHZ,
	.parentClock		=	&dpllDispClock,
	.clkDivider 		= 	&dpllDispM2ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};


/**************************************************************/
MuxParentClockSel wdt1ClkMux = {
	.parentClock		= 	{
								&clkRC32KClock,
								&clk32KhzTimerClock,
							},
};

/*	WDT1_CLK - mux o/p	*/
Clock wdt1ClkClock = {
	.clkName			=	"wdt1ClkClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32KhzTimerClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_WDT1_CLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_WDT1_CLK_CLKSEL_CLK32768, /*	32khz timer	*/
	.activeChildCount	=	0,
};

/*	wakeup clock domain	*/
ClockDomain wkupClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clkStateTransValue	=	(CM_WKUP_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_WKUP_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(	CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_SR_SYSCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_TIMER0_GCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_TIMER1_GCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_WDT0_GCLK |
								CM_WKUP_CLKSTCTRL_CLKACTIVITY_WDT1_GCLK
							),
	.clkList			= 	{
								&wdt1GclkClock,
								&adcFclkClock,
								&l4WkupGclkClock,
								&uart0GclkClock,
								&i2c0GclkClock,
								&gpio0GclkClock,
								&timer1GclkClock,
							},
};

/*	wdt1_gclk	*/
Clock wdt1GclkClock = {
	.clkName			=	"wdt1GclkClock",
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&wdt1ClkClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_WDT1_GCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_WDT1_GCLK_SHIFT,
};

/*	wdt1 functioanl clock	*/
Clock wdt1FclkClock = {
	.clkName			=	"wdt1FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&wdt1GclkClock,
	.activeChildCount	=	0,
};

/*	wdt0	*/

/**************************************************************/
/* can clk*/

/**************************************************************/

/*	Per L3 clock domain	*/
ClockDomain perL3ClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
								CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK |
								//CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK |
								CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK |
								CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK |
								CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MMC_FCLK
							),
	.clkList			= 	{
								&mcaspGclkClock,
								&cpswCptsRftGclkClock,
								&emifGclkClock,
								&l3GclkClock,
							}
};

/*	mcasp_gclk	*/
Clock mcaspGclkClock = {
	.clkName			=	"mcaspGclkClock",
	.clkDomainPtr		=	&perL3ClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK,
	.clockGateStatusShift =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK_SHIFT,
};

/*	mcasp1 functioanl clock	*/
Clock mcasp1FclkClock = {
	.clkName			=	"mcasp1FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&mcaspGclkClock,
	.activeChildCount	=	0,
};

/******************************************************************/

ClockDivider dpllPerClkdcoldoDivider = {
	.dividerConfigReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKDCOLDO_DPLL_PER),
	.clkoutAutoGateCtrlMask =	CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_GATE_CTRL,
	.clkoutAutoGateCtrl 	= 	CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_GATE_CTRL,
	.clkoutGateStatusMask 	= 	CM_WKUP_CM_CLKDCOLDO_DPLL_PER_ST_DPLL_CLKDCOLDO,
	.clkoutGateStatusShift 	= 	CM_WKUP_CM_CLKDCOLDO_DPLL_PER_ST_DPLL_CLKDCOLDO_SHIFT,
	.clkAutoPDCtrlMask		=	CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_PWDN,
	.isPdCtrlValid			=	true,
	.clkAutoPDCtrl			= 	(CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_PWDN_ALWAYS_ACTIVE <<
								CM_WKUP_CM_CLKDCOLDO_DPLL_PER_DPLL_CLKDCOLDO_PWDN_SHIFT),
};

/*	per_pll_CLKDCOLDO_clk	*/
Clock perPllClkdcoldoClock = {
	.clkName			=	"perPllClkdcoldoClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_960_MHZ,
	.parentClock		=	&dpllPerClock,
	.clkDivider 		= 	&dpllPerClkdcoldoDivider,
	.activeChildCount	=	0,
};

/*	USB_PLL_CLK	*/
Clock usbPllclkClock = {
	.clkName			=	"usbPllclkClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_960_MHZ,
	.parentClock		=	&perPllClkdcoldoClock,
	.activeChildCount	=	0,
};

/*	SR_SYSCLK	*/

/***********************************************************************/

/*	L4_RTC_GCLK Interface clock	*/
Clock l4RtcGclkIclkClock = {
	.clkName			=	"l4RtcGclkIclkClock",
	.isLeafClock		=	true,
	.clkDomainPtr		=	&rtcClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ/2,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_RTC_REGS + CM_RTC_CLKSTCTRL),
	.clockGateStatusMask =	CM_RTC_CLKSTCTRL_CLKACTIVITY_L4_RTC_GCLK,
	.clockGateStatusShift =	CM_RTC_CLKSTCTRL_CLKACTIVITY_L4_RTC_GCLK_SHIFT,
};

Clock rtcIclkClock = {
	.clkName			=	"rtcIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4RtcGclkIclkClock,
	.activeChildCount	=	0,
};

/*	GFX_FCLK - MUX	*/

/***********************************************************************/
/*	ADC_FCLK	*/
Clock adcFclkClock = {
	.clkName			=	"adcFclkClock",
	.isLeafClock		=	true,
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED,
	.parentClock		=	&sysClkInClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask = 	CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK,
	.clockGateStatusShift = CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK_SHIFT,
};

/***********************************************************************/
/*	L4_WKUP_GCLK	*/
Clock l4WkupGclkClock = {
	.clkName			=	"l4WkupGclkClock",
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ/2,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK_SHIFT,
};

/*	ADC_TSC_ICLK	*/
Clock adcTscIclkClock = {
	.clkName			=	"adcTscIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};

/*	WDT1_ICLK	*/
Clock wdt1IclkClock = {
	.clkName			=	"wdt1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};

/*	UART0_ICLK	*/
Clock uart0IclkClock = {
	.clkName			=	"uart0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/

/*	L4_WKUP_AON_GCLK clock domain	*/
ClockDomain l4WkupAonClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL),
	.clkStateTransValue	=	(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK),
	.clkList			= 	{
								&l4WkupAonGclkClock,
							},
};

/*	L4_WKUP_AON_GCLK	*/
Clock l4WkupAonGclkClock = {
	.clkName			=	"l4WkupAonGclkClock",
	.clkDomainPtr		=	&l4WkupAonClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ/2,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK,
	.clockGateStatusShift =	CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK_SHIFT,
};

/*	WKUP_M3	*/
Clock wkupM3IclkClock = {
	.clkName			=	"wkupM3IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupAonGclkClock,
	.activeChildCount	=	0,
};

/*	L4WKUP	*/
Clock l4WkupIclkClock = {
	.clkName			=	"l4WkupIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupAonGclkClock,
	.activeChildCount	=	0,
};

/*	DEBUG_CLKA_GCLK	*/
/*	l3_aon_gclk	*/
/*	STD_EFUSE_SYSCLK	*/
/***********************************************************************/

/*	mpu clock domain	*/
ClockDomain mpuClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_MPU_REGS + CM_MPU_CLKSTCTRL),
	.clkStateTransValue	=	(CM_MPU_CLKSTCTRL_CLKTRCTRL_SW_SLEEP <<
							CM_MPU_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_MPU_CLKSTCTRL_CLKACTIVITY_MPU_CLK),
	.clkList			 =	{
								&mpuIclkClock,
							}
};

ClockDivider dpllMpuM2ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU),
	.clkDividerMask		=	CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV,
	.clkDividerValue	= 	{1, 1, 1, 1},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask = CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_GATE_CTRL,
	.clkoutAutoGateCtrl = (CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_GATE_CTRL_CLK_ENABLE <<
							CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_GATE_CTRL_SHIFT),
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M2_DPLL_MPU_ST_DPLL_CLKOUT,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M2_DPLL_MPU_ST_DPLL_CLKOUT_SHIFT,
};

/*	mpu_pll_M2_clk	*/
Clock mpuPllM2Clock = {
	.clkName			=	"mpuPllM2Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_550_MHZ,
	.parentClock		=	&dpllMpuClock,
	.clkDivider 		=	&dpllMpuM2ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};

/*	mpu_clk	*/
Clock mpuIclkClock = {
	.clkName			=	"mpuIclkClock",
	.isLeafClock		=	true,
	.clkDomainPtr		=	&mpuClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_550_MHZ,
	.parentClock		=	&mpuPllM2Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_MPU_REGS + CM_MPU_CLKSTCTRL),
	.clockGateStatusMask = 	CM_MPU_CLKSTCTRL_CLKACTIVITY_MPU_CLK,
	.clockGateStatusShift = CM_MPU_CLKSTCTRL_CLKACTIVITY_MPU_CLK_SHIFT,
};

/***********************************************************************/

/*	dpll Core M5 clock divider	*/
ClockDivider dpllCoreM5ClkDivider = {
	.dividerConfigReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M5_DPLL_CORE),
	.clkDividerMask		=	CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIV,
	.clkDividerValue	= 	{8,8,8,1},
	.clkDivUpdatedStatusMask = CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK,
	.clkDivUpdatedStatusShift = CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_DIVCHACK_SHIFT,
	.clkoutAutoGateCtrlMask = CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL,
	.clkoutAutoGateCtrl = (CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_CLK_ENABLE <<
							CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_GATE_CTRL_SHIFT),
	.clkoutGateStatusMask = CM_WKUP_CM_DIV_M5_DPLL_CORE_ST_HSDIVIDER_CLKOUT2,
	.clkoutGateStatusShift = CM_WKUP_CM_DIV_M5_DPLL_CORE_ST_HSDIVIDER_CLKOUT2_SHIFT,
	.clkAutoPDCtrlMask	=	CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_PWDN,
	.isPdCtrlValid		=	true,
	.clkAutoPDCtrl		=	(CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_PWDN_ALWAYS_ACTIVE <<
							CM_WKUP_CM_DIV_M5_DPLL_CORE_HSDIVIDER_CLKOUT2_PWDN_SHIFT),
};

/*	core_pll_M5_clk	*/
Clock corePllM5Clock = {
	.clkName			=	"corePllM5Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ,
	.parentClock		=	&dpllCoreClock,
	.clkDivider 		= 	&dpllCoreM5ClkDivider,
	.activeChildCount	=	0,
	.OPPSupported		=	CLK_OPP_100,
};


/*	Sysclk2	*/
Clock sysclk2Clock = {
	.clkName			=	"sysclk2Clock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ,
	.parentClock		=	&corePllM5Clock,
	.activeChildCount	=	0,
};

/***********************************************************************/

/*	CPSW_CPTS_RFT_CLK - Mux	*/
MuxParentClockSel cpswCptsRftClkMux = {
	.parentClock		= 	{
								&sysclk2Clock,
								&sysclk1Clock,
							},
};

Clock cpswCptsRftMuxClock = {
	.clkName			=	"cpswCptsRftMuxClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&sysclk1Clock,
	.muxInputSelMask	=	CM_DPLL_CM_CPTS_RFT_CLKSEL_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CM_CPTS_RFT_CLKSEL),
	.muxInputSelVal		=	(CM_DPLL_CM_CPTS_RFT_CLKSEL_CLKSEL_SYSCLK1 <<
							CM_DPLL_CM_CPTS_RFT_CLKSEL_CLKSEL_SHIFT),
	.activeChildCount	=	0,
};

/*
	clock comain
	CPSW_CPTS_RFT_GCLK
	CPSW_CPTS_RFT_FCLK

	Other GMAC clocks
*/
/***********************************************************************/
/*	GFX_L3_GCLK	*/
/*	sysclkout_pre	*/
/***********************************************************************/

/*	LCDC - Mux	*/
MuxParentClockSel lcdClkMux = {
	.parentClock = {
											&clkDispPllM2Clock,
											&sysclk2Clock,
											&clk192MhzClock,
										},
};

/*	LCDC Mux clock	*/
Clock lcdcMuxClock = {
	.clkName			=	"lcdcMuxClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_600_MHZ,
	//.parentClock		=	&clkDispPllM2Clock,
	.parentClock		=	&clk192MhzClock,
	.muxInputSelMask	=	CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_LCDC_PIXEL_CLK),
	.muxInputSelVal		=	(CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_PER_PLL <<
							CM_DPLL_CLKSEL_LCDC_PIXEL_CLK_CLKSEL_SHIFT),
	.activeChildCount	=	0,
};

/*	LCDC_GCLK	*/
Clock lcdcGclkClock = {
	.clkName			=	"lcdcGclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_600_MHZ,
	.parentClock		=	&lcdcMuxClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_LCDC_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_LCDC_GCLK_SHIFT,
};

/*	LCDC Fclk	*/
Clock lcdcFclkClock = {
	.clkName			=	"lcdcFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_600_MHZ,
	.parentClock		=	&lcdcGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/
/*	LCDC clock domain	*/
ClockDomain lcdcClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_LCDC_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_LCDC_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_LCDC_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L3_OCP_GCLK |
							CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L4_OCP_GCLK),
	.clkList			=	{
								&lcdcL3IclkClock,
								&lcdcL4IclkClock,
							}
};

/*	LCD L3 Gclk	*/
Clock lcdL3GclkClock = {
	.clkName			=	"lcdL3GclkClock",
	.clkDomainPtr		=	&lcdcClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_LCDC_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L3_OCP_GCLK,
	.clockGateStatusShift =	CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L3_OCP_GCLK_SHIFT,
};

/*	LCDC L3 Interface clock	*/
Clock lcdcL3IclkClock = {
	.clkName			=	"lcdcL3IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&lcdL3GclkClock,
	.activeChildCount	=	0,
};

/*	LCDC L4 Interface clock	*/
Clock lcdcL4IclkClock = {
	.clkName			=	"lcdcL4IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&lcdL3GclkClock,
	.activeChildCount	=	0,
};

/*	LCD L4s Gclk	*/
Clock lcdL4sGclkClock = {
	.clkName			=	"lcdL4sGclkClock",
	.clkDomainPtr		=	&lcdcClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&core100MClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_LCDC_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L4_OCP_GCLK,
	.clockGateStatusShift =	CM_PER_LCDC_CLKSTCTRL_CLKACTIVITY_LCDC_L4_OCP_GCLK_SHIFT,
};

/*	lcdc L4 clock enable */
Clock lcdcL4ClkEnableClock = {
	.clkName			=	"lcdcL4ClkEnableClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&lcdL4sGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/
/*	DMA_L3_GCLK			*/
/*	L4HS_GCLK			*/
/*	ICSS_IEP_GCLK		*/
/*	L3_INSTR			*/
/*	MSTR_EXPS			*/
/*	PCIE				*/
/*	SLV_EXPS			*/
/*	TPCC                */
/*	TPTC0               */
/*	TPTC1               */
/*	TPTC2               */
/*	mlb_shb_ocp_clk     */
/*	mlb_spb_ocp_clk     */
/*	mlb_sys_ocp_clk     */
/*	ICSS_OCP_GCLK MUX   */
/*	AES0                */
/*	AES1                */
/*	SHA0                */
/*	sysclkout_pre MUX   */
/***********************************************************************/

/*	L3_GCLK	*/
Clock l3GclkClock = {
	.clkName			=	"l3GclkClock",
	.clkDomainPtr		=	&perL3ClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK,
	.clockGateStatusShift =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_SHIFT,
};

/*	L3 Interface clock	*/
Clock l3IclkClock = {
	.clkName			=	"l3IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/*	OCMCRAM Interface clock	*/
Clock ocmcramIclkClock = {
	.clkName			=	"ocmcramIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/*	TPCC Interface clock	*/
Clock tpccIclkClock = {
	.clkName			=	"tpccIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/*	TPTC0 Interface clock	*/
Clock tptc0IclkClock = {
	.clkName			=	"tptc0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/*	TPTC1 Interface clock	*/
Clock tptc1IclkClock = {
	.clkName			=	"tptc1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/*	TPTC2 Interface clock	*/
Clock tptc2IclkClock = {
	.clkName			=	"tptc2IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};


/*	EMIF Interface clock	*/
Clock emifIclkClock = {
	.clkName			=	"emifIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&l3GclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/
/*	L4_CEFUSE_GCLK	*/
/*	L4LS_GFX_GCLK   */
/*	DMA_L4S_GCLK    */

/***********************************************************************/
/*	core 100M	*/
Clock core100MClock = {
	.clkName			=	"core100MClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&sysclk1Clock,
	.activeChildCount	=	0,
};

/***********************************************************************/

/*	L4FW clock domain	*/
ClockDomain l4fwClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4FW_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_L4FW_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_L4FW_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_PER_L4FW_CLKSTCTRL_CLKACTIVITY_L4FW_GCLK),
	.clkList			= 	{
								&l4fwGclkClock,
							},
};

/*	l4fw_gclk	*/
Clock l4fwGclkClock = {
	.clkName			=	"l4fwGclkClock",
	.clkDomainPtr		=	&l4fwClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&core100MClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4FW_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4FW_CLKSTCTRL_CLKACTIVITY_L4FW_GCLK,
	.clockGateStatusShift =	CM_PER_L4FW_CLKSTCTRL_CLKACTIVITY_L4FW_GCLK_SHIFT,
};

/*	L4FW Interconnect	*/
Clock l4fwIclkClock = {
	.clkName			=	"l4fwIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4fwGclkClock,
	.activeChildCount	=	0,
};

/*	EMIF_FW Interconnect - NOT USED AS OF NOW - commented code to be moved
	to corresponding module	*/
Clock emifFwClock = {
	.clkName			=	"emifFwClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
//	.clockCtrlReg		=	(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL),
//	.enableValue		=	(CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_ENABLE <<
//							CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_SHIFT),
//	.disableValue		=	(CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_DISABLE <<
//							CM_PER_EMIF_FW_CLKCTRL_MODULEMODE_SHIFT),
	.parentClock		=	&l4fwGclkClock,
	.activeChildCount	=	0,

//	.moduleStatusReg	=	(SOC_CM_PER_REGS + CM_PER_EMIF_FW_CLKCTRL),
//	.idleStatusMask		=	CM_PER_EMIF_FW_CLKCTRL_IDLEST,
//	.idleStatusShift	=	CM_PER_EMIF_FW_CLKCTRL_IDLEST_SHIFT,
};

/***********************************************************************/
/*	L3S	clock domain*/
ClockDomain l3sClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK),
	.clkList			= 	{
								&l3sGclkClock,
							},
};

/*	l3s_gclk	*/
Clock l3sGclkClock = {
	.clkName			=	"l3sGclkClock",
	.clkDomainPtr		=	&l3sClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&core100MClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK,
	.clockGateStatusShift =	CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK_SHIFT,
};

/*	USB0 Interface Clock	*/
Clock usb0IclkClock = {
	.clkName			=	"usb0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l3sGclkClock,
	.activeChildCount	=	0,
};

/*	GPMC Interface Clock	*/
Clock gpmcIclkClock = {
	.clkName			=	"gpmcIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l3sGclkClock,
	.activeChildCount	=	0,
};

/* L3 interconnect -> L4_EN */
Clock l3l4IcEnableClock = {
	.clkName			=	"l3l4IcEnableClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l3sGclkClock,
	.activeChildCount	=	0,
};

/*	MCASP1 Interface clock	*/
Clock mcasp1IclkClock = {
	.clkName			=	"mcasp1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l3sGclkClock,
	.activeChildCount	=	0,
};

/*	IEEE5000	*/
/*	MCASP0	*/
/*	MCASP2	*/
/*	MMC2	*/

/***********************************************************************/

/*	L4LS_GCLK	*/
Clock l4lsGclkClock = {
	.clkName			=	"l4lsGclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&core100MClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK_SHIFT,
};

/*	L4LS Interconnect interface clock	*/
Clock l4lsIclkClock = {
	.clkName			=	"l4lsIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	MAILBOX0 Interface clock	*/
Clock mailbox0IclkClock = {
	.clkName			=	"mailbox0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	EPWMSS0 Interface clock	*/
Clock epwmss0IclkClock = {
	.clkName			=	"epwmss0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	GPIO3 Interface clock	*/
Clock gpio3IclkClock = {
	.clkName			=	"gpio3IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};


/*	GPIO1 Interface clock	*/
Clock gpio1IclkClock = {
	.clkName			=	"gpio1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	GPIO0_GDBCLK	*/

/*	SPI0 Interface clock	*/
Clock spi0IclkClock = {
	.clkName			=	"spi0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,

	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	I2C1 Interface clock	*/
Clock i2c1IclkClock = {
	.clkName			=	"i2c1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	I2C0 Interface clock	*/
Clock i2c0IclkClock = {
	.clkName			=	"i2c0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};


/*	GPIO0 Interface clock	*/
Clock gpio0IclkClock = {
	.clkName			=	"gpio0IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};


/*	TIMER2 Interface clock	*/
Clock timer2IclkClock = {
	.clkName			=	"timer2IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	TIMER3 Interface clock	*/
Clock timer3IclkClock = {
	.clkName			=	"timer3IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	TIMER4 Interface clock	*/
Clock timer4IclkClock = {
	.clkName			=	"timer4IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*TIMER6 Interface clock*/
Clock timer6IclkClock  = {
    .clkName           =    "timer6IclkClock",
    .isLeafClock       =    true,
    .clockSpeedHz      =    CLK_CLOCK_SPEED_100_MHZ,
    .parentClock       =    &l4lsGclkClock,
    .activeChildCount = 0,
};

/*	TIMER7 Interface clock	*/
Clock timer7IclkClock = {
	.clkName			=	"timer7IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_100_MHZ,
	.parentClock		=	&l4lsGclkClock,
	.activeChildCount	=	0,
};

/*	TIMER1 Interface clock	*/
Clock timer1IclkClock = {
	.clkName			=	"timer1IclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_EXT_CRYSTAL_SPEED/2,
	.parentClock		=	&l4WkupGclkClock,
	.activeChildCount	=	0,
};


/*	DCAN0		*/
/*	DCAN1       */
/*	MMC0        */
/*	MMC1        */
/*	PKA         */
/*	RNG         */
/*	ELM         */
/*	EPWMSS0     */
/*	EPWMSS1     */
/*	EPWMSS2     */
/*	OCPWP       */
/*	SPARE0      */
/*	SPARE1      */
/*	SPINLOCK    */
/*	UART1       */
/*	UART2       */
/*	UART3       */
/*	UART4       */
/*	UART5       */
/*	SPI1        */
/*	GPIO1       */
/*	GPIO2       */
/*	GPIO4       */
/*	GPIO5       */
/*	GPIO6       */
/*	I2C2        */
/*	TIMER3      */
/*	TIMER4      */
/*	TIMER5      */
/*	TIMER6      */
/*	TIMER7      */
/***********************************************************************/

/*	CPSW_125MHZclock domain	*/
ClockDomain cpsw125MhzClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_CPSW_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_CPSW_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_CPSW_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_PER_CPSW_CLKSTCTRL_CLKACTIVITY_CPSW_125MHZ_GCLK),
	.clkList			= 	{
								&cpsw125MhzGclkClock,
							},
};

/*	CPSW_125MHZ_GCLK	*/
Clock cpsw125MhzGclkClock = {
	.clkName			=	"cpsw125MhzGclkClock",
	.clkDomainPtr		=	&cpsw125MhzClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/2,
	.parentClock		=	&sysclk2Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_CPSW_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_CPSW_CLKSTCTRL_CLKACTIVITY_CPSW_125MHZ_GCLK,
	.clockGateStatusShift =	CM_PER_CPSW_CLKSTCTRL_CLKACTIVITY_CPSW_125MHZ_GCLK_SHIFT,
};

/*	CPSW_125MHZ Interface clock	*/
Clock cpsw125MhzIclkClock = {
	.clkName			=	"cpsw125MhzIclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/2,
	.parentClock		=	&cpsw125MhzGclkClock,
	.activeChildCount	=	0,
};
/***********************************************************************/

/*	CPSW mux clock -though there is no HW mux this node is treated as mux just
	for the ease of programming	*/
Clock cpswMuxClock = {
	.clkName			=	"cpswMuxClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/5,
	.parentClock		=	&sysclk2Clock,
	.muxInputSelMask	=	CM_DPLL_CM_MAC_CLKSEL_MII_CLK_SEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CM_MAC_CLKSEL),
	.muxInputSelVal		=	(CM_DPLL_CM_MAC_CLKSEL_MII_CLK_SEL_SYSCLK2_DIV5 <<
							CM_DPLL_CM_MAC_CLKSEL_MII_CLK_SEL_SHIFT),
	.activeChildCount	=	0,
};
/***********************************************************************/
/*	L4HS clock domain	*/
ClockDomain l4hsClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4HS_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_L4HS_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_L4HS_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	( CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_250MHZ_GCLK |
							  CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_50MHZ_GCLK |
							  CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_5MHZ_GCLK |
							  CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_L4HS_GCLK),
	.clkList			= 	{
								&cpsw5MhzGclkClock,
							},
};

/*	CPSW_5MHZ_GCLK	*/
Clock cpsw5MhzGclkClock = {
	.clkName			=	"cpsw5MhzGclkClock",
	.clkDomainPtr		=	&l4hsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/50,
	.parentClock		=	&cpswMuxClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4HS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_5MHZ_GCLK,
	.clockGateStatusShift =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_5MHZ_GCLK_SHIFT,
};

/*	CPSW_5MHZ Functional clock	*/
Clock cpsw5MhzFclkClock = {
	.clkName			=	"cpsw5MhzFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/50,
	.parentClock		=	&cpsw5MhzGclkClock,
	.activeChildCount	=	0,
};

/*	CPSW_50MHZ_GCLK	*/
Clock cpsw50MhzGclkClock = {
	.clkName			=	"cpsw50MhzGclkClock",
	.clkDomainPtr		=	&l4hsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/5,
	.parentClock		=	&cpswMuxClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4HS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_50MHZ_GCLK,
	.clockGateStatusShift =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_50MHZ_GCLK_SHIFT,
};

/*	CPSW_50MHZ Functional clock	*/
Clock cpsw50MhzFclkClock = {
	.clkName			=	"cpsw50MhzFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ/5,
	.parentClock		=	&cpsw50MhzGclkClock,
	.activeChildCount	=	0,
};

/*	CPSW_250MHZ_GCLK	*/
Clock cpsw250MhzGclkClock = {
	.clkName			=	"cpsw250MhzGclkClock",
	.clkDomainPtr		=	&l4hsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ,
	.parentClock		=	&sysclk2Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4HS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_250MHZ_GCLK,
	.clockGateStatusShift =	CM_PER_L4HS_CLKSTCTRL_CLKACTIVITY_CPSW_250MHZ_GCLK_SHIFT,
};

/*	CPSW_250MHZ Functional clock	*/
Clock cpsw250MhzFclkClock = {
	.clkName			=	"cpsw250MhzFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_250_MHZ,
	.parentClock		=	&cpsw250MhzGclkClock,
	.activeChildCount	=	0,
};

/*	CPSW_CPTS_RFT_GCLK	*/
Clock cpswCptsRftGclkClock = {
	.clkName			=	"cpswCptsRftGclkClock",
	.clkDomainPtr		=	&perL3ClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&cpswCptsRftMuxClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK,
	.clockGateStatusShift =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK_SHIFT,
};

/*	CPSW_CPTS_RFT Functional clock	*/
Clock cpswCptsFclkClock = {
	.clkName			=	"cpswCptsFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ,
	.parentClock		=	&cpswCptsRftGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/
/*	EMIF Gclk	*/
Clock emifGclkClock = {
	.clkName			=	"emifGclkClock",
	.clkDomainPtr		=	&perL3ClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ/2,
	.parentClock		=	&ddrPllM2Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK,
	.clockGateStatusShift =	CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_SHIFT,
};

/*	EMIF Functional Clock	*/
Clock emifFclkClock = {
	.clkName			=	"emifFclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_200_MHZ/2,
	.parentClock		=	&emifGclkClock,
	.activeChildCount	=	0,
};

/***********************************************************************/
/*	ICSS UART GCLK	*/
/***********************************************************************/
/*	GPIO_1_GDBCLK	*/
Clock gpio1GclkClock = {
	.clkName			=	"gpio1GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32KhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK_SHIFT,
};


/*	GPIO_2_GDBCLK   */
/*	GPIO_4_GDBCLK   */
/*	GPIO_5_GDBCLK   */
/*	GPIO_6_GDBCLK	*/

/*	GPIO_3_GDBCLK   */
Clock gpio3GclkClock = {
	.clkName			=	"gpio3GclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32KhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_3_GDBCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_3_GDBCLK_SHIFT,
};


Clock gpio3OptClock = {
	.clkName			=	"gpio3OptClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&gpio3GclkClock,
	.activeChildCount	=	0,
};


Clock gpio1OptClock = {
	.clkName			=	"gpio1OptClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&gpio1GclkClock,
	.activeChildCount	=	0,
};
/***********************************************************************/

/*	CLK_24MHZ Clock domain	*/
ClockDomain clk24MhzClkDomain = {
	.isCDInitialized	=	false,
	.activeClockCount	=	0,
	.clkStateTransCtrlReg = (unsigned int *)(SOC_CM_PER_REGS + CM_PER_CLK_24MHZ_CLKSTCTRL),
	.clkStateTransValue	=	(CM_PER_CLK_24MHZ_CLKSTCTRL_CLKTRCTRL_NO_SLEEP <<
							CM_PER_CLK_24MHZ_CLKSTCTRL_CLKTRCTRL_SHIFT),
	.clkGateStatusMask	=	(CM_PER_CLK_24MHZ_CLKSTCTRL_CLKACTIVITY_CLK_24MHZ_GCLK),
	.clkList			= 	{
								&clk24MhzGclkClock,
							},
};

/*	CLK_24MHZ Gclk	*/
Clock clk24MhzGclkClock = {
	.clkName			=	"clk24MhzGclkClock",
	.clkDomainPtr		=	&clk24MhzClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/8,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_CLK_24MHZ_CLKSTCTRL_CLKACTIVITY_CLK_24MHZ_GCLK,
	.clockGateStatusShift =	CM_PER_CLK_24MHZ_CLKSTCTRL_CLKACTIVITY_CLK_24MHZ_GCLK_SHIFT,
};

/*	CLKDIV32K Interface clock - NOT USED AS OF NOW - commented code to be moved
	to corresponding module	*/
Clock clkdiv32KIclkCLock = {
	.clkName			=	"clkdiv32KIclkCLock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/8,
//	.clockCtrlReg		=	(SOC_CM_PER_REGS + CM_PER_CLKDIV32K_CLKCTRL),
//	.enableValue		=	(CM_PER_CLKDIV32K_CLKCTRL_MODULEMODE_ENABLE <<
//							CM_PER_CLKDIV32K_CLKCTRL_MODULEMODE_SHIFT),
//	.disableValue		=	(CM_PER_CLKDIV32K_CLKCTRL_MODULEMODE_DISABLE <<
//							CM_PER_CLKDIV32K_CLKCTRL_MODULEMODE_SHIFT),
	.parentClock		=	&clk24MhzGclkClock,
	.activeChildCount	=	0,
//	.moduleStatusReg	=	(SOC_CM_PER_REGS + CM_PER_CLKDIV32K_CLKCTRL),
//	.idleStatusMask		=	CM_PER_CLKDIV32K_CLKCTRL_IDLEST,
//	.idleStatusShift	=	CM_PER_CLKDIV32K_CLKCTRL_IDLEST_SHIFT,
};

/***********************************************************************/
/*	i2c_fclk Gclk	*/
Clock i2cGclkClock = {
	.clkName			=	"i2cGclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK_SHIFT,
};

/*	i2c1 Functional Clock	*/
Clock i2c1FclkClock = {
	.clkName			=	"i2c1FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&i2cGclkClock,
	.activeChildCount	=	0,
};


/*	I2C0_GFCLK */
Clock i2c0GclkClock = {
	.clkName			=	"i2c0GclkClock",
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK_SHIFT,
};

/*	I2C0 Functional Clock	*/
Clock i2c0FclkClock = {
	.clkName			=	"i2c0FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&i2c0GclkClock,
	.activeChildCount	=	0,
};


/*	GPIO_0_GDBCLK   */
Clock gpio0GclkClock = {
	.clkName			=	"gpio0GclkClock",
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&clk32768Clock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK_SHIFT,

	.muxInputSelMask	=	CM_DPLL_CLKSEL_GPIO0_DBCLK_CLKSEL,
	.muxInputSelReg		=	(unsigned int *)(SOC_CM_DPLL_REGS + CM_DPLL_CLKSEL_GPIO0_DBCLK),
	.muxInputSelVal		=	CM_DPLL_CLKSEL_GPIO0_DBCLK_CLKSEL_SEL1, /*	RC OSC	*/
};


/* GPIO_0_OPT Clock */
Clock gpio0OptClock = {
	.clkName			=	"gpio0OptClock",
	.clockSpeedHz		=	CLK_CLOCK_SPEED_32768_HZ,
	.parentClock		=	&gpio0GclkClock,
	.activeChildCount	=	0,
};


/*	I2C2	*/

/***********************************************************************/
/*	spi_Gclk	*/
Clock spiGclkClock = {
	.clkName			=	"spiGclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK_SHIFT,
};

/*	spi0 Functional Clock	*/
Clock spi0FclkClock = {
	.clkName			=	"spi0FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&spiGclkClock,
	.activeChildCount	=	0,
};

/*	spi1	*/

/***********************************************************************/
/*	uart_gfclk Gclk	*/
Clock uartGclkClock = {
	.clkName			=	"uartGclkClock",
	.clkDomainPtr		=	&l4lsClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL),
	.clockGateStatusMask =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK,
	.clockGateStatusShift =	CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_UART_GFCLK_SHIFT,
};

/*	UART1	*/
/*	UART2	*/
/*	UART3	*/
/*	UART4	*/
/*	UART5	*/

/***********************************************************************/
/*	UART0_GFCLK */
Clock uart0GclkClock = {
	.clkName			=	"uart0GclkClock",
	.clkDomainPtr		=	&wkupClkDomain,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&clk192MhzClock,
	.activeChildCount	=	0,
	.clkGateStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL),
	.clockGateStatusMask =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK,
	.clockGateStatusShift =	CM_WKUP_CLKSTCTRL_CLKACTIVITY_UART0_GFCLK_SHIFT,
};

/*	UART0 Functional Clock	*/
Clock uart0FclkClock = {
	.clkName			=	"uart0FclkClock",
	.isLeafClock		=	true,
	.clockSpeedHz		=	CLK_CLOCK_SPEED_192_MHZ/4,
	.parentClock		=	&uart0GclkClock,
	.activeChildCount	=	0,
};
/***********************************************************************/
/*	MMC_CLK	*/
/*	TIMER0	*/

/***********************************************************************/

/*	MPU_CLK	*/
ModuleClock mpuModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_MPU_REGS + CM_MPU_MPU_CLKCTRL),
		.enableValue		=	(CM_MPU_MPU_CLKCTRL_MODULEMODE_ENABLE <<
								CM_MPU_MPU_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_MPU_MPU_CLKCTRL_MODULEMODE_DISABLE <<
								CM_MPU_MPU_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_MPU_REGS + CM_MPU_MPU_CLKCTRL),
		.idleStatusMask		=	CM_MPU_MPU_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_MPU_MPU_CLKCTRL_IDLEST_SHIFT,
		.stbyStatusMask		=	CM_MPU_MPU_CLKCTRL_STBYST,
		.stbyStatusShift	=	CM_MPU_MPU_CLKCTRL_STBYST_SHIFT,

		.iClk				=	{&mpuIclkClock},
};

/*	EMIF	*/
ModuleClock emifModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL),
		.enableValue		=	(CM_PER_EMIF_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_EMIF_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_EMIF_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_EMIF_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_EMIF_CLKCTRL),
		.idleStatusMask		=	CM_PER_EMIF_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_EMIF_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&emifIclkClock},
		.fClk	=	{&emifFclkClock},
};

/*	TIMER2	*/
ModuleClock timer2ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TIMER2_CLKCTRL),
		.enableValue		=	(CM_PER_TIMER2_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TIMER2_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TIMER2_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TIMER2_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(CM_PER_TIMER2_CLKCTRL + SOC_CM_PER_REGS),
		.idleStatusMask		=	CM_PER_TIMER2_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TIMER2_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&timer2IclkClock},
		.fClk	=	{&timer2FclkClock},
};


/*	TIMER3	*/
ModuleClock timer3ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TIMER3_CLKCTRL),
		.enableValue		=	(CM_PER_TIMER3_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TIMER3_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TIMER3_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TIMER3_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(CM_PER_TIMER3_CLKCTRL + SOC_CM_PER_REGS),
		.idleStatusMask		=	CM_PER_TIMER3_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TIMER3_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&timer3IclkClock},
		.fClk	=	{&timer3FclkClock},
};


/*	TIMER4	*/
ModuleClock timer4ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TIMER4_CLKCTRL),
		.enableValue		=	(CM_PER_TIMER4_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TIMER4_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TIMER4_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TIMER4_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(CM_PER_TIMER4_CLKCTRL + SOC_CM_PER_REGS),
		.idleStatusMask		=	CM_PER_TIMER4_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TIMER4_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&timer4IclkClock},
		.fClk	=	{&timer4FclkClock},
};

/* TIMER6 */
ModuleClock timer6ModClock = {
    .clockCtrlReg           =   (unsigned int *)
                                (SOC_CM_PER_REGS + CM_PER_TIMER6_CLKCTRL),
    .enableValue            =   (CM_PER_TIMER6_CLKCTRL_MODULEMODE_ENABLE <<
                                 CM_PER_TIMER6_CLKCTRL_MODULEMODE_SHIFT),
    .disableValue           =   (CM_PER_TIMER6_CLKCTRL_MODULEMODE_DISABLED <<
                                 CM_PER_TIMER6_CLKCTRL_MODULEMODE_SHIFT),
    .moduleStatusReg        =
                                (unsigned int *)
                                (CM_PER_TIMER6_CLKCTRL + SOC_CM_PER_REGS),
    .idleStatusMask         =   CM_PER_TIMER6_CLKCTRL_IDLEST,
    .idleStatusShift        =   CM_PER_TIMER6_CLKCTRL_IDLEST_SHIFT,

    .iClk   =   {&timer6IclkClock},
    .fClk   =   {&timer6FclkClock},
};

/*	TIMER7	*/
ModuleClock timer7ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TIMER7_CLKCTRL),
		.enableValue		=	(CM_PER_TIMER7_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TIMER7_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TIMER7_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TIMER7_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(CM_PER_TIMER7_CLKCTRL + SOC_CM_PER_REGS),
		.idleStatusMask		=	CM_PER_TIMER7_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TIMER7_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&timer7IclkClock},
		.fClk	=	{&timer7FclkClock},
};



/*	TIMER1	*/
ModuleClock timer1ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_TIMER1_CLKCTRL),
		.enableValue		=	(CM_WKUP_TIMER1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_TIMER1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_TIMER1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_TIMER1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(CM_WKUP_TIMER1_CLKCTRL + SOC_CM_WKUP_REGS),
		.idleStatusMask		=	CM_WKUP_TIMER1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_TIMER1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&timer1IclkClock},
		.fClk	=	{&timer1FclkClock},
};

/*	I2C1	*/
ModuleClock i2c1ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_I2C1_CLKCTRL),
		.enableValue		=	(CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_I2C1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_I2C1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_I2C1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_I2C1_CLKCTRL),
		.idleStatusMask		=	CM_PER_I2C1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_I2C1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&i2c1IclkClock},
		.fClk	=	{&i2c1FclkClock},
};

/*	GPIO3	*/
ModuleClock gpio3ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL),
		.enableValue		=	(CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_GPIO3_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_GPIO3_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_GPIO3_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL),
		.idleStatusMask		=	CM_PER_GPIO3_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_GPIO3_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&gpio3IclkClock},
		.optClk =	{&gpio3OptClock},
};

/*	SPI0	*/
ModuleClock spi0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL),
		.enableValue		=	(CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_SPI0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_SPI0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_SPI0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL),
		.idleStatusMask		=	CM_PER_SPI0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_SPI0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&spi0IclkClock},
		.fClk	=	{&spi0FclkClock},
};


/*	UART0	*/
ModuleClock uart0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL),
		.enableValue		=	(CM_WKUP_UART0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_UART0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_UART0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_UART0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_UART0_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_UART0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_UART0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&uart0IclkClock},
		.fClk	=	{&uart0FclkClock},
};

/*	MAILBOX0	*/
ModuleClock mailbox0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_MAILBOX0_CLKCTRL),
		.enableValue		=	(CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_MAILBOX0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_MAILBOX0_CLKCTRL),
		.idleStatusMask		=	CM_PER_MAILBOX0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_MAILBOX0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&mailbox0IclkClock},
};

/*	EPWMSS0	*/
ModuleClock epwmss0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_EPWMSS0_CLKCTRL),
		.enableValue		=	(CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_EPWMSS0_CLKCTRL),
		.idleStatusMask		=	CM_PER_EPWMSS0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&epwmss0IclkClock},
};


/*	RTC	*/
ModuleClock rtcModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL),
		.enableValue		=	(CM_RTC_RTC_CLKCTRL_MODULEMODE_ENABLE <<
								CM_RTC_RTC_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_RTC_RTC_CLKCTRL_MODULEMODE_DISABLE <<
								CM_RTC_RTC_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_RTC_REGS + CM_RTC_RTC_CLKCTRL),
		.idleStatusMask		=	CM_RTC_RTC_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_RTC_RTC_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&rtcIclkClock},
		.fClk	=	{&rtcFclkClock},
};

/*	L4LS	*/
ModuleClock l4lsModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL),
		.enableValue		=	(CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_L4LS_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_L4LS_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_L4LS_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL),
		.idleStatusMask		=	CM_PER_L4LS_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_L4LS_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&l4lsIclkClock},
};

/*	WDT1	*/
ModuleClock wdt1ModClock = {
		.clockCtrlReg		= 	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_WDT1_CLKCTRL),
		.enableValue		=	(CM_WKUP_WDT1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_WDT1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_WDT1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_WDT1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_WDT1_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_WDT1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_WDT1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&wdt1IclkClock},
		.fClk	=	{&wdt1FclkClock},
};

/*	ADC_TSC	*/
ModuleClock adcTscModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL),
		.enableValue		=	(CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_ADC_TSC_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_ADC_TSC_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&adcTscIclkClock},
		.fClk	=	{&adcFclkClock},
};

/*	LCDC	*/
ModuleClock lcdcModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_LCDC_CLKCTRL),
		.enableValue		=	(CM_PER_LCDC_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_LCDC_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_LCDC_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_LCDC_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_LCDC_CLKCTRL),
		.idleStatusMask		=	CM_PER_LCDC_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_LCDC_CLKCTRL_IDLEST_SHIFT,
		.stbyStatusMask		=	CM_PER_LCDC_CLKCTRL_STBYST,
		.stbyStatusShift	=	CM_PER_LCDC_CLKCTRL_STBYST_SHIFT,

		.iClk	=	{&lcdcL3IclkClock,
					 &lcdcL4IclkClock},
		.fClk	=	{&lcdcFclkClock},
		.clkEnable = {&lcdcL4ClkEnableClock},
};

/*	WKUP_M3	*/
ModuleClock wkupM3ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_WKUP_M3_CLKCTRL),
		.enableValue		=	(CM_WKUP_WKUP_M3_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_WKUP_M3_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_WKUP_M3_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_WKUP_M3_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_WKUP_M3_CLKCTRL),
		.stbyStatusMask		=	CM_WKUP_WKUP_M3_CLKCTRL_STBYST,
		.stbyStatusShift	=	CM_WKUP_WKUP_M3_CLKCTRL_STBYST_SHIFT,

		.iClk	=	{&wkupM3IclkClock},
};

/*	L4WKUP	*/
ModuleClock l4WkupModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL),
		.enableValue		=	(CM_WKUP_L4WKUP_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_L4WKUP_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_L4WKUP_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_L4WKUP_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_L4WKUP_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&l4WkupIclkClock},
};

/*	MCASP1	*/
ModuleClock mcasp1ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_MCASP1_CLKCTRL),
		.enableValue		=	(CM_PER_MCASP1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_MCASP1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_MCASP1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_MCASP1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_MCASP1_CLKCTRL),
		.idleStatusMask		=	CM_PER_MCASP1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_MCASP1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&mcasp1IclkClock},
		.fClk	=	{&mcasp1FclkClock},
};


/*	OCMCRAM	*/
ModuleClock ocmcramModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_OCMCRAM_CLKCTRL),
		.enableValue		=	(CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_OCMCRAM_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_OCMCRAM_CLKCTRL),
		.idleStatusMask		=	CM_PER_OCMCRAM_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_OCMCRAM_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&ocmcramIclkClock},
};


/*	TPCC	*/
ModuleClock tpccModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPCC_CLKCTRL),
		.enableValue		=	(CM_PER_TPCC_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TPCC_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TPCC_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TPCC_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPCC_CLKCTRL),
		.idleStatusMask		=	CM_PER_TPCC_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TPCC_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&tpccIclkClock},
};


/*	TPTC0	*/
ModuleClock tptc0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL),
		.enableValue		=	(CM_PER_TPTC0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TPTC0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL),
		.idleStatusMask		=	CM_PER_TPTC0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TPTC0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&tptc0IclkClock},
};

/*	TPTC1	*/
ModuleClock tptc1ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL),
		.enableValue		=	(CM_PER_TPTC1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TPTC1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL),
		.idleStatusMask		=	CM_PER_TPTC1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TPTC1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&tptc1IclkClock},
};

/*	TPTC2	*/
ModuleClock tptc2ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL),
		.enableValue		=	(CM_PER_TPTC2_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_TPTC2_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL),
		.idleStatusMask		=	CM_PER_TPTC2_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_TPTC2_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&tptc2IclkClock},
};


/*	L3	*/
ModuleClock l3ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL),
		.enableValue		=	(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_L3_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_L3_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_L3_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL),
		.idleStatusMask		=	CM_PER_L3_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_L3_CLKCTRL_IDLEST_SHIFT,

		.iClk		=	{&l3IclkClock},
		.clkEnable 	= 	{&l3l4IcEnableClock},
};

/*	GPMC	*/
ModuleClock gpmcModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL),
		.enableValue		=	(CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_GPMC_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL),
		.idleStatusMask		=	CM_PER_GPMC_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_GPMC_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&gpmcIclkClock},
};

/*	USB0	*/
ModuleClock usbModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL),
		.enableValue		=	(CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_USB0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL),
		.idleStatusMask		=	CM_PER_USB0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_USB0_CLKCTRL_IDLEST_SHIFT,
		.stbyStatusMask		=	CM_PER_USB0_CLKCTRL_STBYST,
		.stbyStatusShift	=	CM_PER_USB0_CLKCTRL_STBYST_SHIFT,

		.iClk	=	{&usb0IclkClock},
		.optClk = {&usbPllclkClock},
};

/*	L4FW	*/
ModuleClock l4fwModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4FW_CLKCTRL),
		.enableValue		=	(CM_PER_L4FW_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_L4FW_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_L4FW_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_L4FW_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_L4FW_CLKCTRL),
		.idleStatusMask		=	CM_PER_L4FW_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_L4FW_CLKCTRL_IDLEST_SHIFT,
		.iClk	=	{&l4fwIclkClock},
};

/*	CPGMAC0	*/
ModuleClock cpgmac0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_CPGMAC0_CLKCTRL),
		.enableValue		=	(CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_CPGMAC0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_CPGMAC0_CLKCTRL),
		.idleStatusMask		=	CM_PER_CPGMAC0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_CPGMAC0_CLKCTRL_IDLEST_SHIFT,
		.stbyStatusMask		=	CM_PER_CPGMAC0_CLKCTRL_STBYST,
		.stbyStatusShift	=	CM_PER_CPGMAC0_CLKCTRL_STBYST_SHIFT,

		.iClk	=	{&cpsw125MhzIclkClock},
		.fClk =	{
						&cpsw5MhzFclkClock,
						&cpsw50MhzFclkClock,
						&cpsw250MhzFclkClock,
						&cpswCptsFclkClock,
				},
};

/*	I2C0	*/
ModuleClock i2c0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL),
		.enableValue		=	(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_I2C0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_I2C0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_I2C0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&i2c0IclkClock},
		.fClk	=	{&i2c0FclkClock},
};


/*	GPIO0	*/
ModuleClock gpio0ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL),
		.enableValue		=	(CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE <<
								CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_DISABLE <<
								CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL),
		.idleStatusMask		=	CM_WKUP_GPIO0_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_WKUP_GPIO0_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&gpio0IclkClock},
		.optClk	=	{&gpio0OptClock},
};


/*	GPIO1	*/
ModuleClock gpio1ModClock = {
		.clockCtrlReg		=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL),
		.enableValue		=	(CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE <<
								CM_PER_GPIO1_CLKCTRL_MODULEMODE_SHIFT),
		.disableValue		=	(CM_PER_GPIO1_CLKCTRL_MODULEMODE_DISABLE <<
								CM_PER_GPIO1_CLKCTRL_MODULEMODE_SHIFT),
		.moduleStatusReg	=	(unsigned int *)(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL),
		.idleStatusMask		=	CM_PER_GPIO1_CLKCTRL_IDLEST,
		.idleStatusShift	=	CM_PER_GPIO1_CLKCTRL_IDLEST_SHIFT,

		.iClk	=	{&gpio1IclkClock},
		.optClk	=	{&gpio1OptClock},
};


/*	List of clocks	*/
ModuleClock *ModuleClockList[] =
{
	&mpuModClock,
	&emifModClock,
	&timer2ModClock,
	&i2c1ModClock,
	&gpio3ModClock,
	&spi0ModClock,
	&uart0ModClock,
	&mailbox0ModClock,
	&rtcModClock,
	&l4lsModClock,
	&wdt1ModClock,
	&adcTscModClock,
	&lcdcModClock,
	&wkupM3ModClock,
	&l4WkupModClock,
	&mcasp1ModClock,
	&ocmcramModClock,
	&l3ModClock,
	&gpmcModClock,
	&usbModClock,
	&l4fwModClock,
	&cpgmac0ModClock,
	&i2c0ModClock,
	&gpio0ModClock,
	&gpio1ModClock,
	&timer1ModClock,
	&timer3ModClock,
	&timer4ModClock,
	&timer7ModClock,
	&epwmss0ModClock,
	&tpccModClock,
	&tptc0ModClock,
	&tptc1ModClock,
	&tptc2ModClock,
    &timer6ModClock,
};

/*	List of clock domains	*/
ClockDomain *clockDomainList[] =
{
	&l4lsClkDomain,
	&rtcClkDomain,
	&wkupClkDomain,
	&perL3ClkDomain,
	&l4WkupAonClkDomain,
	&mpuClkDomain,
	&lcdcClkDomain,
	&l4fwClkDomain,
	&l3sClkDomain,
	&cpsw125MhzClkDomain,
	&l4hsClkDomain,
	&clk24MhzClkDomain,
};



#ifdef __cplusplus
}
#endif
