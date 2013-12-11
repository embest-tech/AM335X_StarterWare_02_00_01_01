/**
 *  \file   demoPwrMgmnt.c
 *
 *  \brief  Functions used Power management implementation are defined here
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
 
#include <string.h>
#include "hs_mmcsd.h"
#include "hw_usbOtg_AM335x.h"
#include "hw_usbphyGS70.h"
#include "hw_cm_wkup.h"
#include "hw_cm_per.h"
#include "hw_prm_cefuse.h"
#include "hw_prm_gfx.h"
#include "hw_cm_cefuse.h"
#include "hw_cm_gfx.h"
#include "hw_emif4d.h"
#include "delay.h"
#include "consoleUtils.h"
#include "board.h"
#include "ecap.h"
#include "edma.h"
#include "ehrpwm.h"
#include "edma_event.h"
#include "hsi2c.h"
#include "gpmc.h"
#include "raster.h"
#include "tsc_adc.h"
#include "interrupt.h"
#include "cpsw.h"
#include "mdio.h"
#include "mcasp.h"
#include "dmtimer.h"
#include "uart_irda_cir.h"
#include "device.h"
#include "pin_mux.h"
#include "clock.h"
#include "mailbox.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "demoCfg.h"
#include "demoDvfs.h"
#include "demoRaster.h"
#include "demoEcap.h"
#include "demoEnet.h"
#include "demoTimer.h"
#include "demoMain.h"
#include "demoGpio.h"
#include "demoRtc.h"
#include "demoUart.h"
#include "demoTouch.h"
#include "demoToneLoop.h"
#include "cm3wkup_proxy.h"
#include "demoPwrMgmnt.h"

/******************************************************************************
**              EXTERNAL VARIABLES & FUNCTIONS
******************************************************************************/
extern volatile unsigned int IsTSPress;
extern volatile unsigned int x_data[];
extern volatile unsigned int y_data[];
extern void romRestoreLocation(void);
extern CPSWCONTEXT cpswContext;
extern MDIOCONTEXT mdioContext;

/******************************************************************************
**              FUNCTION PROTOTYPES
******************************************************************************/
void ConfigIdleMode(void);
static void PeripheralsContextSave(unsigned int slpMode, unsigned int wakeSrc);
static void PeripheralsContextRestore(unsigned int slpMode, unsigned int wakeSrc);
static void PeripheralsHalt(void);
static void PeripheralsResume(void);
static void PowerDownConfig(void);

/******************************************************************************
**                       GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
RASTERCONTEXT rasterContext;
MCASPCONTEXT mcaspContext;
DMTIMERCONTEXT dmtimerContext[4];
EDMACONTEXT edmaContext;
ECAPCONTEXT ecapContext;
I2CCONTEXT i2cContext;
GPIOCONTEXT gpioContext[2];
CTRLREGCONTEXT ctrlContext;

tIOPin pmStdbySrcIOPin;

/* Hold Wake source */
volatile unsigned int wakeSource = WAKE_SOURCE_TMR;

/*
** This list is application specific. It indexes into the master module
** clock list. It lists the modules to be enabled/disabled.
*/
tMapModule pmModuleList[] =
{
    {CLK_MAILBOX0, TRUE},
    {CLK_GPIO1, TRUE},
    {CLK_GPIO0, TRUE},
    {CLK_LCDC, TRUE},
    {CLK_I2C1, TRUE},
    {CLK_ADC_TSC, TRUE},
    {CLK_RTC, TRUE},
    {CLK_MPU_CLK, TRUE},
    {CLK_TIMER2, TRUE},
    {CLK_TIMER3, TRUE},
    {CLK_TIMER4, TRUE},
    {CLK_TIMER6, TRUE},
    {CLK_TIMER7, TRUE},
    {CLK_MCASP1, TRUE},
    {CLK_TPTC2, TRUE},
    {CLK_TPTC1, TRUE},
    {CLK_TPTC0, TRUE},
    {CLK_TPCC, TRUE},
    {CLK_CPGMAC0, TRUE},
    {CLK_EPWMSS0, TRUE},
    {CLK_UART0, TRUE}
};

/* OPP to string table */
tValStr sleepModeStrMap[] =
{
    {0, ""},                            /* NONE */
    {SLEEP_MODE_DS0, "DS0"},            /* Deep Sleep0 */
    {SLEEP_MODE_DS1, "DS1"},            /* Deep Sleep1 */
    {0, ""},                            /* NONE */
    {SLEEP_STAND_BY_MODE, "Standby"},   /* Standby */
};

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

/*
** Halts any currently active transactions
*/
static void PeripheralsHalt(void)
{
    RasterIdleModeConfig(LCDC_BASE_ADDR, LCDC_SYSCONFIG_IDLEMODE_FORCE <<
                                              LCDC_SYSCONFIG_STANDBYMODE_SHIFT);

    RasterStandbyModeConfig(LCDC_BASE_ADDR,
                                         LCDC_SYSCONFIG_STANDBYMODE_FORCE <<
                                                 LCDC_SYSCONFIG_IDLEMODE_SHIFT);

    delay(30);

    /* Disable EDMA for the transfer */
    EDMA3DisableTransfer(EDMA_BASE_ADDR, EDMA3_CHA_MCASP1_TX,
                         EDMA3_TRIG_MODE_EVENT);

    McASPTxReset(MCASP_CTRL_BASE_ADDR);

    /* LCD back light OFF */
    EcapBkLightDisable();

    /* Disable End-of-frame interrupt	*/
    Raster0EOFIntDisable();

    CPSWCPDMACmdIdleEnable(CPSW_CPDMA_BASE_ADDR);

    /*  configure Modules to idle mode  */
    ConfigIdleMode();

    TouchScreenFIFOFlush();

    if(wakeSource != WAKE_SOURCE_TSC)
    {
        /* Config ADC in powerdown mode */
        TSCADCSetADCPowerDown(ADC_TSC_BASE_ADDR);
        TouchDisable();
    }

    /* Disable GFX power domain */
    HWREG(SOC_PRM_GFX_REGS + PRM_GFX_PM_GFX_PWRSTCTRL) = 0x0;

    /* Disable GFX module */
    HWREG(SOC_CM_GFX_REGS + CM_GFX_L3_CLKSTCTRL) = 0x0;
    HWREG(SOC_CM_GFX_REGS + CM_GFX_BITBLT_CLKCTRL) = 0x0;
    HWREG(SOC_CM_GFX_REGS + CM_GFX_MMUCFG_CLKCTRL) = 0x0;
    HWREG(SOC_CM_GFX_REGS + CM_GFX_MMUDATA_CLKCTRL) = 0x0;

    /* Set GFX modules to SW_SLEEP */
    HWREG(SOC_CM_GFX_REGS + CM_GFX_L4LS_GFX_CLKSTCTRL) = 0x1;
    HWREG(SOC_CM_GFX_REGS + PRM_GFX_PM_GFX_PWRSTCTRL) = 0x1;

    /* disable Cust_efuse PD */
    HWREG(SOC_PRM_CEFUSE_REGS + PRM_CEFUSE_PM_CEFUSE_PWRSTCTRL) = 0x0;

    /* disable cust_efuse module */
    HWREG(SOC_CM_CEFUSE_REGS + CM_CEFUSE_CEFUSE_CLKCTRL) = 0x0;

    /* Set CUST_EFUSE modules to SW_SLEEP */
    HWREG(SOC_CM_CEFUSE_REGS + CM_CEFUSE_CLKSTCTRL) = 0x1;
}

/*
** Halts any currently active transactions
*/
static void PeripheralsResume(void)
{
    RasterStandbyModeConfig(LCDC_BASE_ADDR,
                                         LCDC_SYSCONFIG_STANDBYMODE_NOSTANDBY <<
                                                 LCDC_SYSCONFIG_IDLEMODE_SHIFT);
    RasterIdleModeConfig(LCDC_BASE_ADDR, LCDC_SYSCONFIG_IDLEMODE_NOIDLE <<
                                              LCDC_SYSCONFIG_STANDBYMODE_SHIFT);

    AudioTxActivate();

    Raster0EOFIntEnable();
    EcapBkLightEnable();

    /* Bring CPSW out of standby and restart DHCP */   
    CPSWWrControlRegReset(CPSW_WR_BASE_ADDR);
    CPSWCPDMACmdIdleDisable(CPSW_CPDMA_BASE_ADDR);
    EnetDHCPRestart();	

    /* Set CUST_EFUSE modules to SW_WAKE */
    HWREG(SOC_CM_CEFUSE_REGS + CM_CEFUSE_CLKSTCTRL) = 0x2;

    /* Set GFX modules to SW_WAKE */
    HWREG(SOC_CM_GFX_REGS + CM_GFX_L3_CLKSTCTRL) = 0x2;
    HWREG(SOC_CM_GFX_REGS + CM_GFX_L4LS_GFX_CLKSTCTRL) = 0x2;

    if(wakeSource != WAKE_SOURCE_TSC)
    {
        while(TSCADCSequencerFSMBusyStatus(ADC_TSC_BASE_ADDR));
        TouchEnable();

        /* Enable ADC power */
        TSCADCSetADCPowerUp(ADC_TSC_BASE_ADDR);
    }

    TSCADCIntStatusClear(ADC_TSC_BASE_ADDR,0xFFFF);
}

/*
** Save the peripheral register context and suspends any transactions
*/
static void PeripheralsContextSave(unsigned int slpMode, unsigned int wakeSrc)
{
    switch(slpMode)
    {
        case SLEEP_MODE_DS0:

            /* Save the ctrl reg config - IO Pads, pwmss_ctrl, gmmi_sel */
            ControlRegContextSave(&ctrlContext);

            RasterContextSave(LCDC_BASE_ADDR, &rasterContext);
	
            EcapContextSave(ECAP_BASE_ADDR, PWMSS_BASE_ADDR, &ecapContext);
	
            EDMA3ContextSave(EDMA_BASE_ADDR, &edmaContext);

            McASPContextSave(MCASP_CTRL_BASE_ADDR, MCASP_FIFO_BASE_ADDR,
                             &mcaspContext, McASP_CONTEXT_TX);
					  
            EnetContextSave();

            DMTimerContextSave(DMTIMER2_BASE_ADDR, &dmtimerContext[0]);
            DMTimerContextSave(DMTIMER3_BASE_ADDR, &dmtimerContext[1]);
            DMTimerContextSave(DMTIMER4_BASE_ADDR, &dmtimerContext[2]);
            DMTimerContextSave(DMTIMER7_BASE_ADDR, &dmtimerContext[3]);
  
            I2CContextSave(I2C1_BASE_ADDR, &i2cContext);
	
            gpioContextSave(GPIO0_BASE_ADDR, &gpioContext[0]);
            gpioContextSave(GPIO1_BASE_ADDR, &gpioContext[1]);

            EnetReset();
            break;

        case SLEEP_STAND_BY_MODE:
            if(WAKE_SOURCE_UART == wakeSrc)
            {
                IOPadContextSave(&ctrlContext, GPIO_UART_RDX_PAD_OFFSET);
                gpioContextSave(GPIO_INST_BASE_UART_RXD, &gpioContext[1]);
            }
            else if(WAKE_SOURCE_GPIO == wakeSrc)
            {
                IOPadContextSave(&ctrlContext, GPIO_SW_PAD_OFFSET);
                gpioContextSave(GPIO_INST_BASE_SW, &gpioContext[0]);
            }
            break;

        default:
            break;
    }
}

/*
** Restore the peripherals context and resumes operation
*/
static void PeripheralsContextRestore(unsigned int slpMode, unsigned int wakeSrc)
{
    switch(slpMode)
    {
        case SLEEP_MODE_DS0:

            /* Restore the ctrl reg config - IO Pads, pwmss_ctrl, gmmi_sel */
            ControlRegContextRestore(&ctrlContext);

            RasterContextRestore(LCDC_BASE_ADDR, &rasterContext);
		
            EcapContextRestore(ECAP_BASE_ADDR, PWMSS_BASE_ADDR, &ecapContext);
	
            gpioContextRestore(GPIO0_BASE_ADDR, &gpioContext[0]);
            gpioContextRestore(GPIO1_BASE_ADDR, &gpioContext[1]);

            DMTimerContextRestore(DMTIMER2_BASE_ADDR, &dmtimerContext[0]);
            DMTimerContextRestore(DMTIMER3_BASE_ADDR, &dmtimerContext[1]);
            DMTimerContextRestore(DMTIMER4_BASE_ADDR, &dmtimerContext[2]);
            DMTimerContextRestore(DMTIMER7_BASE_ADDR, &dmtimerContext[3]);

            initializeMailbox(MAILBOX_BASE_ADDR);

            MDIOContextRestore(CPSW_MDIO_BASE_ADDR, &mdioContext);

            CPSWContextRestore(&cpswContext);
	
            EDMA3ContextRestore(EDMA_BASE_ADDR, &edmaContext);
            McASPContextRestore(MCASP_CTRL_BASE_ADDR, MCASP_FIFO_BASE_ADDR,
                                &mcaspContext, McASP_CONTEXT_TX);
	
            I2CContextRestore(I2C1_BASE_ADDR, &i2cContext);
            break;

        case SLEEP_STAND_BY_MODE:
            if(WAKE_SOURCE_UART == wakeSrc)
            {
                IOPadContextRestore(&ctrlContext, GPIO_UART_RDX_PAD_OFFSET);
                gpioContextRestore(GPIO_INST_BASE_UART_RXD, &gpioContext[1]);
            }
            else if(WAKE_SOURCE_GPIO == wakeSrc)
            {
                IOPadContextRestore(&ctrlContext, GPIO_SW_PAD_OFFSET);
                gpioContextRestore(GPIO_INST_BASE_SW, &gpioContext[0]);
            }
            break;

        default:
            break;
    }
}

void ConfigIdleMode(void)
{
    unsigned int  usbPhyCfg = 0;

    /* Configure the modules Mode to idle state */
    HWREG(CPSW_WR_BASE_ADDR + CPSW_WR_CONTROL) = 0x02;

    /* USB0 */
    HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) =
                                       (CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE <<
                                          CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT);

    delay(1);

    HWREG(USB_BASE_ADDR + USBSS_SYSCONFIG) = 0x08;

    HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) =
                                      (CM_PER_USB0_CLKCTRL_MODULEMODE_DISABLE <<
                                          CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT);

    /* Turns off USB phy */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKDCOLDO_DPLL_PER) = 0x0u;
    usbPhyCfg = HWREG(CFGCHIP2_USBPHYCTRL);
    usbPhyCfg |= (USBPHY_CM_PWRDN | USBPHY_OTG_PWRDN);
    HWREG(CFGCHIP2_USBPHYCTRL) = usbPhyCfg;

    /* TPTC0 */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) =
                                      (CM_PER_TPTC0_CLKCTRL_MODULEMODE_ENABLE <<
                                         CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT);

    while((HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) &
          (CM_PER_TPTC0_CLKCTRL_STBYST | CM_PER_TPTC0_CLKCTRL_IDLEST)));

    HWREG(EDMATC0_BASE_ADDR + EDMA3TC_SYSCONFIG) = 0x08;

    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) =
                                     (CM_PER_TPTC0_CLKCTRL_MODULEMODE_DISABLE <<
                                         CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT);

    /* TPTC1 */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) =
                                      (CM_PER_TPTC1_CLKCTRL_MODULEMODE_ENABLE <<
                                         CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT);

    while((HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) &
          (CM_PER_TPTC1_CLKCTRL_STBYST | CM_PER_TPTC1_CLKCTRL_IDLEST)));

    HWREG(EDMATC1_BASE_ADDR + EDMA3TC_SYSCONFIG) = 0x08;

    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) =
                                     (CM_PER_TPTC1_CLKCTRL_MODULEMODE_DISABLE <<
                                         CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT);

    /* TPTC2 */
    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) =
                                      (CM_PER_TPTC2_CLKCTRL_MODULEMODE_ENABLE <<
                                         CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT);

    while((HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) &
          (CM_PER_TPTC2_CLKCTRL_STBYST | CM_PER_TPTC2_CLKCTRL_IDLEST)));

    HWREG(EDMATC2_BASE_ADDR + EDMA3TC_SYSCONFIG) = 0x08;

    HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) =
                                     (CM_PER_TPTC2_CLKCTRL_MODULEMODE_DISABLE <<
                                         CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT);

    /* GPMC */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) =
                                       (CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE <<
                                          CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT);

    while((HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) &
          (CM_PER_GPMC_CLKCTRL_IDLEST)));

    GPMCIdleModeSelect(GPMC_BASE_ADDR, GPMC_IDLEMODE_FORCEIDLE);

    HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) =
                                      (CM_PER_GPMC_CLKCTRL_MODULEMODE_DISABLE <<
                                          CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT);

    /* MMC0 */
    HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) =
                                       (CM_PER_MMC0_CLKCTRL_MODULEMODE_ENABLE <<
                                          CM_PER_MMC0_CLKCTRL_MODULEMODE_SHIFT);

    while((HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) &
          (CM_PER_MMC0_CLKCTRL_IDLEST)));

    HSMMCSDSystemConfig(MMCSD_BASE_ADDR, HS_MMCSD_SMARTIDLE_FORCE);

    HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) =
                                      (CM_PER_MMC0_CLKCTRL_MODULEMODE_DISABLE <<
                                          CM_PER_MMC0_CLKCTRL_MODULEMODE_SHIFT);

    /*
    ** SPI0
    ** Bootloader may initialize this module based on the boot mode selection.
    ** If this module is enabled then system fails to enter DS0 mode.
    */
    HWREG(SOC_CM_PER_REGS + CM_PER_SPI0_CLKCTRL) =
                                     (CM_PER_SPI0_CLKCTRL_MODULEMODE_DISABLED <<
                                          CM_PER_SPI0_CLKCTRL_MODULEMODE_SHIFT);
}

/*
** Configure the flag for given module in Module List
*/
static void ModuleListConfig(unsigned int status, unsigned int selModule)
{
    unsigned int index = 0;
    unsigned int noOfElements = sizeof(pmModuleList)/sizeof(pmModuleList[0]);

    for(index = 0; index < noOfElements; index++)
    {
        if((0xFF == selModule) || (selModule == pmModuleList[index].module))
        {
            pmModuleList[index].select = status;
        }
    }
}

/*
** This function configures given source for standby wakeup.
*/
void enableStandbyWakeSrc(unsigned int wakeSource)
{
    /* IO Pad Configuration */
    pmStdbySrcIOPin.padConfig.slewRate = 0;
    pmStdbySrcIOPin.padConfig.mode = 7;
    pmStdbySrcIOPin.padConfig.type = CONTROL_CONF_RXACTIVE;
    pmStdbySrcIOPin.padConfig.pullEnable = CONTROL_CONF_PULLUDDISABLE;
    pmStdbySrcIOPin.padConfig.pullSel = 0;

    /* GPIO Pin Configuration */
    pmStdbySrcIOPin.gpioConfig.dir = GPIO_DIR_INPUT;
    pmStdbySrcIOPin.gpioConfig.debouEnable = GPIO_DEBOUNCE_FUNC_DISABLE;
    pmStdbySrcIOPin.gpioConfig.intrEnable = 1;
    pmStdbySrcIOPin.gpioConfig.intrType = GPIO_INT_TYPE_BOTH_EDGE;

    switch(wakeSource)
    {
        case WAKE_SOURCE_TSC:

            /* Skip Touchscreen for Enable/Disable Module  */
            ModuleListConfig(FALSE, CLK_ADC_TSC);

            /* Enable hardware pen event interrupt */
            TSCADCEventInterruptEnable(TSC_ADC_INSTANCE,
                                       TSCADC_ASYNC_HW_PEN_EVENT_INT);
            break;

        case WAKE_SOURCE_UART:

            /* Skip GPIO for Enable/Disable Module  */
            ModuleListConfig(FALSE, CLK_GPIO1);

            /* Enable GPIO Interrupt on UART RXD Pin */
            pmStdbySrcIOPin.ioPadOff = GPIO_UART_RDX_PAD_OFFSET;
            pmStdbySrcIOPin.pinNum = GPIO_UART_RDX_PIN_NUM;
            pmStdbySrcIOPin.gpioBaseAddr = GPIO_INST_BASE_UART_RXD;
            pmStdbySrcIOPin.gpioConfig.intrLine = GPIO_UART_RXD_INTR_LINE;
            pmStdbySrcIOPin.intrNum = GPIO_UART_RXD_SYS_INT_NUM;
            pmStdbySrcIOPin.gpioIsr = gpioStdbyUartIsr;

            DemoGpioPinStandbySrcConfig(&pmStdbySrcIOPin);
            break;

        case WAKE_SOURCE_TMR:

            /* Skip Timer for Enable/Disable Module  */
            ModuleListConfig(FALSE, CLK_TIMER6);

            ConsoleUtilsPrintf("\r\n Peripheral domain Timer is configured as"
                               "wake source.\r\n\r\n ... system will release "
                               "from standby in 20 seconds ...\r\n\r\n");

            /* Configure Timer 6 */
            Timer6Config();

            /* Set the counter value */
            DMTimerCounterSet(DMTIMER6_BASE_ADDR, TIMER_OVRFLW_20_SECOND_24MHZ);

            /* Start the timer */
            Timer6Start();
            break;

        case WAKE_SOURCE_GPIO:

            /* Skip GPIO for Enable/Disable Module  */
            ModuleListConfig(FALSE, CLK_GPIO0);

            /* Enable GPIO Interrupt on UART RXD Pin */
            pmStdbySrcIOPin.ioPadOff = GPIO_SW_PAD_OFFSET;
            pmStdbySrcIOPin.pinNum = GPIO_SW_PIN_NUM;
            pmStdbySrcIOPin.gpioBaseAddr = GPIO_INST_BASE_SW;
            pmStdbySrcIOPin.gpioConfig.intrLine = GPIO_SW_INTR_LINE;
            pmStdbySrcIOPin.intrNum = GPIO_SW_SYS_INT_NUM;
            pmStdbySrcIOPin.gpioIsr = gpioStdbyGPIOIsr;

            DemoGpioPinStandbySrcConfig(&pmStdbySrcIOPin);
            break;

        case WAKE_SOURCE_RTC:

            /* Include GPIO for Enable/Disable Module  */
            ModuleListConfig(FALSE, CLK_RTC);

            ConsoleUtilsPrintf("\t..Alarm is configured to wakeup system after "
                               "20 Sec..");
            configWakeRTC();
            enableRTCAlarmIntr();
            break;

        default:
            break;
    }
}

/*
** This function will disable interrupts of wake sources corresponding
** to Standby mode
*/
void disableStandbyWakeSrc(unsigned int wakeSource)
{
    switch(wakeSource)
    {
        case WAKE_SOURCE_TSC:

            /* Include Touchscreen for Enable/Disable Module  */
            ModuleListConfig(TRUE, CLK_ADC_TSC);
            break;

        case WAKE_SOURCE_UART:

            /* Include GPIO for Enable/Disable Module  */
            ModuleListConfig(TRUE, CLK_GPIO1);
            break;

        case WAKE_SOURCE_TMR:

            /* Include Timer for Enable/Disable Module  */
            ModuleListConfig(TRUE, CLK_TIMER6);

            /* Clear the status of the interrupt flags */
            DMTimerIntStatusClear(DMTIMER6_BASE_ADDR, DMTIMER_INT_OVF_EN_FLAG);
            break;

        case WAKE_SOURCE_GPIO:

            /* Include GPIO for Enable/Disable Module  */
            ModuleListConfig(TRUE, CLK_GPIO0);
            break;

        case WAKE_SOURCE_RTC:

            /* Include GPIO for Enable/Disable Module  */
            ModuleListConfig(TRUE, CLK_RTC);

            disableRTCIntr();
            break;

        default:
            break;
       }
}

void enableWakeSource(unsigned int wakeSource)
{
    StepDisable();  /* TS Step disable */

    switch(wakeSource)
    {
        case WAKE_SOURCE_TSC:

            /* Enable touch screen wake */
            configTSWakeup();
            enableTSWakeup();

            /* Skip ADC IO Pads for low power mode configuration */
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN0, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN1, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN2, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN3, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN4, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN5, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN6, true);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN7, true);
            break;
		
        case WAKE_SOURCE_UART:
            enableUartWakeup();

            /* Skip UART IO Pads for low power mode configuration */
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_CTSN(0), true);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_RTSN(0), true);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_RXD(0), true);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_TXD(0), true);
            break;
		
        case WAKE_SOURCE_TMR:
            ConsoleUtilsPrintf("\t...system will wakeup after 20 Sec... ");
            setTimerCount(TIMER_OVRFLW_20_SECOND_16KHZ); /* 20 Sec */
            break;
		
        case WAKE_SOURCE_GPIO:
            configWakeGpio();
            enableGpioWake();

            /* Skip GPIO SW Pad for low power mode configuration */
            IOPadSel(&ctrlContext, GPIO_SW_PAD_OFFSET, true);
            break;

        case WAKE_SOURCE_RTC:
            ConsoleUtilsPrintf("\t..Alarm is configured to wakeup system after "
                               "20 Sec..");
            configWakeRTC();
            enableRTCAlarmWake();
            break;

        default:
            break;
    }
}

void disableWakeSource(unsigned int wakeSource)
{
    StepEnable(); /* TS Step Enable */

    switch(wakeSource)
    {
        case WAKE_SOURCE_TSC:

            /* Disable touch screen wake event */
            disableTSWakeup();

            /* Add ADC IO Pads for low power mode configuration */
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN0, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN1, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN2, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN3, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN4, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN5, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN6, false);
            IOPadSel(&ctrlContext, CONTROL_CONF_AIN7, false);
            break;
		
        case WAKE_SOURCE_UART:
            disableUartWakeup();

            /* Add UART IO Pads for low power mode configuration */
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_CTSN(0), false);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_RTSN(0), false);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_RXD(0), false);
            IOPadSel(&ctrlContext, CONTROL_CONF_UART_TXD(0), false);
            break;
		
        case WAKE_SOURCE_TMR:
            /* Clear timer interrupt */
            clearTimerInt();
            break;
		
        case WAKE_SOURCE_GPIO:
            disableGpioWake();

            /* Add GPIO SW Pad for low power mode configuration */
            IOPadSel(&ctrlContext, GPIO_SW_PAD_OFFSET, false);
            break;

        case WAKE_SOURCE_RTC:
            disableRTCAlarm();
            break;

        default:
            break;
    }
}

/*
** Configurations for DS0
*/
static void PowerDownConfig(void)
{
    unsigned int index = 0;

    for(index = 0; index < 16; index++)
    {
        /* Configure LCD DATA IO Pads for low power mode */
        IOPadConfigure(CONTROL_CONF_LCD_DATA(index), IOPAD_GPIO);

        /* Skip LCD DATA Clock from configuration */
        IOPadSel(&ctrlContext, CONTROL_CONF_LCD_DATA(index), true);
    }

    /* Configure Ethernet Clocks for low power mode */
    IOPadConfigure(CONTROL_CONF_RMII1_REFCLK, IOPAD_GPIO);
    IOPadConfigure(CONTROL_CONF_MDIO_CLK, IOPAD_GPIO);

    /* Skip Ethernet Clock from configuration */
    IOPadSel(&ctrlContext, CONTROL_CONF_RMII1_REFCLK, true);
    IOPadSel(&ctrlContext, CONTROL_CONF_MDIO_CLK, true);

    /* Skip system I2C Bus from configuration */
    IOPadSel(&ctrlContext, CONTROL_CONF_I2C0_SDA, true);
    IOPadSel(&ctrlContext, CONTROL_CONF_I2C0_SCL, true);

    /* Skip IO pads from CONTROL_CONF_NRESETIN_OUT to DDR for configuration */
    for(index = 0u;
            index < (CONTROL_CONF_AIN0 - CONTROL_CONF_NRESETIN_OUT)/4u; index++)
    {
        IOPadSel(&ctrlContext, CONTROL_CONF_NRESETIN_OUT + (4u * index), true);
    }

    /* Skip configuration of touchscreen IO Pads here */

    /* Skip other IO pads from CONTROL_CONF_VREFP for configuration */
    for(index = 0u;
               index <= (CONTROL_CONF_TESTOUT - CONTROL_CONF_VREFP)/4u; index++)
    {
        IOPadSel(&ctrlContext, CONTROL_CONF_VREFP + (4u * index), true);
    }

    /* Add Emulation IO Pads for low power mode configuration */
    IOPadSel(&ctrlContext, CONTROL_CONF_EMU(0u), false);
    IOPadSel(&ctrlContext, CONTROL_CONF_EMU(1u), false);

    /* Add USB VBUS IO Pads for low power mode configuration */
    IOPadSel(&ctrlContext, CONTROL_CONF_USB_DRVVBUS(0u), false);
    IOPadSel(&ctrlContext, CONTROL_CONF_USB_DRVVBUS(1u), false);

    /* Configure other IO Pads */
    IOPadSelConfigure(&ctrlContext, IOPAD_GPIO_PULLDN_RXACTIVE);
}

/*
** Enter the RTC Only power save mode
*/
void RTCOnlyModeEnter(void)
{
    PeripheralsHalt();

    enableRTCOnly();
    configRTCOnly();

    while(1); /* wait for RTC Only to execute */
}
 
/*
** Enter the desired power save mode
*/
void PowerSaveModeEnter(deepSleepData dsData, unsigned int slpMode)
{
    unsigned int memType = 0;
    unsigned int i = 0;
    unsigned int index = 0;
    unsigned int noOfElements = sizeof(pmModuleList)/sizeof(pmModuleList[0]);

    /*
    ** DDR Type is defined in bootloader
    */
    memType = ((HWREG(EMIF_BASE_ADDR + EMIF_SDRAM_CONFIG)
                & EMIF_SDRAM_CONFIG_REG_SDRAM_TYPE) >>
               EMIF_SDRAM_CONFIG_REG_SDRAM_TYPE_SHIFT);

    if((memType != MEM_TYPE_DDR2) && (memType != MEM_TYPE_DDR3))
    {
       ConsoleUtilsPrintf("\n\r ERROR: Not a valid DDR Type... ");
       ConsoleUtilsPrintf("\n\r Abort sleep mode execution... ");
       return;
    }

    PeripheralsContextSave(slpMode, wakeSource);

    PeripheralsHalt();

    dsData.dsDataBits.wakeSources = wakeSource;

    /*
    ** Configure CMD_ID and other parameters which are to be
    ** communicated with CM3
    */
    configIPCRegs(dsData);
    syncCm3();

    if(SLEEP_STAND_BY_MODE == slpMode)
    {
        /* Enable wake source interupt */
        enableStandbyWakeSrc(dsData.dsDataBits.wakeSources);
    }
    else
    {
        /*  Enable wake source      */
        enableWakeSource(dsData.dsDataBits.wakeSources);
    }

    /* Include MPU Clock in the disable list */
    for(index = 0; index < noOfElements; index++)
    {
        if(CLK_MPU_CLK == pmModuleList[index].module)
        {
            pmModuleList[index].select = TRUE;
        }
    }

    /* Disable clock */
    while(true != disableSelModuleClock(pmModuleList,
                               (sizeof(pmModuleList)/sizeof(pmModuleList[0]))));

    /* Configure for minimum OPP supported by SoC */
    ConfigMinimumOPP();

    /* Disable I2C Module after PMIC COnfiguration */
    disableModuleClock(CLK_I2C0, TRUE);

    /* Disable IRQ */
    IntMasterIRQDisable();

    if(SLEEP_MODE_DS0 == slpMode)
    {
        PowerDownConfig();
    }

    /* 
    **  Save A8 context 
    **  WFI
    **  Restore A8 context
    */
    saveRestoreContext(slpMode, memType, deviceVersion);
   
    /*
    ** Enable Timer3 clock before enabling Interrupts.
    ** Touch screen ISR is using Timer3 module.
    */
    enableModuleClock(CLK_TIMER3);

    /* Enable IRQ */
    IntMasterIRQEnable();
  
    enableModuleClock(CLK_I2C0);

    /* Restore OPP configuration */
    DemoOppChange(mpuOpp);

    /* Exclude MPU Clock from the disable list */
    for(index = 0; index < noOfElements; index++)
    {
        if(CLK_MPU_CLK == pmModuleList[index].module)
        {
            pmModuleList[index].select = FALSE;
        }
    }

    /* Device clock enable */
    enableSelModuleClock(pmModuleList,
                         (sizeof(pmModuleList)/sizeof(pmModuleList[0])));

    EDMAModuleClkConfig();

    /* disable wake source */
    if(SLEEP_STAND_BY_MODE == slpMode)
    {
        disableStandbyWakeSrc(dsData.dsDataBits.wakeSources);
    }
    else
    {
        disableWakeSource(dsData.dsDataBits.wakeSources);
    }

    PeripheralsContextRestore(slpMode, dsData.dsDataBits.wakeSources);

    PeripheralsResume();

    /*
    ** Print string name of the sleep mode.
    */
    ConsoleUtilsPrintf("\n\r%s", sleepModeStrMap[slpMode].str);

    /* Check the DS status */
    if(PM_CMD_FAIL == (readCmdStatus() & (PM_CMD_FAIL)))
    {
        ConsoleUtilsPrintf(" attempt failed");
    }
    else
    {
        ConsoleUtilsPrintf(" attempt passed");
    }

    /* Reset CM3 State Machine */
    configIPCRegs(dsDataM3reset);
    syncCm3();
	
    if(slpMode & SLEEP_MODE_DS1)
    {
        /* delay to reduce the frequency of sleep/wake cycle for DS1 */
        delay(500);
    }

    for(i = 0; i < 2;  i++)
    {
         x_data[i] = 0;
         y_data[i] = 0;
    }

    /* Reset sleep trigger flag */
    IsTSPress = 0;
}
