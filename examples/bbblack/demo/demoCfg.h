/**
 * \file   demoCfg.h
 * 
 * \brief  Configuration values for the Demo Application  
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

#ifndef _DEMOCFG_H_
#define _DEMOCFG_H_

/*
** SoC Parameters
*/

#define EDMA_BASE_ADDR              (SOC_EDMA30CC_0_REGS)
#define EDMATC0_BASE_ADDR           (0x49800000)
#define EDMATC1_BASE_ADDR           (0x49900000)
#define EDMATC2_BASE_ADDR           (0x49A00000)
#define CPSW_CPDMA_BASE_ADDR        (SOC_CPSW_CPDMA_REGS)
#define CPSW_WR_BASE_ADDR           (SOC_CPSW_WR_REGS)
#define CPSW_MDIO_BASE_ADDR         (SOC_CPSW_MDIO_REGS)
#define CPSW_ALE_BASE_ADDR          (SOC_CPSW_ALE_REGS)
#define CPSW_SS_BASE_ADDR           (SOC_CPSW_SS_REGS)
#define CPSW_PORT1_BASE_ADDR        (SOC_CPSW_PORT_1_REGS)
#define CPSW_PORT2_BASE_ADDR        (SOC_CPSW_PORT_2_REGS)
#define CPSW_SLIVER1_BASE_ADDR      (SOC_CPSW_SLIVER_1_REGS)
#define CPSW_SLIVER2_BASE_ADDR      (SOC_CPSW_SLIVER_2_REGS)
#define CPSW_CPPI_RAM_BASE_ADDR     (SOC_CPSW_CPPI_RAM_REGS)
#define MMCSD_BASE_ADDR             (SOC_MMCHS_0_REGS)
#define GPIO1_BASE_ADDR             (SOC_GPIO_1_REGS)
#define DMTIMER2_BASE_ADDR          (SOC_DMTIMER_2_REGS)
#define DMTIMER3_BASE_ADDR          (SOC_DMTIMER_3_REGS)
#define DMTIMER4_BASE_ADDR          (SOC_DMTIMER_4_REGS)
#define DMTIMER6_BASE_ADDR          (SOC_DMTIMER_6_REGS)
#define DMTIMER7_BASE_ADDR          (SOC_DMTIMER_7_REGS)
#define MAILBOX_BASE_ADDR           (0x480C8000)
#define I2C0_BASE_ADDR              (SOC_I2C_0_REGS)
#define USB_BASE_ADDR               (USBSS_BASE)
#define GPMC_BASE_ADDR              (SOC_GPMC_0_REGS)
#define MMCSD_BASE_ADDR             (SOC_MMCHS_0_REGS)

/* Address of TPS65910A (PMIC - control)  over I2C0. */
#define  PMIC_CNTL_I2C_SLAVE_ADDR  		(0x24)

/* Address of TPS65910A (PMIC - SR)  over I2C0. */
#define  PMIC_SR_I2C_SLAVE_ADDR     		(0x12)

/*
** The indexes of Each Icon in the html page
*/
#define MENU_IDX_MAIN                          (0u)
#define MENU_IDX_WWW                           (1u)
#define MENU_IDX_RTC                           (2u)
#define MENU_IDX_LED                           (3u)
#define MENU_IDX_TIMER                         (4u)
#define MENU_IDX_SD                            (5u)
#define MENU_IDX_PM                            (6u)

/*
** Number of Icons in the images
*/
#define NUM_ICON_DFLT                          (3u)
#define NUM_ICON_MENU                          (6u)
#define NUM_ICON_RTC                           (4u)
#define NUM_ICON_WWW                           (4u)
#define NUM_ICON_GPIO                          (NUM_ICON_DFLT)
#define NUM_ICON_TMR                           (NUM_ICON_DFLT)
#define NUM_ICON_MMCSD                         (4u)
#define NUM_ICON_PM                            (8u)

/*
** Counter overflow value for 20 seconds
*/
#define  TIMER_OVRFLW_20_SECOND_24MHZ     (0xE363C7FFu)
/* For timer1 prescaler is configured as 2 */
#define  TIMER_OVRFLW_20_SECOND_16KHZ     (0xFFFAFFFFu)

/*
** Sleep Modes
*/
#define  SLEEP_MODE_DS0                   (0x01u)
#define  SLEEP_STAND_BY_MODE              (0x04u)
#define  SLEEP_MODE_DS1                   (0x02u)

/*
** Wake Source
*/
#define WAKE_SOURCE_UART                  (0x0010u)
#define WAKE_SOURCE_TMR                   (0x0008u)
#define WAKE_SOURCE_MPU                   (0x0800u)

/* UART RXD - GPIO Configuration */
#define  GPIO_INST_BASE_UART_RXD                (SOC_GPIO_1_REGS)
#define  GPIO_UART_RXD_INTR_LINE                (GPIO_INT_LINE_2)
#define  GPIO_UART_RDX_PAD_OFFSET               (GPIO_1_10)
#define  GPIO_UART_RDX_PIN_NUM                  (10u)
#define  GPIO_UART_RXD_SYS_INT_NUM              (SYS_INT_GPIOINT1B)

/* IO Pad Power Down Configuration */
#define IOPAD_GPIO                        (CONTROL_CONF_PULLUDDISABLE | \
                                           CONTROL_CONF_MUXMODE(7))
#define IOPAD_GPIO_PULLDN_RXACTIVE        (CONTROL_CONF_RXACTIVE | \
                                           CONTROL_CONF_MUXMODE(7))

#endif
