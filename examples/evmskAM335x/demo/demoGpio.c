/**
 * \file   demoGpio.c
 *
 * \brief  This file uses APIs of GPIO to turn ON and turn OFF user LED D1.
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

#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "interrupt.h"
#include "gpio_v2.h"
#include "delay.h"
#include "pin_mux.h"
#include "demoGpio.h"
#include "demoCfg.h"

/******************************************************************************
**              EXTERNAL VARIABLES
******************************************************************************/
extern volatile unsigned int IsTSPress;
extern unsigned int clickIdx;

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/

#define GPIO_INSTANCE_ADDRESS        (SOC_GPIO_1_REGS)
#define GPIO_INSTANCE_PIN_NUMBER     (4)

#define GPIO_WAKE_INSTANCE	     (SOC_GPIO_0_REGS)
#define GPIO_WAKE_PIN_NUM	     (30)
#define GPIO_LATENCY_PIN_NUM	     (2)

/******************************************************************************
**              INTERNAL FUNCTION DEFINITIONS
******************************************************************************/

void gpioLedInit()
{
    /* Enabling the GPIO module. */
    GPIOModuleEnable(GPIO_INSTANCE_ADDRESS);

    /* Set the specified pin as an output pin. */
    GPIODirModeSet(GPIO_INSTANCE_ADDRESS,
                   GPIO_INSTANCE_PIN_NUMBER,
                   GPIO_DIR_OUTPUT);
}

void gpioLedBlink()
{
    IsTSPress = 0;

    while(!(IsTSPress || clickIdx || UARTCharsAvail(SOC_UART_0_REGS)))
    {
        /* Driving GPIO0[7] pin to logic HIGH. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_HIGH);

        delay(500);

        /* Driving GPIO0[7] pin to logic LOW. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_LOW);

        delay(500);
    }
}

void configWakeGpio()
{
	/* Enabling the GPIO module. */
	GPIOModuleEnable(GPIO_WAKE_INSTANCE);
	
	/* Perform a module reset of the GPIO module. */
    //GPIOModuleReset(GPIO_WAKE_INSTANCE);
	
	/* Set the specified pin as an Input pin. */
    GPIODirModeSet(GPIO_WAKE_INSTANCE,
                   GPIO_WAKE_PIN_NUM,
                   GPIO_DIR_INPUT);
				   
	GPIOIntTypeSet(GPIO_WAKE_INSTANCE,
					GPIO_WAKE_PIN_NUM,
					GPIO_INT_TYPE_BOTH_EDGE);
					
	HWREG(GPIO_WAKE_INSTANCE + 0x34) = 0x40000000;
	HWREG(GPIO_WAKE_INSTANCE + 0x38) = 0x40000000;
	
	HWREG(GPIO_WAKE_INSTANCE + 0x44) = 0x40000000;
	
}

void enableGpioWake()
{
	/*	Clearing immediately after wake up is not consistent.
	**	Seems event is logged after clearing	*/
	/*	Clear wake interrupt	*/
	HWREG(GPIO_WAKE_INSTANCE + 0x2C) = 0x40000000;
	HWREG(GPIO_WAKE_INSTANCE + 0x30) = 0x40000000;

	/*	Configure GPIO in smart-dile and wake capable mode	*/
	HWREG(GPIO_WAKE_INSTANCE + GPIO_SYSCONFIG) = 0x1C;
}

void disableGpioWake()
{
	/*	Configure GPIO in Force-idle mode	*/
	HWREG(GPIO_WAKE_INSTANCE + GPIO_SYSCONFIG) = 0;
}

void toggleLatGpio()
{
	GPIOPinWrite(SOC_GPIO_1_REGS, GPIO_LATENCY_PIN_NUM, 
			((~GPIOPinRead(SOC_GPIO_1_REGS, GPIO_LATENCY_PIN_NUM)) >> GPIO_LATENCY_PIN_NUM) & 0x1);
}

/*
** This API Configures GPIO Pin for direction, debounce, interrupt and wake.
*/
void _DemoGpioPinConfig(unsigned int baseAddr, unsigned int pinNum,
                        tGPIOPinConfig *gpioPinConfig)
{
    /* Configure the Card Detection Pin as an Input/Output Pin. */
    GPIODirModeSet(baseAddr, pinNum, gpioPinConfig->dir);

    /* Enable Debouncing feature for the GPIO Pin. */
    GPIODebounceFuncControl(baseAddr, pinNum, gpioPinConfig->debouEnable);

    if(GPIO_DEBOUNCE_FUNC_ENABLE == gpioPinConfig->debouEnable)
    {
        /*
        ** Configure the Debouncing Time for all the input pins of
        ** the seleceted GPIO instance.
        */
        GPIODebounceTimeConfig(baseAddr, gpioPinConfig->debouTime);
    }

    if(!gpioPinConfig->intrEnable)
    {
        /* Enable interrupt for the specified GPIO Pin. */
        GPIOPinIntDisable(baseAddr, gpioPinConfig->intrLine, pinNum);
    }
    else
    {
        /*
        ** Configure interrupt generation on detection of a logic HIGH or
        ** LOW levels or a rising or a falling edge.
        */
        GPIOIntTypeSet(baseAddr, pinNum, gpioPinConfig->intrType);

        /* Enable interrupt for the specified GPIO Pin. */
        GPIOPinIntEnable(baseAddr, gpioPinConfig->intrLine, pinNum);
    }

    if(!gpioPinConfig->wakeEnable)
    {
        /* Enable wakeup generation for GPIO Module */
        GPIOWakeupGlobalDisable(baseAddr);

        /* Configure input GPIO Pin to wakeup */
        GPIOPinIntWakeUpDisable(baseAddr, gpioPinConfig->intrLine, pinNum);
    }
    else
    {
        /* Enable wakeup generation for GPIO Module */
        GPIOWakeupGlobalEnable(baseAddr);

        /* Configure input GPIO Pin to wakeup */
        GPIOPinIntWakeUpEnable(baseAddr, gpioPinConfig->intrLine, pinNum);
    }
}

/*
** Uart RXD ISR
*/
void gpioStdbyUartIsr(void)
{
    /* Clear interrupt status for the specified GPIO Pin */
    GPIOPinIntClear(GPIO_INST_BASE_UART_RXD, GPIO_UART_RXD_INTR_LINE,
                    GPIO_UART_RDX_PIN_NUM);
}

/*
** GPIO SW ISR
*/
void gpioStdbyGPIOIsr(void)
{
    /* Clear interrupt status for the specified GPIO Pin */
    GPIOPinIntClear(GPIO_INST_BASE_SW, GPIO_SW_INTR_LINE, GPIO_SW_PIN_NUM);
}

/*
** Configure an io pin for GPIO wake in standby
*/
void DemoGpioPinStandbySrcConfig(tIOPin *ioPin)
{
    /* Perform Pin Multiplexing for the IO Pad */
    GpioPinMuxSetup(ioPin->ioPadOff,
                    (ioPin->padConfig.slewRate |
                     ioPin->padConfig.mode |
                     ioPin->padConfig.type |
                     ioPin->padConfig.pullEnable |
                     ioPin->padConfig.pullSel));

    /* Configure GPIO Pin of Module */
    _DemoGpioPinConfig(ioPin->gpioBaseAddr, ioPin->pinNum,
                         &(ioPin->gpioConfig));

    /* Register the Interrupt Service Routine(ISR). */
    IntRegister(ioPin->intrNum, ioPin->gpioIsr); //ioPin->gpioIsr);

    /* Set the priority for the GPIO system interrupt in INTC. */
    IntPrioritySet(ioPin->intrNum, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the GPIO system interrupt in INTC. */
    IntSystemEnable(ioPin->intrNum);
}
