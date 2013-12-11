/**
 * \file    demoLedIf.c
 *
 * \brief   This file contains LED interface related functions.
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

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "demoCfg.h"
#include "gpio_v2.h"
#include "demoLedIf.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LED_TOGGLE                           (0x01u)
#define LED_OFF                              (GPIO_PIN_LOW)

/*******************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static unsigned char ledState = GPIO_PIN_LOW;

/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Configures the I2C for the LED interface
*/
void LedIfConfig(void)
{
    /* Enabling the GPIO module. */
    GPIOModuleEnable(SOC_GPIO_1_REGS);

    /* Resetting the GPIO module. */
    GPIOModuleReset(SOC_GPIO_1_REGS);

    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_DIR_OUTPUT);
}

/*
** Toggle the LED state
*/
void LedToggle(void)
{
    ledState ^= LED_TOGGLE;  
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, ledState); 
}

/*
** Turn the  LED Off.
*/
void LedOff(void)
{
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
}
/*
** Turn the  LED Off.
*/
void LedOn(void)
{
    GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
}

/****************************** End of file **********************************/



