/**
 *  \file   gpioLCDBacklight.c
 *
 *  \brief  This application uses a GPIO pin to switch the backlight of the
 *          LCD panel ON and OFF.
 *
 *          Application Configurations:
 *
 *              Modules Used:
 *                  GPIO0
 *
 *              Configuration Parameters:
 *                  None
 *
 *          Application Use Case:
 *              1) The GPIO pin GPIO0[7] is used as an output pin.
 *              2) This pin is alternately driven HIGH and LOW. A finite delay
 *                 is given to retain the pin in its current state.
 *
 *          Running the example:
 *              On running the example, the backlight of the LCD screen would
 *              be seen turning ON and OFF alternatively.
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*
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
#include "evmAM335x.h"
#include "gpio_v2.h"

/*****************************************************************************
**                   INTERNAL MACRO DEFINITIONS
*****************************************************************************/
#define  GPIO_INSTANCE_PIN_NUMBER      (7u)

/*****************************************************************************
**                   INTERNAL FUNCTION PROTOTYPES
*****************************************************************************/
static void Delay(volatile unsigned int count);

/*****************************************************************************
**                        FUNCTION DEFINITIONS
*****************************************************************************/

int main(void)
{
    /* unsigned int count = 0; */

    /* Configuring the functional clock for GPIO0 instance. */
    GPIO0ModuleClkConfig();

    /* Doing a pin multiplexing and selecting GPIO0[7] for use. */    
    GPIO0Pin7PinMuxSetup();

    /* Enabling the GPIO module. */
    GPIOModuleEnable(SOC_GPIO_0_REGS);

    /* Resetting the GPIO module. */
    GPIOModuleReset(SOC_GPIO_0_REGS);

    /* Configuring GPIO0[7] pin as an output pin. */ 
    GPIODirModeSet(SOC_GPIO_0_REGS,
                   GPIO_INSTANCE_PIN_NUMBER,
                   GPIO_DIR_OUTPUT);
    while(1)
    {
        /* Driving GPIO0[7] pin to logic HIGH. */    
        GPIOPinWrite(SOC_GPIO_0_REGS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_HIGH);

        Delay(0xFFFFF);

        /* Driving GPIO0[7] pin to logic LOW. */
        GPIOPinWrite(SOC_GPIO_0_REGS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_LOW);
        
        Delay(0xFFFFF);
    }
}

/*
** This function provides a delay for the specified count value.
*/

static void Delay(volatile unsigned int count)
{
    while(count--);
}
