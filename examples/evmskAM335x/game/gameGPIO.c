/**
 *  \file   gameGPIO.c
 *
 *  \brief  This file contains definitions to disable and enable LCD backlight.
 */

/* Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 6288 of the EK-LM3S2965 Firmware Package.
 * This file is modified to make it work for StarterWare. */



//#include "hw_syscfg0_AM1808.h"
#include "soc_AM335x.h"
#include "hw_types.h"
#include "evmAM335x.h"
#include "gpio_v2.h"


extern void ConfigRasterGpioPins(void);

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/

/*****************************************************************************
**                       FUNCTION DEFINITION
*****************************************************************************/

/**
 * \brief   This function configures the pin 48 as output.
 *
 * \param   None.
 *
 * \return  None.
 */
void ConfigRasterDisplay(void)
{
    // /* The Local PSC number for GPIO is 3. GPIO belongs to PSC1 module.*/
    // PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON,
		     // PSC_MDCTL_NEXT_ENABLE);

    // ConfigRasterGpioPins();

    // /* Sets the pin 48(GP2[15]) as output.*/
    // GPIODirModeSet(SOC_GPIO_0_REGS, 48, GPIO_DIR_OUTPUT);
}

/**
 * \brief   This function disables the backlight.
 *
 * \param   None.
 *
 * \return  None.
 */
void DisableBackLight()
{
    /* write to GP2[15] to disable back light*/
    GPIOPinWrite(SOC_GPIO_0_REGS, 48, 0);
}

/**
 * \brief   This function enables the backlight.
 *
 * \param   None.
 *
 * \return  None.
 */
void EnableBackLight()
{
	/* write to GP2[15] to enable back light*/
    GPIOPinWrite(SOC_GPIO_0_REGS, 48, 1);
}
