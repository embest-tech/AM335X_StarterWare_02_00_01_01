/**
 * \file  gameTimer.h
 *
 * \brief Part of game example application.
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


#ifndef _DEMOTIMER_H_
#define _DEMOTIMER_H_

/******************************************************************************
**                      VARIABLE DEFINITIONS
******************************************************************************/
extern unsigned int tmrFlag;
extern unsigned int tmrStepVary;

/******************************************************************************
**                      FUNCTION DEFINITIONS
******************************************************************************/
extern void Timer2IntRegister();
extern void Timer2Config(void);
extern void Timer2IntEnable(void);
extern void Timer2Start(void);
extern void Timer2Stop(void);

#endif
