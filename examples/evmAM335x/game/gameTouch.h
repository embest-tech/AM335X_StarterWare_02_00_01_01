/**
 * \file   demoTouch.h
 *
 * \brief  Function prototypes for the Touch interface
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


#ifndef _DEMOTOUCH_H_
#define _DEMOTOUCH_H_

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void InitTouchScreen(void);
extern unsigned int TouchDetect(void);
extern unsigned int TouchReleaseDetect(void);
extern void TouchCoOrdGet(int *pX, int *pY);
extern void ReadAxis(char mode, char*p1, char*p2);
extern void TouchIntEnable(void);
extern void TouchIntRegister(void);
extern void TouchEnable(void);
extern void StepEnable(void);
extern void StepDisable(void);

#endif
