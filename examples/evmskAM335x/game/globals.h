/**
 * \file  globals.h
 *
 * \brief Shared configuration and global variables.
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

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include "game_example.h"

/* The clock rate for the SysTick interrupt.  All events in the application
 * occur at some fraction of this clock rate. */
#define CLOCK_RATE              300

/* A set of flags used to track the state of the application. */
#define FLAG_CLOCK_TICK         0           /* A timer interrupt has occurred */
#define FLAG_CLOCK_COUNT_LOW    1           /* The low bit of the clock count */
#define FLAG_CLOCK_COUNT_HIGH   2           /* The high bit of the clock count */
#define FLAG_UPDATE             3           /* The display should be updated */
#define FLAG_BUTTON             4           /* Debounced state of the button */
#define FLAG_DEBOUNCE_LOW       5           /* Low bit of the debounce clock */
#define FLAG_DEBOUNCE_HIGH      6           /* High bit of the debounce clock */
#define FLAG_BUTTON_PRESS       7           /* The button was just pressed */

/* The speed of the processor. */
extern unsigned int g_ulSystemClock;

/* Storage for a local frame buffer. */
extern unsigned char g_pucFrame[(GAME_W*GAME_H/2)+6+(16*3)];

/* The set of switches that are currently pressed. */
extern unsigned char g_ucSwitches;

/* A set of flags used to track the state of the application. */
extern unsigned int g_ulFlags;

/* The array contatining TI Logo image. */
extern const unsigned char g_pucTILogo128x96[];

#endif /* __GLOBALS_H__ */
