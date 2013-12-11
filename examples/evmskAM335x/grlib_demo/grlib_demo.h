/**
 * \file   grlib_demo.h
 *
 * \brief  This file contains macros and prototypes for graphics lib demo.
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

#ifndef _GRLIB_DEMO_H_
#define _GRLIB_DEMO_H_

#define PALETTE_SIZE         (32)
#define LCD_WIDTH            (480)
#define LCD_HEIGHT           (272)
#define NUM_ICONS            (4)
#define PB	                 (0)
#define CB	                 (1)
#define RB1                  (2)
#define RB2                  (3)

#define	PB_X_MIN	         (45)
#define	PB_Y_MIN	         (45)
#define	PB_X_MAX	         (45 + 75 - 1)
#define	PB_Y_MAX	         (45 + 75 - 1)

#define	CB_X_MIN	         (175)
#define	CB_Y_MIN	         (45)
#define	CB_X_MAX	         (175 + 75 - 1)
#define	CB_Y_MAX	         (45 + 75 - 1)

#define	RB1_X_MIN	         (275)
#define	RB1_Y_MIN	         (45)
#define	RB1_X_MAX	         (275 + 150 - 1)
#define	RB1_Y_MAX	         (45 + 35 - 1)

#define	RB2_X_MIN	         (275)
#define	RB2_Y_MIN	         (80)
#define	RB2_X_MAX	         (275 + 150 - 1)
#define	RB2_Y_MAX	         (80 + 35 - 1)


enum button_state
{
	PRESSED = 0,
	RELEASED = 1
};

struct icon
{
	enum button_state curState;
	tRectangle rect;
};

#endif


