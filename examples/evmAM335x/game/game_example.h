/**
 * \file  game_example.h
 *
 * \brief This file contains the declarations for game application.
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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
*/

#ifndef __GAME_EXAMPLE_H__
#define __GAME_EXAMPLE_H__

/* Header files inclusion */
#include "grlib.h"
#include "soc_AM335x.h"
#include "hw_types.h"
#include "game.h"
#include "gameTouch.h"
#include "gameCfg.h"

/* Macro Definitions*/
#define GAME_X	250
#define GAME_Y	100
#define GAME_W  250
#define GAME_H  180
#define NO_KEYS	5

#define GAME_W_b    (GAME_W / 2)
#define GAME_H_b    (GAME_H / 2)
#define GAME_W_midx (GAME_W - 1)
#define GAME_H_midx (GAME_H - 1)

/* Extern variables declaration */
extern volatile unsigned int flagA;
extern volatile unsigned int flagB;
extern tContext sContext0;
extern tContext sContext1;

/* Extern functions declaration */
extern void Delay(unsigned int ulCount);
extern void I2C0IfConfig(unsigned int slaveAddr, unsigned int speed);
extern void ConfigureAINTCIntI2C(void);


/* Button structure - Arrow keys */
struct buttons
{
	unsigned int id;
	unsigned int x_min;
	unsigned int y_min;
	unsigned int x_max;
	unsigned int y_max;
};

/* Local functions */
void getInputs(void);

#endif //__GAME_EXAMPLE_H__
