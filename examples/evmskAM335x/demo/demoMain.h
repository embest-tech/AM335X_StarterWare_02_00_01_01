/**
 * \file  demoMain.h
 *
 * \brief Part of StarterWare demo application
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

#ifndef _DEMOMAIN_H_
#define _DEMOMAIN_H_


#define BACK_TILT		0x1
#define FORWARD_TILT	0x2
#define LEFT_TILT		0x3
#define RIGHT_TILT		0x4

/******************************************************************************
**                            TYPE DEFINITIONS
*******************************************************************************/
/*
** Touch Specifications of an image.
*/
typedef struct touchSpec
{
    int const *coOrd;
    void (*action)(void);
    char *helpStr;
}TOUCHSPEC;

/*
** Context of an Image
*/
typedef struct imageContext
{
    unsigned int const *pImageAddr;
    
    /* The number of icons in the image */
    unsigned int numIcon; 

    TOUCHSPEC const *touchSpec;
}IMAGECONTEXT;

/******************************************************************************
**                      EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern IMAGECONTEXT contextInfo[];
extern volatile unsigned int imageCount;
extern unsigned int clickIdx;
extern unsigned int deviceVersion;

/******************************************************************************
**                      EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void initTempSensor(void);
extern void updateTemperature(unsigned int *intVal, unsigned int *floatVal);
extern void UpdateUartConsoleHelp(void);
void initAccelerometer(void);
extern void getAcceleroPos(int* tiltDir, int* tiltDeg);

#endif
