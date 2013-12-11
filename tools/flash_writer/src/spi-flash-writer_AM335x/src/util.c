/*
 * util.c
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

// General type include
//#include "tistdtypes.h"

// This module's header file
#include "util.h"

/************************************************************
* Explicit External Declarations                            *
************************************************************/

/* extern __FAR__ Uint32 EXTERNAL_RAM_START, EXTERNAL_RAM_END;*/
#define EXTERNAL_RAM_START	(0xc0000000)		//REVISIT dynamic memory allocation in DDR
#define EXTERNAL_RAM_END	(EXTERNAL_RAM_START	+ 0x00020000)


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

// Global memory allocation pointer
static Uint32 currMemPtr;


/************************************************************
* Global Function Definitions                               *
************************************************************/

// DDR Memory allocation routines (for storing large data)
void *UTIL_getCurrMemPtr(void)
{
	return ((void *)currMemPtr);
}

// Mem copy routine
void UTIL_memcpy(int *dest, int *src, int size)
{
	int i;

	for (i=0;i<size;i++) {
		dest[i]=src[i];
	}
}


// Setup for an adhoc heap
void AM335X_UTIL_setCurrMemPtr(void *value)
{
	currMemPtr = (VUint32)value;
}

// Allocate memory from the ad-hoc heap
void *AM335X_UTIL_allocMem(Uint32 size)
{
	void *cPtr;
	Uint32 size_temp;

	// Ensure word boundaries
	size_temp = ((size + 4) >> 2 ) << 2;
	if((EXTERNAL_RAM_START + currMemPtr + size_temp) >
			((Uint32) EXTERNAL_RAM_END)) {
		return NULL;
	}

	cPtr = (void *) (((Uint32) EXTERNAL_RAM_START) + currMemPtr);
	currMemPtr += size_temp;
	return cPtr;
}

// Allocate memory from the ad-hoc heap
void *AM335X_UTIL_callocMem(Uint32 size)
{
	void *ptr;
	Uint8 *cPtr;
	Uint32 i;

	// Alloc the memory
	ptr = AM335X_UTIL_allocMem(size);

	// Clear the memory
	for (i=0,cPtr = ptr; i<size; i++) {
		cPtr[i] = 0x00;
	}
	
	return ptr;
}

// Simple wait loop - comes in handy.
void UTIL_waitLoop(Uint32 loopcnt)
{
	Uint32 i;

	for (i = 0; i<loopcnt; i++) {
		asm("   NOP");
	}
}

// Accurate n = ((t us * f MHz) - 5) / 1.65 
void UTIL_waitLoopAccurate (Uint32 loopcnt)
{
#if defined(_TMS320C6X)
	//  asm ("      SUB     B15, 8, B15     ");                 // Done by compiler 
	//  asm ("      STW     A4, *+B15[1]    ");                 // Done by compiler 
	asm ("      STW     B0, *+B15[2]    ");
	asm ("      SUB     A4, 24, A4      ");          // Total cycles taken by this function, with n = 0, including clocks taken to jump to this function 
	asm ("      CMPGT   A4, 0, B0       ");
	asm ("loop:                         ");
	asm (" [B0] B       loop            ");
	asm (" [B0] SUB     A4, 6, A4       ");          // Cycles taken by loop 
	asm ("      CMPGT   A4, 0, B0       ");
	asm ("      NOP     3               ");
	asm ("      LDW     *+B15[2], B0    ");
	//  asm ("      B       B3              ");                 // Done by compiler 
	//  asm ("      ADD     B15, 8, B15     ");                 // Done by compiler 
	//  asm ("      NOP     4     
#elif defined(_TMS320C5XX) || defined(__TMS320C55X__)
	UTIL_waitLoop(loopcnt);
#elif defined(_TMS320C28X)
	UTIL_waitLoop(loopcnt);
#elif (defined(__TMS470__) || defined(__GNUC__))
	UTIL_waitLoop(loopcnt);
#endif
}
