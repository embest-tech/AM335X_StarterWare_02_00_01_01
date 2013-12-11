/*
 * nandwriter.h
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
/* --------------------------------------------------------------------------
 * FILE        : nandwriter.h
 * PURPOSE     : Uboot main header file
 * PROJECT     : AM335X CCS FLASH WRITER
 * AUTHOR      : Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		 Vinay Agrawal (vinayagw@ti.com)
 * DATE        : 04-Jun-2007  
 * HISTORY
 * v1.00 - DJA - 04-Jun-2007
 * Completion (with support for DM6441 and DM6441_LV)
 * --------------------------------------------------------------------------
 */

#ifndef _NANDWRITER_H_
#define _NANDWRITER_H_

#include "tistdtypes.h"
#include "AM335X_nand.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/**************************************************
* Global Macro Declarations                       *
**************************************************/


/************************************************
* Global Typedef declarations                   *
************************************************/

typedef struct {
	Uint32 magicNum;    // Expected magic number
	Uint32 entryPoint;  // Entry point of the user application
	Uint32 numPage;     // Number of pages where boot loader is stored
	Uint32 block;       // Starting block number where User boot loader is stored
	Uint32 page;        // Starting page number where boot-loader is stored
	Uint32 ldAddress;   // Starting RAM address where image is to copied - XIP Mode
}AM335X_NANDWRITER_Boot;


/******************************************************
* Global Function Declarations                        *
******************************************************/
extern Uint32 nandwriter(AM335X_NAND_InfoHandle  hNandhandle);
/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif //_NANDWRITER_H_

/* --------------------------------------------------------------------------
 * HISTORY
 * v1.00 - DJA - 02-Nov-2007
 * Initial release
 * --------------------------------------------------------------------------
 */
