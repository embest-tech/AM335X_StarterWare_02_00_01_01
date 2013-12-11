/*
 * AM335X_ecc.h
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
 * FILE        : ecc.h
 * PROJECT     : OMAP-L138 ROM Boot Loader
 * AUTHOR      : Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		 Vinay Agrawal (vinayagw@ti.com)
 * DESC        : Define ELM register address.
 * --------------------------------------------------------------------------
 */

#ifndef _AM335X_ECC_H_
#define _AM335X_ECC_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/
#define DEVICE_NAND_ECC_START_OFFSET (6)
// ELM Module Registers
#define	ELM_REVISION			(0x48080000)
#define	ELM_SYSCONFIG			(0x48080010)
#define	ELM_SYSSTATUS			(0x48080014)
#define	ELM_IRQSTATUS			(0x48080018)
#define	ELM_IRQENABLE			(0x4808001C)
#define	ELM_LOCATION_CONFIG		(0x48080020)
#define	ELM_PAGE_CTRL			(0x48080080)
#define	ELM_SYNDROME_FRAGMENT_0		(0x48080400)
#define	ELM_SYNDROME_FRAGMENT_1		(0x48080404)
#define	ELM_SYNDROME_FRAGMENT_2		(0x48080408)
#define	ELM_SYNDROME_FRAGMENT_3		(0x4808040C)
#define	ELM_SYNDROME_FRAGMENT_4		(0x48080410)
#define	ELM_SYNDROME_FRAGMENT_5		(0x48080414)
#define	ELM_SYNDROME_FRAGMENT_6		(0x48080418)
#define	ELM_LOCATION_STATUS		(0x48080800)
#define	ELM_ERROR_LOCATION_0		(0x48080880)
#define	ELM_ERROR_LOCATION_1		(0x48080884)
#define	ELM_ERROR_LOCATION_2		(0x48080888)
#define	ELM_ERROR_LOCATION_3		(0x4808088C)
#define	ELM_ERROR_LOCATION_4		(0x48080890)
#define	ELM_ERROR_LOCATION_5		(0x48080894)
#define	ELM_ERROR_LOCATION_6		(0x48080898)
#define	ELM_ERROR_LOCATION_7		(0x4808089C)
#define	ELM_ERROR_LOCATION_8		(0x480808A0)
#define	ELM_ERROR_LOCATION_9		(0x480808A4)
#define	ELM_ERROR_LOCATION_10		(0x480808A8)
#define	ELM_ERROR_LOCATION_11		(0x480808AC)
#define	ELM_ERROR_LOCATION_12		(0x480808B0)
#define	ELM_ERROR_LOCATION_13		(0x480808B4)
#define	ELM_ERROR_LOCATION_14		(0x480808B8)
#define	ELM_ERROR_LOCATION_15		(0x480808BC)

// Define the syndrome Polynomial (Any value between 0-7)
#define ELM_DEFAULT_POLY                (0)

/************************************************************
* Global Variable Declarations                              *
************************************************************/

/******************************************************
* Global Typedef declarations                         *
******************************************************/
extern AM335X_NAND_BB_InfoObj AM335X_DEVICE_NAND_BB_info;
extern AM335X_NAND_ECC_InfoObj AM335X_DEVICE_NAND_ECC_info;
extern Uint32 AM335X_Device_setECC( Uint32 busWidth, Uint32 eccType);

/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_H_
