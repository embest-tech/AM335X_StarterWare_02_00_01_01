/*
 * AM335X ECC.C 
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
* FILE        : AM335X_ecc.c
* PROJECT     : TI Boot and Flash Utils
* AUTHOR      : Mansoor Ahamed (mansoor.ahamed@ti.com)
* 		 Vinay Agrawal (vinayagw@ti.com)
* DESC        : This file provides low-level init functions.
* --------------------------------------------------------------------------
*/


/************************************************************
* Include Files                                             *
************************************************************/

// General type include
#include "tistdtypes.h"
#include "string.h"
// Utility functions
#include "util.h"

// This module's header file
#include "AM335X_device.h"

#include "AM335X_nand.h"

// Debug functions for non-ROMed version
#include "debug.h"

#include "AM335X_ecc.h"

/************************************************************
* Global Variable Definitions                               *
************************************************************/
// Bad Block mark functions
static void DEVICE_NAND_BB_markSpareBytes(AM335X_NAND_InfoHandle hNandInfo,
Uint8 *spareBytes);
static Uint32 DEVICE_NAND_BB_checkSpareBytes(AM335X_NAND_InfoHandle hNandInfo,
Uint8 *spareBytes);

AM335X_NAND_BB_InfoObj AM335X_DEVICE_NAND_BB_info = {
TRUE,
TRUE,
&(DEVICE_NAND_BB_markSpareBytes),
&(DEVICE_NAND_BB_checkSpareBytes)
};

// ECC functions
static void ECC_Enable(AM335X_NAND_InfoHandle hNandInfo);
static void ECC_disable(AM335X_NAND_InfoHandle hNandInfo);
static void ECC_store(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes,
Uint8 opNum, Uint8 *calcECC);
// Functions for Hamming ECC Algo
static void DEVICE_NAND_ECC_BCH_calculate(AM335X_NAND_InfoHandle hNandInfo,
Uint8 *data, Uint8 *calcECC);
static void DEVICE_NAND_ECC_BCH_calculate_BE(AM335X_NAND_InfoHandle hNandInfo,
Uint8 *data, Uint8 *calcECC);
static Uint32 DEVICE_NAND_ECC_BCH_correct(AM335X_NAND_InfoHandle hNandInfo,
Uint32 pageLoc, Uint8 *spareBytes, Uint8 *data, Uint32 opNum);
static void DEVICE_NAND_ECC_BCH_readset(void);
static void DEVICE_NAND_ECC_BCH_writeset(void);

static Uint32 ELM_CORRECT_errors(Uint8 *data, Uint32 numError, Uint32 *errorLoc);
static Uint32 ELM_CheckErrors(Uint32 *numError, Uint32 *errorloc, Uint8 * syndrome);

// Function for HAM Algo
static void DEVICE_NAND_ECC_HAM_calculate(AM335X_NAND_InfoHandle hNandInfo,
Uint8 *data, Uint8 *calcECC);
static Uint32 DEVICE_NAND_ECC_HAM_correct(AM335X_NAND_InfoHandle hNandInfo,
Uint32 pageLoc, Uint8 *spareBytes, Uint8 *data, Uint32 opNum);
static void DEVICE_NAND_ECC_HAM_readset(void);
static void DEVICE_NAND_ECC_HAM_writeset(void);

AM335X_NAND_ECC_InfoObj AM335X_DEVICE_NAND_ECC_info;
AM335X_NAND_ECC_InfoObj *NAND_ECC_Info_Handle = &(AM335X_DEVICE_NAND_ECC_info);

/************************************************************
* Local Function Definitions                                *
************************************************************/


void DEVICE_NAND_BB_markSpareBytes(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes) {
	Uint32 i, j;

	// Mark all the free bytes (non-ECC bytes) as 0x00
	for (j = 0; j < hNandInfo->numOpsPerPage; j++) {
		for (i = 0; i < DEVICE_NAND_ECC_START_OFFSET; i++) {
			spareBytes[i + hNandInfo->spareBytesPerOp * j] = 0x00;
			}
		}
}

// Function to determine if the spare bytes indicate a bad block

static Uint32 DEVICE_NAND_BB_checkSpareBytes(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes) {
	Uint32 i, j;

	// Check all the free bytes (non-ECC bytes) for 0xFF
	for (j = 0; j < hNandInfo->numOpsPerPage; j++) {
		for (i = 0; i < DEVICE_NAND_ECC_START_OFFSET; i++) {
			if (spareBytes[i + hNandInfo->spareBytesPerOp * j] != 0xFF)
				return E_FAIL;
		}
	}
	return E_PASS;
}

static void ECC_enable(AM335X_NAND_InfoHandle hNandInfo) {
	Uint32 *ptr;

	ptr = (Uint32 *) GPMC_ECC_CONTROL;
	*ptr = (*ptr | 0x00000100u); // Clear the ECC outputs.
	*ptr &= ~0xF; 
	*ptr |= 0x1;
	ptr = (Uint32 *) GPMC_ECC_CONFIG;
	*ptr = (*ptr | 0x1); // Enable the ECC (ECCENABLE = 1)
}

static void ECC_disable(AM335X_NAND_InfoHandle hNandInfo) {
	Uint32 *ptr;

	ptr = (Uint32 *) GPMC_ECC_CONFIG;
	*ptr = (*ptr & 0xFFFFFFFEu); // Disable the ECC (ECCENABLE = 0)
}

static void ECC_store(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes,
		Uint8 opNum, Uint8 *calcECC) {
	Uint32 i = 0;

	for (i = 0; i < AM335X_DEVICE_NAND_ECC_info.storedECCByteCnt; i++) {
		spareBytes[i + hNandInfo->hEccInfo->offset +
			(AM335X_DEVICE_NAND_ECC_info.storedECCByteCnt * opNum)] = *calcECC;
		calcECC++;
	}
}

Uint32 AM335X_Device_setECC(Uint32 busWidth, Uint32 eccType) {
	NAND_ECC_Info_Handle->fxnEnable = &(ECC_enable);
	NAND_ECC_Info_Handle->fxnDisable = &(ECC_disable);
	NAND_ECC_Info_Handle->fxnStore = &(ECC_store);

	switch (eccType) {
		// For BCH 8 bit ECC scheme.
		case 1:
			// Configure the ECC register
			//GPMC_ECC_CONFIG: ECCALGORITHM (ECH), ECCBCHTSEL(8bit), ECCWRAPMODE(1), ECC16B(hNandInfo->busWidth),ECCTOPSECTOR(0),ECCCS(0)
			GPMC_Write(GPMC_ECC_CONFIG, 0, (0x00011100) | (busWidth << 7));
			//GPMC_ECC_CONTROL : ECCCLEAR(1), ECCPOINTER(1)
			GPMC_Write(GPMC_ECC_CONTROL, 0, 0x00000101u);
			// GPMC_ECC_SIZE_CONFIG: ECCSIZE0(0bytes), ECCSIZE1 (14byte), ECC1RESULTSIZE(Size0)
			//GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, 0x03800000);
			GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, 0x07000000);
			// Configure the ELM register
			// ELM_SYSCONFIG : NO IDLE , FREE Clock Running
			GPMC_Write(ELM_SYSCONFIG, 0, 0x00000108u);
			//ELM_IRQSTATUS : Clear Interrupt of given Syndrome poly register.
			GPMC_Write(ELM_IRQSTATUS, 0, (0x1 << ELM_DEFAULT_POLY));
			//ELM_IRQENABLE : Enable interrupt for Syndrome Polynomial
			GPMC_Write(ELM_IRQENABLE, 0, (0x1 << ELM_DEFAULT_POLY));
			// ELM_LOCATION_CONFIG :  Use Maximum Size and 8 bit BCH Algo
			GPMC_Write(ELM_LOCATION_CONFIG, 0, 0x07FF0001);
			// ELM_PAGE_CTRL: Continous mode for given Syndorme Polynomial
			GPMC_Write(ELM_PAGE_CTRL, 0, (0x1 << ELM_DEFAULT_POLY));
			//Assing BCH functions accroding to ECC scheme type
			NAND_ECC_Info_Handle->ECCEnable = TRUE;
			NAND_ECC_Info_Handle->storedECCByteCnt = 14;
			NAND_ECC_Info_Handle->offset = 2;
			NAND_ECC_Info_Handle->fxnCalculate = &(DEVICE_NAND_ECC_BCH_calculate);
			NAND_ECC_Info_Handle->fxnCorrect = &(DEVICE_NAND_ECC_BCH_correct);
			NAND_ECC_Info_Handle->fxnReadset = &(DEVICE_NAND_ECC_BCH_readset);
			NAND_ECC_Info_Handle->fxnWriteset = &(DEVICE_NAND_ECC_BCH_writeset);
			printf("\n  Set the BCH 8 bit ECC scheme  ");
			break;
			
			// Cofigure ECC for HAM scheme
		case 2:
			// Configure the ECC register
			//GPMC_ECC_CONFIG: ECCALGORITHM (HAM),8 columns, ECCCS(0),ECC disenable
			GPMC_Write(GPMC_ECC_CONFIG, busWidth << 7  , 0);
			//GPMC_ECC_CONTROL : ECCCLEAR(1), ECCPOINTER(1)
			GPMC_Write(GPMC_ECC_CONTROL, 0, 0x00000101u);
			// GPMC_ECC_SIZE_CONFIG: ECCSIZE0(512bytes), ECCSIZE1 (512 byte), ECC1RESULTSIZE(Size0)
			GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, 0xFF << 22 | 0xFF << 12);
			//Assing HAM functions accroding to ECC scheme type
			NAND_ECC_Info_Handle->ECCEnable = TRUE;
			NAND_ECC_Info_Handle->storedECCByteCnt = 3;
			NAND_ECC_Info_Handle->offset = 40;
			NAND_ECC_Info_Handle->fxnCalculate = &(DEVICE_NAND_ECC_HAM_calculate);
			NAND_ECC_Info_Handle->fxnCorrect = &(DEVICE_NAND_ECC_HAM_correct);
			NAND_ECC_Info_Handle->fxnReadset = &(DEVICE_NAND_ECC_HAM_readset);
			NAND_ECC_Info_Handle->fxnWriteset = &(DEVICE_NAND_ECC_HAM_writeset);
			printf("\n  Set the HAM ECC scheme  ");
			break;
			
		default:
			printf("Wrong ECC scheme selected \n");
			return E_FAIL;
	}
	return E_PASS;
}

static void DEVICE_NAND_ECC_BCH_calculate(AM335X_NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *syndrome) {
	Uint32 *ptr = (Uint32*)GPMC_BCH_RESULT_3;

	syndrome[0] = *ptr & 0xFF;
   
	ptr--;

	syndrome[1] = (*ptr >> 24) & 0xFF;
	syndrome[2] = (*ptr >> 16) & 0xFF;
	syndrome[3] = (*ptr >> 8) & 0xFF;
	syndrome[4] = *ptr & 0xFF;

	ptr--;
	syndrome[5] = (*ptr >> 24) & 0xFF;
	syndrome[6] = (*ptr >> 16) & 0xFF;
	syndrome[7] = (*ptr >> 8) & 0xFF;
	syndrome[8] = *ptr & 0xFF;

	ptr--;
	syndrome[9] = (*ptr >> 24) & 0xFF;
	syndrome[10] = (*ptr >> 16) & 0xFF;
	syndrome[11] = (*ptr >> 8) & 0xFF;
	syndrome[12] = *ptr & 0xFF;


	// Only first 14 bytes are useful for ECC data
	/* syndrome[14] = (GPMC_BCH_RESULT_3 >> 16) & 0xFF;
	 * syndrome[15] = (GPMC_BCH_RESULT_3 >> 24) & 0xFF;
	 */

}

static void DEVICE_NAND_ECC_BCH_calculate_BE(AM335X_NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *syndrome) {
	Uint32 *ptr = (Uint32*)GPMC_BCH_RESULT_0;


	syndrome[0] = *ptr & 0xFF;
	syndrome[1] = (*ptr >> 8) & 0xFF;
	syndrome[2] = (*ptr >> 16) & 0xFF;
	syndrome[3] = (*ptr >> 24) & 0xFF;

	ptr++;
	syndrome[4] = *ptr & 0xFF;
	syndrome[5] = (*ptr >> 8) & 0xFF;
	syndrome[6] = (*ptr >> 16) & 0xFF;
	syndrome[7] = (*ptr >> 24) & 0xFF;

	ptr++;
	syndrome[8] = *ptr & 0xFF;
	syndrome[9] = (*ptr >> 8) & 0xFF;
	syndrome[10] = (*ptr >> 16) & 0xFF;
	syndrome[11] = (*ptr >> 24) & 0xFF;

	ptr++;	 
	syndrome[12] = *ptr & 0xFF;

	// Only first 14 bytes are useful for ECC data

	/* syndrome[14] = (GPMC_BCH_RESULT_3 >> 16) & 0xFF;
	 * syndrome[15] = (GPMC_BCH_RESULT_3 >> 24) & 0xFF;
	 */
}

static void DEVICE_NAND_ECC_BCH_writeset()
{
	Uint32 size0 = 0;
	Uint32 size1 = 26;

	// GPMC_ECC_SIZE_CONFIG: ECCSIZE0(0bytes), ECCSIZE1 (14byte), ECC1RESULTSIZE(Size0)
	/* set for for nibbles and not for bytes. BCH syndrome counted as nibbles */

	/*  GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, 0x03800000); */
	GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, ((size1 << 22) | (size0 << 12)));
}

static void DEVICE_NAND_ECC_BCH_readset(){
	Uint32 size0 = 26;
	Uint32 size1 = 2;

	// GPMC_ECC_SIZE_CONFIG: ECCSIZE0(13bytes), ECCSIZE1 (1byte), ECC1RESULTSIZE(Size0)
	/* set for for nibbles and not for bytes. BCH syndrome counted as nibbles */
	/* GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, 0x00006000); */

	GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, ((size1 << 22) | (size0 << 12)));
}

static Uint32 DEVICE_NAND_ECC_BCH_correct(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 pageLoc, Uint8 *spareBytes, Uint8 *data, Uint32 opNum) {
	Uint8 syndrome[16];
	Uint32 numerror;
	Uint32 errorloc[8];

	// sapreBytes is of no use therefore used as dump array
	// Read the spare bytes of the given operation

	AM335X_NAND_readBCHSpare(hNandInfo, pageLoc, spareBytes, opNum, 1);
	
	//Stop the ECC engine.
	ECC_disable(hNandInfo);

	// Get the syndrome from GPMC BCH ECC.
	DEVICE_NAND_ECC_BCH_calculate_BE(hNandInfo, spareBytes, syndrome);

	// Get the number of errors and location by feeding the syndrome in ELM
	if (ELM_CheckErrors(&numerror, errorloc, syndrome) != E_PASS) {
		// Error found in data are not fixable
		return E_FAIL;
	}

	// IF Number of error found >0 and correctable
	if (numerror > 0) {
		ELM_CORRECT_errors(data, numerror, errorloc);
	}
	return E_PASS;
}

static Uint32 ELM_CheckErrors(Uint32 *numError, Uint32 *errorloc, Uint8 * syndrome) {
	Uint32 *ptr;
	// Get the offset for fragment register
	Uint32 polyoffset = (0x40 * ELM_DEFAULT_POLY);
	Uint32 i = 0;

	ptr = (Uint32 *) (ELM_SYNDROME_FRAGMENT_0 + polyoffset);
	*ptr = syndrome[0] | (syndrome[1] << 8) | (syndrome[2] << 16) | (syndrome[3] << 24);

	ptr = (Uint32 *) (ELM_SYNDROME_FRAGMENT_1 + polyoffset);
	*ptr = syndrome[4] | (syndrome[5] << 8) | (syndrome[6] << 16) | (syndrome[7] << 24);
	
	ptr = (Uint32 *) (ELM_SYNDROME_FRAGMENT_2 + polyoffset);
	*ptr = syndrome[8] | (syndrome[9] << 8) | (syndrome[10] << 16) | (syndrome[11] << 24);

	ptr = (Uint32 *) (ELM_SYNDROME_FRAGMENT_3 + polyoffset);
	*ptr = syndrome[12] | (syndrome[13] << 8) | (syndrome[14] << 16) | (syndrome[15] << 24);

	// Start processing the syndrome
	ptr = (Uint32 *) (ELM_SYNDROME_FRAGMENT_6 + polyoffset);
	*ptr = 0x10000;

	// Wait for the processing is over.
	ptr = (Uint32 *) (ELM_IRQSTATUS);
	while ((*ptr & (0x1 << ELM_DEFAULT_POLY)) == 0) {
		; // wait
	}

	//Clear the current interupt
	*ptr = *ptr | (0x1 << ELM_DEFAULT_POLY);

	// Get location status
	ptr = (Uint32 *) (ELM_LOCATION_STATUS + (0x100 * ELM_DEFAULT_POLY));

	if ((*ptr & (0x1 << 8)) == 0) {
		// Unable to fix the errors.
		return E_FAIL;
	} else {
		*numError = (*ptr & 0x1F);
	}

	// if error found then read out the error location
	if (*numError > 0) {
		//Get the error location register address
		ptr = (Uint32 *) (ELM_ERROR_LOCATION_0 + (0x100 * ELM_DEFAULT_POLY));
		for (i = 0; i < *numError; i++) {
			errorloc[i] = *ptr;
			ptr++;
		}
		return E_PASS;
	}
	return E_PASS; // No Error found
}

static Uint32 ELM_CORRECT_errors(Uint8 *data, Uint32 numError, Uint32 *errorLoc) {
	Uint8 count = 0;
	Uint32 errorBytePos;
	Uint32 errorBitMask;
	// BCH 8 Bit contain 26 Nibbles.
	Uint32 lastBit = (26 * 4) - 1;

	/* Flip all bits as specified by the error location array. */
	/* FOR( each found error location ) */
	for (count = 0; count < numError; count++) {
		if (errorLoc[count] > lastBit) {
			//Remove the ECC spare bits from correction.
			errorLoc[count] -= (lastBit + 1);
			//Offset bit in data region
			errorBytePos = (512 * 8)-(errorLoc[count] / 8) - 1;
			//Error Bit mask
			errorBitMask = 0x1 << (errorLoc[count] % 8);
			// Toggle the error bit to make the correction.
			data[errorBytePos] ^= errorBitMask;
		}
	}
	return E_PASS;
}

// HAM ECC scheme funtions definations.

static void DEVICE_NAND_ECC_HAM_calculate(AM335X_NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *calcECC) {
	Uint32 val;

	val = *((Uint32 *) GPMC_ECC1_RESULT);
	calcECC[0] = val & 0xFF;
	calcECC[1] = (val >> 16) & 0xFF;
	calcECC[2] = ((val >> 8) & 0x0F) | ((val >> 20) & 0xF0);
}

static Uint32 DEVICE_NAND_ECC_HAM_correct(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 pageLoc, Uint8 *spareBytes, Uint8 *data, Uint32 opNum) {
	Uint8 odd[2], even[2];
	Uint8 readECC[16];

	//Stop the ECC engine.
	ECC_disable(hNandInfo);

	// Get the HAM bytes from GPMC
	DEVICE_NAND_ECC_HAM_calculate(hNandInfo, data, readECC);

	// Set spareBytes pointer correponding to read data.
	spareBytes = spareBytes + hNandInfo->hEccInfo->offset + (hNandInfo->hEccInfo->storedECCByteCnt * opNum);
	even[0] = (*spareBytes) ^ (readECC[0]);
	spareBytes++;
	odd[0] = (*spareBytes) ^ (readECC[1]);
	spareBytes++;
	even[1] = ((*spareBytes) ^ (readECC[2])) & 0xF;
	odd[1] = (((*spareBytes) ^ (readECC[2])) >> 4) & 0xF;

	if (((odd[0] ^ even[0]) == 0) & ((odd[1] ^ even[1] == 0))) {
		return E_PASS; // No error is detected;
	} else {
		if (((odd[0] ^ even[0]) == 0xFF) & ((odd[1] ^ even[1]) == 0x07)) {
			// Error is found and it can be fixed (1 bit error)
			int col, row;
			// last three bits of odd parity will give column number
			col = odd[0] & 0x7;
			row = (odd[1] << 5) | (odd[0] >> 3);
			data = data + row - 1;
			// toggle the col bit of data
			*data ^= (0x1 << col);
		} else {
			return E_FAIL;
		}
	}
	return E_PASS;
}

static void DEVICE_NAND_ECC_HAM_writeset(){
	//No special setting for Wright in HAM
	Uint32 size0 = 0xff;
	Uint32 size1 = 0xff;

	GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, ((size1 << 22) | (size0 << 12)));
}

static void DEVICE_NAND_ECC_HAM_readset(){
	Uint32 size0 = 0xff;
	Uint32 size1 = 0xff;

	GPMC_Write(GPMC_ECC_SIZE_CONFIG, 0, ((size1 << 22) | (size0 << 12)));
//No special setting for Read in HAM
}
