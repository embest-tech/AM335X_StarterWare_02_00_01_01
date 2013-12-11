
/*
 * nand.c
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
 * FILE      :  nand.c
 * PROJECT   :  TI Booting and Flashing Utilities
 * AUTHOR    :  Mansoor Ahamed (mansor.ahamed@ti.com)
 * 		Vinay Agrawal (vinayagw@ti.com)
 * DESC      : Generic NAND driver file for GPMC peripheral
 * --------------------------------------------------------------------------
 */

/************************************************************
 * Include Files                                             *
 ************************************************************/

// General type include
#include "tistdtypes.h"
#include "stdio.h"

// Util functions
#include "util.h"

// Debug functions for non-ROMed version
#include "debug.h"

// This module's header file 
#include "AM335X_nand.h"


// Device NAND specific stuff

/************************************************************
 * Explicit External Declarations                            *
 ************************************************************/




/************************************************************
 * Local Macro Declarations                                  *
 ************************************************************/


/************************************************************
 * Local Function Declarations                               *
 ************************************************************/

// Low-level NAND functions read, write, and command functions
static VUint8 *LOCAL_flashMakeAddr(Uint32 baseAddr, Uint32 offset); // done
static void LOCAL_flashWriteData(AM335X_NAND_InfoHandle hNandInfo, Uint32 offset, Uint32 data); // done
static Uint32 LOCAL_flashReadData(AM335X_NAND_InfoHandle hNandInfo); // done

// Address byte write functions
static void LOCAL_flashWriteColAddrBytes(AM335X_NAND_InfoHandle hNandInfo, Uint32 offset); // done
static void LOCAL_flashWriteRowAddrBytes(AM335X_NAND_InfoHandle hNandInfo, Uint32 page); // done

// Array data writing functions
static void LOCAL_flashWriteBytes(AM335X_NAND_InfoHandle hNandInfo, void *pSrc, Uint32 numBytes); // done

// Function to erase a block
static Uint32 LOCAL_eraseBlock(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Bool force); // done


// Array data reading functions
static void LOCAL_flashReadBytes(AM335X_NAND_InfoHandle hNandInfo, void *pDest, Uint32 numBytes); // done

// Wait for ready signal seen at NANDFSCR
static Uint32 LOCAL_flashWaitForRdy(AM335X_NAND_InfoHandle hNandInfo, Uint32 timeout); // done

// Wait for status result from device to read good
static Uint32 LOCAL_flashWaitForStatus(AM335X_NAND_InfoHandle hNandInfo, Uint32 timeout); // done

// page Pointer set function
static Uint32 LOCAL_setPagePtr(AM335X_NAND_InfoHandle hNandInfo, AM335X_NAND_RegionType regtionType, Uint32 opNum); // done

// ONFI CRC check for Read Parameter Page command
static Bool LOCAL_onfiParamPageCRCCheck(Uint8 *paramPageData);


/************************************************************
 * Local Variable Definitions                                *
 ************************************************************/


/************************************************************
 * Global Variable Definitions                               *
 ************************************************************/

/************************************************************
 * Global Function Definitions                               *
 ************************************************************/


// Routine to check a particular block to see if it is good or bad

Uint32 AM335X_NAND_badBlockCheck(AM335X_NAND_InfoHandle hNandInfo, Uint32 block)
{
	Uint8 spareBytes[256];

	if (!hNandInfo->hBbInfo->BBCheckEnable) {
		return E_PASS;
	} else if (hNandInfo->currBlock != block) {
		hNandInfo->currBlock = block;

		// Read and check spare bytes of first page of block (ONFI and normal)
		AM335X_NAND_readSpareBytesOfPage(hNandInfo, block, 0, spareBytes);
		
		if ((*hNandInfo->hBbInfo->fxnBBCheck)(hNandInfo, spareBytes) != E_PASS) {
			hNandInfo->isBlockGood = FALSE;
			return E_FAIL;
		}

		// Read and check spare bytes of second page of block (normal)
		AM335X_NAND_readSpareBytesOfPage(hNandInfo, block, 1, spareBytes);

		if ((*hNandInfo->hBbInfo->fxnBBCheck)(hNandInfo, spareBytes) != E_PASS) {
			hNandInfo->isBlockGood = FALSE;
			return E_FAIL;
		}

		if (hNandInfo->isONFI) {
			// Read and check spare bytes of last page of block (for ONFI)
			AM335X_NAND_readSpareBytesOfPage(hNandInfo, block,
					(hNandInfo->pagesPerBlock - 1), spareBytes);
			if ((*hNandInfo->hBbInfo->fxnBBCheck)(hNandInfo, spareBytes)
					!= E_PASS) {
				hNandInfo->isBlockGood = FALSE;
				return E_FAIL;
			}
		}
		
		hNandInfo->isBlockGood = TRUE;
		return E_PASS;
	} else if (hNandInfo->isBlockGood == FALSE) {
		return E_FAIL;
	} else {
		return E_PASS;
	}
}


// Routine to reset the NAND device

Uint32 AM335X_NAND_reset(AM335X_NAND_InfoHandle hNandInfo)
{

	// Send reset command to NAND
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_RESET);

	return LOCAL_flashWaitForRdy(hNandInfo, 512);
}


// Routine to read a page from NAND

Uint32 NAND_readPage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Uint32 page, Uint8 *dest) {
	Uint32 i, currPagePtr;
	// This is enough to support 8 Kbyte page devices
	 Uint8 spareBytes[256];

	 (*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);

	 // Get spare bytes of page (includes all stored ECC data)
	 AM335X_NAND_readSpareBytesOfPage(hNandInfo, block, page, spareBytes);

	 // Write read command
	 LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_READ_PAGE);

	 // Jump to first data region
	 currPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_DATA, 0);

	 LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
	 LOCAL_flashWriteRowAddrBytes(hNandInfo, (block * hNandInfo->pagesPerBlock) + page);

	 // Additional confirm command for large page devices
	 if (hNandInfo->isLargePage) {
		 LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				 AM335X_NAND_READ_30H);
	 }

	 // Wait for data to be available
	 if (LOCAL_flashWaitForRdy(hNandInfo, hNandInfo->nandhardware.timeout)
			 != E_PASS)
		 return E_FAIL;

	 // Clear the ECC hardware before starting
	 (*hNandInfo->hEccInfo->fxnReadset)();
	 (*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);

	 // Read data bytes with ECC enabled
	 for (i = 0; i < hNandInfo->numOpsPerPage; i++) {
		 (*hNandInfo->hEccInfo->fxnEnable)(hNandInfo);
		 LOCAL_flashReadBytes(hNandInfo, &dest[hNandInfo->dataBytesPerOp * i],
				 hNandInfo->dataBytesPerOp);

		 // Use ECC bytes to correct any errors
		 if ((*hNandInfo->hEccInfo->fxnCorrect)(hNandInfo, (block * hNandInfo->pagesPerBlock) + page,
					 spareBytes, &dest[hNandInfo->dataBytesPerOp * i], i) != E_PASS) {
			 return E_FAIL;
		 }

		 (*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);

		 // Go to next data region of page
		 currPagePtr += hNandInfo->dataBytesPerOp;
		 LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				 AM335X_NAND_RANDOM_READ_PAGE);
		 LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
		 LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				 AM335X_NAND_RANDOM_READ_E0H);
	 }

	 // Return status check result
	 return LOCAL_flashWaitForStatus(hNandInfo, hNandInfo->nandhardware.timeout);
}

// Function to just read the sparebytes region of a page

Uint32 AM335X_NAND_readSpareBytesOfPage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Uint32 page, Uint8 *dest)
{
	Uint32 i, currPagePtr, nextPagePtr;

	// The large page device MUST support Random Read Command (0x05)
	if (hNandInfo->isLargePage) {
		// Write read command
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_READ_PAGE);

		// Jump to first spare bytes region
		currPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_SPARE, 0);
		LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
		LOCAL_flashWriteRowAddrBytes(hNandInfo, (block * hNandInfo->pagesPerBlock) + page);

		// Additional confirm command for large page devices
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_READ_30H);


		// Wait for data to be available
		if (LOCAL_flashWaitForRdy(hNandInfo, hNandInfo->nandhardware.timeout) != E_PASS)
			return E_FAIL;

		// Collect spare bytes regions from the page into end of pageBuffer
		for (i = 0; i < hNandInfo->numOpsPerPage; i++) {
			// Read spare bytes
			LOCAL_flashReadBytes(hNandInfo, &dest[hNandInfo->spareBytesPerOp * i],
					hNandInfo->spareBytesPerOp);
			currPagePtr += hNandInfo->spareBytesPerOp;
			nextPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_SPARE, i + 1);

			if ((i != (hNandInfo->numOpsPerPage - 1)) && (currPagePtr != nextPagePtr)) {
				// Adjust page pointer
				currPagePtr = nextPagePtr;

				// Jump back to start of page (random read command)
				LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
						AM335X_NAND_RANDOM_READ_PAGE);
				LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
				LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
						AM335X_NAND_RANDOM_READ_E0H);
			}
		}
	} else {
		// Go to start of spare bytes region
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_EXTRA_PAGE);
		LOCAL_flashWriteColAddrBytes(hNandInfo, 0x0);
		LOCAL_flashWriteRowAddrBytes(hNandInfo, (block * hNandInfo->pagesPerBlock) + page);

		// Wait for data to be available
		if (LOCAL_flashWaitForRdy(hNandInfo, hNandInfo->nandhardware.timeout) != E_PASS)
			return E_FAIL;

		// Read spare bytes (includes ECC data)
		LOCAL_flashReadBytes(hNandInfo, dest, hNandInfo->spareBytesPerOp);

	}


	// Return status check result
	return LOCAL_flashWaitForStatus(hNandInfo, hNandInfo->nandhardware.timeout);
}

Uint32 AM335X_NAND_readBCHSpare(AM335X_NAND_InfoHandle hNandInfo, Uint32 pageLoc,
		Uint8 *dest, Uint32 opNum, Uint32 count)
{
	Uint32 currPagePtr;

	// The large page device MUST support Random Read Command (0x05)
	if (hNandInfo->isLargePage) {
		// Write read command
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				AM335X_NAND_READ_PAGE);

		// Jump to first spare bytes region
		currPagePtr = hNandInfo->dataBytesPerPage + hNandInfo->hEccInfo->offset +
			(hNandInfo->hEccInfo->storedECCByteCnt * opNum);

		LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
		LOCAL_flashWriteRowAddrBytes(hNandInfo, pageLoc);

		// Additional confirm command for large page devices
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_READ_30H);

		// Wait for data to be available
		if (LOCAL_flashWaitForRdy(hNandInfo, hNandInfo->nandhardware.timeout) != E_PASS)
			return E_FAIL;

		// Collect spare bytes regions from the page into end of pageBuffer
		// FIXME: Need to confirm the number of bytes to read.
		LOCAL_flashReadBytes(hNandInfo, dest, hNandInfo->hEccInfo->storedECCByteCnt);
	}
	return LOCAL_flashWaitForStatus(hNandInfo, hNandInfo->nandhardware.timeout);
}




// Defining this macro for the build will cause write (flash) ability to be removed
// This can be used for using this driver as read-only for ROM code
#ifndef USE_IN_ROM

// Check to see if device is write protected

Bool AM335X_NAND_isWriteProtected(AM335X_NAND_InfoHandle hNandInfo)
{
	Uint32 status;

	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_STATUS);
	status = LOCAL_flashReadData(hNandInfo);
	return (status & AM335X_NAND_STATUS_PROTECTED) ? FALSE : TRUE;
}

// Function to mark a bad block

Uint32 NAND_badBlockMark(AM335X_NAND_InfoHandle hNandInfo, Uint32 block)
{
	Uint8 spareBytes[256];

	// Check if marking enabled
	if (!hNandInfo->hBbInfo->BBMarkEnable) {
		return E_PASS;
	}

	// Check to see if this block is already known as bad
	if ((hNandInfo->currBlock == block) && (hNandInfo->isBlockGood == FALSE)) {
		return E_PASS;
	}

	// Indicate that this block is bad
	hNandInfo->currBlock = block;
	hNandInfo->isBlockGood = FALSE;

	// Mark the spare bytes according to device specific function
	(*hNandInfo->hBbInfo->fxnBBMark)(hNandInfo, spareBytes);

	// Erase the block so that we can mark the pages
	if (LOCAL_eraseBlock(hNandInfo, block, TRUE) != E_PASS) {
		return E_FAIL;
	}

	// Write the marked spare bytes to the first page (ONFI and normal)
	if (AM335X_NAND_writeOnlySpareBytesOfPage(hNandInfo, block, 0, spareBytes)
			!= E_PASS) {
		// Marking the first page didn't succeed, try a second
		if (hNandInfo->isONFI) {
			// Write the marked spare bytes to the last page (ONFI)
			return AM335X_NAND_writeOnlySpareBytesOfPage(hNandInfo, block,
					(hNandInfo->pagesPerBlock - 1), spareBytes);
		} else {
			// Write the marked spare bytes to the second page (normal)
			return AM335X_NAND_writeOnlySpareBytesOfPage(hNandInfo, block,
					1, spareBytes);
		}
	}
	return E_PASS;
}


// Function to write only the spare bytes region (page must be
// erased and not written to prior to calling this)

Uint32 AM335X_NAND_writeOnlySpareBytesOfPage(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8* spareBytes)
{
	Uint32 i, currPagePtr, nextPagePtr;

	// For small page devices, set pointer
	if (!hNandInfo->isLargePage)
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				AM335X_NAND_EXTRA_PAGE);

	// Write program command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_PGRM_START);

	// Write address bytes
	currPagePtr = LOCAL_setPagePtr(hNandInfo,
			NAND_REGION_SPARE, 0);

	if (!hNandInfo->isLargePage)
		LOCAL_flashWriteColAddrBytes(hNandInfo, 0);
	else
		LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);

	LOCAL_flashWriteRowAddrBytes(hNandInfo, (block * hNandInfo->pagesPerBlock) + page);
	
	// Write spare bytes sections of page
	for (i = 0; i < hNandInfo->numOpsPerPage; i++) {
		LOCAL_flashWriteBytes(hNandInfo, &spareBytes[hNandInfo->spareBytesPerOp * i],
				hNandInfo->spareBytesPerOp);

		currPagePtr += hNandInfo->spareBytesPerOp;

		if (i != (hNandInfo->numOpsPerPage - 1)) {
			nextPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_SPARE, i + 1);
			if (currPagePtr != nextPagePtr) {
				// Adjust page pointer
				currPagePtr = nextPagePtr;

				// Jump to next data section of page (random program command)
				LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
						AM335X_NAND_RANDOM_PGRM);
				LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
			}
		}
	}

	// Write program end command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_PGRM_END);

	// Wait for the device to be ready
	if (LOCAL_flashWaitForRdy(hNandInfo, hNandInfo->nandhardware.timeout) != E_PASS)
		return E_FAIL;

	// Return status check result
	return LOCAL_flashWaitForStatus(hNandInfo, hNandInfo->nandhardware.timeout);
}



// Generic routine to write a page of data to NAND

Uint32 AM335X_NAND_writePage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Uint32 page, Uint8 *src)
{
	Uint32 i, currPagePtr, nextPagePtr;
	Uint8 calcECC[16];
	// This is enough to support 8 Kbyte page devices
	Uint8 spareBytes[256];

	// Fill in the spare bytes region with 0xFF
	for (i = 0; i < hNandInfo->spareBytesPerPage; i++) {
		spareBytes[i] = 0xFF;
	}

	// For small page devices, set pointer
	if (!hNandInfo->isLargePage)
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				AM335X_NAND_LO_PAGE);

	// Write program command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_PGRM_START);

	// Write address bytes
	currPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_DATA, 0);
	LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
	LOCAL_flashWriteRowAddrBytes(hNandInfo,
			(block * hNandInfo->pagesPerBlock) + page);
	
	// Clear the ECC hardware before starting
	(*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);
	(*hNandInfo->hEccInfo->fxnWriteset)();
	(*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);

	// Write data sections of page, getting ECC data
	for (i = 0; i < hNandInfo->numOpsPerPage; i++) {
		(*hNandInfo->hEccInfo->fxnEnable)(hNandInfo);
		LOCAL_flashWriteBytes(hNandInfo, &src[hNandInfo->dataBytesPerOp * i],
			       hNandInfo->dataBytesPerOp);

		/* LOCAL_flashWriteBytes(hNandInfo, &src[hNandInfo->dataBytesPerOp * i], 40); */
		(*hNandInfo->hEccInfo->fxnDisable)(hNandInfo);
		(*hNandInfo->hEccInfo->fxnCalculate)(hNandInfo,
				&src[hNandInfo->dataBytesPerOp * i], calcECC);
		(*hNandInfo->hEccInfo->fxnStore)(hNandInfo, spareBytes, i, calcECC);
		currPagePtr += hNandInfo->dataBytesPerOp;
	}

	currPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_SPARE, 0);

	// Write spare bytes sections of page
	for (i = 0; i < hNandInfo->numOpsPerPage; i++) {
		LOCAL_flashWriteBytes(hNandInfo, &spareBytes[hNandInfo->spareBytesPerOp * i],
				hNandInfo->spareBytesPerOp);
		currPagePtr += hNandInfo->spareBytesPerOp;

		if (i != (hNandInfo->numOpsPerPage - 1)) {
			nextPagePtr = LOCAL_setPagePtr(hNandInfo, NAND_REGION_SPARE, i + 1);
			if (currPagePtr != nextPagePtr) {
				// Adjust page pointer
				currPagePtr = nextPagePtr;

				// Jump to next spare section of page (random program command)
				LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
						AM335X_NAND_RANDOM_PGRM);
				LOCAL_flashWriteColAddrBytes(hNandInfo, currPagePtr);
			}
		}
	}

	// Write program end command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_PGRM_END);

	// Wait for the device to be ready
	if (LOCAL_flashWaitForRdy(hNandInfo,
				hNandInfo->nandhardware.timeout) != E_PASS)
		return E_FAIL;

	// Return status check result
	return LOCAL_flashWaitForStatus(hNandInfo, hNandInfo->nandhardware.timeout);
}


// Verify data written by reading and comparing byte for byte

Uint32 AM335X_NAND_verifyPage(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8* src, Uint8* dest)
{
	Uint32 i, errCnt;

	if (NAND_readPage(hNandInfo, block, page, dest) != E_PASS)
		return E_FAIL;

	errCnt = 0;
	for (i = 0; i < (hNandInfo->dataBytesPerPage); i++) {
		// Check for data read errors
		if (src[i] != dest[i]) {
			errCnt++;
			DEBUG_printString("Data verification failed! Byte # ");
			DEBUG_printHexInt(i);
			DEBUG_printString(" Expected Data: ");
			DEBUG_printHexInt(src[i]);
			DEBUG_printString(", Received Byte: ");
			DEBUG_printHexInt(dest[i]);
			DEBUG_printString("\r\n");
		}
	}

	if (errCnt != 0) {
		return E_FAIL;
	} else {
		return E_PASS;
	}

}

// Verify erase succeeded by reading and comparing byte for byte

Uint32 AM335X_NAND_verifyBlockErased(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Uint8* dest)
{
	Uint32 i, j;

	for (j = 0; j < hNandInfo->pagesPerBlock; j++) {
		if (NAND_readPage(hNandInfo, block, j, dest) != E_PASS)
			return E_FAIL;

		for (i = 0; i < (hNandInfo->dataBytesPerPage >> 2); i++) {
			// Check for data read errors
			if (((Uint32 *) dest)[i] += 0xFFFFFFFF) {
				return E_FAIL;
			}
		}
	}
	return E_PASS;
}

// Global Erase NAND Flash

Uint32 AM335X_NAND_globalErase(AM335X_NAND_InfoHandle hNandInfo)
{
	// We don't erase block 0, and possibly some ending blocks reserved for BBT
	return AM335X_NAND_eraseBlocks(hNandInfo, 0,
			(hNandInfo->numBlocks - 1 - AM335X_NAND_NUM_BLOCKS_RESERVED_FOR_BBT));
}

// GLobal Erase NAND flash with Bad Block checking and marking

Uint32 NAND_globalErase_with_bb_check(AM335X_NAND_InfoHandle hNandInfo)
{
	// We don't erase block 0, and possibly some ending blocks reserved for BBT
	return AM335X_NAND_eraseBlocks_with_bb_check(hNandInfo, 1,
			(hNandInfo->numBlocks - 1));
}

// NAND Flash erase block function

Uint32 AM335X_NAND_eraseBlocks(AM335X_NAND_InfoHandle hNandInfo, Uint32 startBlkNum, Uint32 blkCnt)
{
	Uint32 i;
	Uint32 endBlkNum = startBlkNum + blkCnt - 1;

	// Do bounds checking
	if (endBlkNum >= hNandInfo->numBlocks)
		return E_FAIL;

	// Output info about what we are doing
	for (i = startBlkNum; i <= endBlkNum; i++) {
		if (LOCAL_eraseBlock(hNandInfo, i, FALSE) != E_PASS) {
			printf("FAIL!! to erase block number  0x%x\n", i);
		}
		//return E_FAIL;
	}
	return E_PASS;
}

Uint32 AM335X_NAND_eraseBlocks_with_bb_check(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 startBlkNum, Uint32 blkCnt)
{
	Uint32 i;
	Uint32 endBlkNum = startBlkNum + blkCnt - 1;

	// Do bounds checking
	if (endBlkNum >= hNandInfo->numBlocks)
		return E_FAIL;

	// Output info about what we are doing
	for (i = startBlkNum; i <= endBlkNum; i++) {
		if (LOCAL_eraseBlock(hNandInfo, i, FALSE) != E_PASS) {
#if (0)
			DEBUG_printString(" Bad block at block ");
			DEBUG_printHexInt(i);
			DEBUG_printString(". Erasing is skipped\n");
			NAND_badBlockMark(hNandInfo, i);
#endif
		}
	}
	return E_PASS;
}

// NAND Flash unprotect command

Uint32 AM335X_NAND_unProtectBlocks(AM335X_NAND_InfoHandle hNandInfo, Uint32 startBlkNum, Uint32 blkCnt)
{
	Uint32 endBlkNum = startBlkNum + blkCnt - 1;

	// Do bounds checking
	if (endBlkNum >= hNandInfo->numBlocks)
		return E_FAIL;

	// Output info about what we are doing
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_UNLOCK_START);
	LOCAL_flashWriteRowAddrBytes(hNandInfo,
			hNandInfo->pagesPerBlock * startBlkNum);
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_UNLOCK_END);
	LOCAL_flashWriteRowAddrBytes(hNandInfo,
			hNandInfo->pagesPerBlock * endBlkNum);
	return E_PASS;
}

// NAND Flash protect command

void AM335X_NAND_protectBlocks(AM335X_NAND_InfoHandle hNandInfo)
{
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_LOCK);
}

#endif // END of !defined(USE_IN_ROM) section


/************************************************************
 * Local Function Definitions                                *
 ************************************************************/

// Generic Low-level NAND access functions

static VUint8 *LOCAL_flashMakeAddr(Uint32 baseAddr, Uint32 offset)
{
	return ((VUint8 *) (baseAddr + offset));
}

static void LOCAL_flashWriteData(AM335X_NAND_InfoHandle hNandInfo, Uint32 offset, Uint32 data)
{
	//volatile NAND_Ptr addr;
	//NAND_Data dataword;
	//dataword.l = data;
	volatile char *cPtr;

	cPtr = LOCAL_flashMakeAddr(hNandInfo->flashBase, offset);
	*cPtr = (char)data;
}

static Uint32 LOCAL_flashReadData(AM335X_NAND_InfoHandle hNandInfo)
{
	//volatile NAND_Ptr addr;
	//NAND_Data cmdword;


	//cmdword.l = 0x0;
	volatile char *cPtr;
	cPtr = LOCAL_flashMakeAddr(hNandInfo->flashBase, hNandInfo->nanddataoffset);
	/*
	switch (hNandInfo->busWidth) {
		case BUS_8BIT:
			cmdword.c = *addr.cp;
			break;
		case BUS_16BIT:
			cmdword.w = *addr.wp;
			break;
	}
	*/
		return (Uint32)*cPtr;
}

static void LOCAL_flashWriteRowAddrBytes(AM335X_NAND_InfoHandle hNandInfo, Uint32 page)
{
	Uint32 i;

	for (i = 0; i < hNandInfo->numRowAddrBytes; i++) {
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandaleoffset,
				((page >> (8 * i)) & 0xff));
	}
}

static void LOCAL_flashWriteColAddrBytes(AM335X_NAND_InfoHandle hNandInfo, Uint32 offset)
{
	Uint32 i;

	// Adjust column address for 16-bit buswidth since we address words instead of bytes
	if (hNandInfo->busWidth == (Uint8) AM335X_DEVICE_BUSWIDTH_16BIT) {
		offset = offset / 2;
	}

	for (i = 0; i < hNandInfo->numColAddrBytes; i++) {
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandaleoffset,
				((offset >> (8 * i)) & 0xff));
	}
}

#ifndef USE_IN_ROM

static void LOCAL_flashWriteBytes(AM335X_NAND_InfoHandle hNandInfo, void* pSrc, Uint32 numBytes) {
	//volatile NAND_Ptr destAddr, srcAddr;
	Uint32 i;
	volatile char *cPtr, *srcPtr = pSrc;

	cPtr = LOCAL_flashMakeAddr(hNandInfo->flashBase, hNandInfo->nanddataoffset);
	for (i = 0; i < (numBytes); i++)
		*cPtr = *srcPtr++;

	/*
	srcAddr.cp = (VUint8*) pSrc;
	destAddr.cp = LOCAL_flashMakeAddr(hNandInfo->flashBase, hNandInfo->nanddataoffset);
	switch (hNandInfo->busWidth) {
		case BUS_8BIT:
			for (i = 0; i < (numBytes); i++)
				*destAddr.cp = *srcAddr.cp++;
			break;
		case BUS_16BIT:
			for (i = 0; i < (numBytes >> 1); i++)
				*destAddr.wp = *srcAddr.wp++;
			break;
	}
	*/
}

static Uint32 LOCAL_eraseBlock(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Bool force)
{
	// Start erase command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_BERASEC1);

	// Write the row addr bytes only
	LOCAL_flashWriteRowAddrBytes(hNandInfo,
			hNandInfo->pagesPerBlock * block);

	// Confirm erase command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_BERASEC2);

	// Wait for the device to be ready
	if (LOCAL_flashWaitForRdy(hNandInfo,
				hNandInfo->nandhardware.timeout) != E_PASS)
		return E_FAIL;

	// Verify the op succeeded by reading status from flash
	return LOCAL_flashWaitForStatus(hNandInfo,
			hNandInfo->nandhardware.timeout);

}
#endif

static void LOCAL_flashReadBytes(AM335X_NAND_InfoHandle hNandInfo,
		void* pDest, Uint32 numBytes)
{
	//volatile NAND_Ptr destAddr, srcAddr;
	Uint32 i;
	volatile char *cPtr, *destPtr = pDest;

	cPtr = LOCAL_flashMakeAddr(hNandInfo->flashBase, hNandInfo->nanddataoffset);
	for (i = 0; i < (numBytes); i++)
		*destPtr++ = *cPtr;

	/*
	destAddr.cp = (VUint8*) pDest;
	srcAddr.cp = LOCAL_flashMakeAddr(hNandInfo->flashBase, hNandInfo->nanddataoffset);
	switch (hNandInfo->busWidth) {
		case BUS_8BIT:
			for (i = 0; i < (numBytes); i++)
				*destAddr.cp++ = *srcAddr.cp;
			break;
		case BUS_16BIT:
			for (i = 0; i < (numBytes >> 1); i++)
				*destAddr.wp++ = *srcAddr.wp;
			 break;
	}
	*/
}

// Poll bit of NANDFSR to indicate ready

static Uint32 LOCAL_flashWaitForRdy(AM335X_NAND_InfoHandle hNandInfo, Uint32 timeout)
{
	VUint32 cnt;
	Uint32 status;
	Bool status_reached_zero = FALSE;

	cnt = timeout;

	// Wait for the status to show busy, and then after that
	// point we start checking for it to be high.  If we
	// don't do this we might see the status as ready before
	// it has even transitioned to show itself as busy.
	do {
		status = (*(hNandInfo->hAsyncMemInfo->hDeviceInfo->fxnNandIsReadyPin))
			(hNandInfo->hAsyncMemInfo);
		if (status == 0)
			status_reached_zero = TRUE;
	} while (((cnt--) > 0) && (!status || (status_reached_zero == FALSE)));

	return E_PASS;
}


// Wait for the status to be ready in NAND register
//      There were some problems reported in DM320 with Ready/Busy pin
//      not working with all NANDs. So this check has also been added.

static Uint32 LOCAL_flashWaitForStatus(AM335X_NAND_InfoHandle hNandInfo, Uint32 timeout)
{
	VUint32 cnt;
	Uint32 status;
	cnt = timeout;

	do {
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset, AM335X_NAND_STATUS);
		status = LOCAL_flashReadData(hNandInfo);
		cnt--;
	} while ((cnt > 0) && !(status & AM335X_NAND_STATUS_READY));

	if ((cnt == 0) || (status & AM335X_NAND_STATUS_ERROR)) {
		return E_FAIL;
	}

	return E_PASS;
}


// Function to set page pointepr to appropriate location // good function

static Uint32 LOCAL_setPagePtr(AM335X_NAND_InfoHandle hNandInfo,
		AM335X_NAND_RegionType regionType, Uint32 opNum)
{
	Uint32 currPtr = 0;
	AM335X_NAND_RegionHandle hRegion;
	hRegion = (regionType == NAND_REGION_DATA) ?
		&(hNandInfo->hPageLayout->dataRegion) : &(hNandInfo->hPageLayout->spareRegion);

	currPtr = (hRegion->offsetType == NAND_OFFSETS_RELTODATA) ? 0 : hNandInfo->dataBytesPerPage;
	currPtr += hRegion->offsets[opNum];
	return currPtr;
}


// Get details of the NAND flash used from the id and the table of NAND devices

Uint32 AM335X_flashGetDetails(AM335X_NAND_InfoHandle hNandInfo)
{

	Uint8 devID[4];
	Uint32 i, j;
	Uint32 blocksize = 0;

	// Check if ONFI compatible device
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_RDID);
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandaleoffset,
			AM335X_NAND_ONFIRDIDADD);
	if (LOCAL_flashWaitForRdy(hNandInfo,
				hNandInfo->nandhardware.timeout) != E_PASS)
		return E_FAIL;

	// Read ID bytes (to check for ONFI signature)
	for (i = 0; i < 4; i++)
		devID[i] = LOCAL_flashReadData(hNandInfo) & 0xFF;

	// Set ONFI flag as appropriate
	if (*((Uint32 *) devID) == AM335X_NANDONFI_STRING)
		hNandInfo->isONFI = TRUE;
	else
		hNandInfo->isONFI = FALSE;

	// Send reset command to NAND
	if (AM335X_NAND_reset(hNandInfo) != E_PASS)
		return E_FAIL;

	// Issue device read ID command
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
			AM335X_NAND_RDID);
	LOCAL_flashWriteData(hNandInfo, hNandInfo->nandaleoffset,
			AM335X_NAND_RDIDADD);

	if (LOCAL_flashWaitForRdy(hNandInfo,
				hNandInfo->nandhardware.timeout) != E_PASS)
		return E_FAIL;

	// Read ID bytes (to get true device ID data)
	for (i = 0; i < 4; i++)
		devID[i] = LOCAL_flashReadData(hNandInfo) & 0xFF;

	// Use JEDEC manufacturer ID from Parameter Page
	hNandInfo->manfID = devID[0];

	// Use Device ID from the standard READID command
	hNandInfo->devID             = (Uint8) devID[1];
	hNandInfo->dataBytesPerPage  = 1024 << (devID[3] &0x3); // Last two bits give Page size
	hNandInfo->spareBytesPerPage = 32 << ((devID[3] &0xF) >> 2); // Bit 2,3 bit give spare area
	blocksize                    = 64 << ((devID[3] >> 4) & 0x3); // Bit 4,5 give Block size;
	hNandInfo->pagesPerBlock     = (blocksize * 1024) / hNandInfo->dataBytesPerPage;
	while (hNandInfo->hChipInfo[i].devID != 00) {
		if (hNandInfo->hChipInfo[i].devID == hNandInfo->devID) {
			hNandInfo->numBlocks =
				(hNandInfo->hChipInfo[i].capacity * 1024) / (blocksize * 8);
		}
		i++;
	}

	if ((devID[3] >> 6) & 0x1) { // Device says it is 16-bit
		hNandInfo->busWidth = (Uint8) AM335X_DEVICE_BUSWIDTH_16BIT;
		hNandInfo->nandhardware.fxnsetNand_16(hNandInfo->flashBase);
	} else {   // Device says it is 8-bit
		hNandInfo->busWidth = (Uint8) AM335X_DEVICE_BUSWIDTH_8BIT;
	}
	
	if (hNandInfo->isONFI) {
		Uint8 paramPageData[256];

		// Issue read param page command
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandcleoffset,
				AM335X_NANDONFI_RDPARAMPAGE);

		// Issue lo-page address
		LOCAL_flashWriteData(hNandInfo, hNandInfo->nandaleoffset,
				AM335X_NAND_READ_PAGE);

		// Wait for data to be available
		if (LOCAL_flashWaitForRdy(hNandInfo,
					hNandInfo->nandhardware.timeout) != E_PASS)
			return E_FAIL;

		// Read 256 bytes of param page data
		j = 0;
		do {
			for (i = 0; i < 256; i++)
				paramPageData[i] = LOCAL_flashReadData(hNandInfo) & 0xFF;
			j++;
		} while ((!LOCAL_onfiParamPageCRCCheck(paramPageData)) && (j < 3));

		// We never got a good param page, so look in the NAND table
		if (j == 3) goto SEARCH_TABLE;
		printf("\n\nThe NAND Flash is ONFI compatible \n ");
		hNandInfo->numColAddrBytes = (Uint8) ((paramPageData[101] >> 4) & 0xF);
		hNandInfo->numRowAddrBytes = (Uint8) (paramPageData[101] & 0xF);
	} else {
SEARCH_TABLE:
		i = 0;

		// Setup row and column address byte-lengths
		hNandInfo->numColAddrBytes = (hNandInfo->dataBytesPerPage > 512) ? 2 : 1;

		j = 0;
		while (((hNandInfo->numBlocks * hNandInfo->pagesPerBlock) >> j) > 1)
			j++;
		hNandInfo->numRowAddrBytes = j >> 3;
		if (j > (hNandInfo->numRowAddrBytes << 3))
			hNandInfo->numRowAddrBytes++;
	}

	// Assign the number of operations per page value
	hNandInfo->numOpsPerPage = 0;
	while ((hNandInfo->numOpsPerPage * hNandInfo->nandhardware.maxbyteperop)
			< hNandInfo->dataBytesPerPage)
		hNandInfo->numOpsPerPage++;

	// Assign the bytes per operation value
	if (hNandInfo->dataBytesPerPage <
			hNandInfo->hPageLayout->dataRegion.bytesPerOp) {
		hNandInfo->dataBytesPerOp = hNandInfo->dataBytesPerPage;
	} else {
		hNandInfo->dataBytesPerOp = hNandInfo->hPageLayout->dataRegion.bytesPerOp;
	}

	// Assign the spare bytes per operation value
	if (hNandInfo->spareBytesPerPage <
			hNandInfo->hPageLayout->spareRegion.bytesPerOp) {
		hNandInfo->spareBytesPerOp = hNandInfo->spareBytesPerPage;
	} else {
		hNandInfo->spareBytesPerOp = hNandInfo->hPageLayout->spareRegion.bytesPerOp;
	}

	// Check to make sure there are enough spare bytes to satisfy our needs
	if ((hNandInfo->numOpsPerPage * hNandInfo->spareBytesPerOp) >
			hNandInfo->spareBytesPerPage)
		return E_FAIL;

	// Check and make sure we have enough spare bytes per op
	if (hNandInfo->spareBytesPerOp < hNandInfo->nandhardware.minsparebyteperop)
		return E_FAIL;

	// Assign the large page flag
	hNandInfo->isLargePage = (hNandInfo->dataBytesPerPage > 512) ? TRUE : FALSE;
	printf("\n\n----------------------\n");
	printf("  NAND FLASH DETAILS\n");
	printf("----------------------\n");
	 printf(" Device ID : 0x%x\n     \
			Manufacture ID : 0x%x\n  \
			Page Size : %d Bytes\n   \
			Spare Size : %d Bytes\n  \
			Pages_Per_Block : %d\n   \
			Number_of_Blocks : %d\n  \
			Device_width : %d Byte\n \
			DeviceSize : %d MB\n\n ",
			hNandInfo->devID,
			hNandInfo->manfID,
			hNandInfo->dataBytesPerPage,
			hNandInfo->spareBytesPerPage,
			hNandInfo->pagesPerBlock,
			hNandInfo->numBlocks,
			hNandInfo->busWidth + 1,
			(hNandInfo->pagesPerBlock * hNandInfo->numBlocks *
			 hNandInfo->dataBytesPerPage) / (1024 * 1024));

	 //Expecting oxDA 0x2c 2048 64 64 2048 1 256  //REVISIT data is as per spcification MT29F2G08AB
	 return E_PASS;
}

static Bool LOCAL_onfiParamPageCRCCheck(Uint8 *paramPageData)
{
	// Bit by bit algorithm without augmented zero bytes
	const Uint16 polynom = 0x8005; // Polynomial
	const Uint16 crcmask = 0xFFFF, crchighbit = 0x8000;
	Uint16 crc = 0x4F4E; // Initialize the crc shift register with 0x4F4E
	Uint16 i, j, bit;

	for (i = 0; i < 254; i++) {
		for (j = 0x80; j; j >>= 1) {
			bit = crc & crchighbit;
			crc <<= 1;
			if (paramPageData[i] & j)
				bit ^= crchighbit;
			if (bit) crc ^= polynom;
		}
		crc &= crcmask;
	}
	return (crc == *((Uint16 *) & paramPageData[254]));
}

/***********************************************************
 * End file                                                 *
 ***********************************************************/
