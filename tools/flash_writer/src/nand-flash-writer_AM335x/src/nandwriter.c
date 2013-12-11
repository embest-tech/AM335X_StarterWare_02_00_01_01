/*
 * nandwriter.c
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
 * FILE        : nandwriter.c
 * PROJECT     : TI Booting and Flashing Utilities
 * AUTHOR      : Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		 Vinay Agrawal (vinayagw@ti.com)
 * DESC	      : CCS-based utility to flash the DM644x in preparation for
 * 		NAND booting
 *  -------------------------------------------------------------------------
 */

// C standard I/O library
#include "stdio.h"   // same
#include "debug.h"
#include "stdlib.h"
#include "string.h"

// General type include
#include "tistdtypes.h" //same


// main header file
#include "main.h"

// This module's header file
#include "AM335X_nandwriter.h"

// NAND driver include
#include "AM335X_nand.h" // same

// Misc. utility function include
#include "util.h"  // same


/************************************************************
 * Explicit External Declarations                            *
 ************************************************************/
extern VUint32 __FAR__ DDRStart;
/************************************************************
 * Local Macro Declarations                                  *
 ************************************************************/

/************************************************************
 * Local Typedef Declarations                                *
 ************************************************************/


/************************************************************
 * Local Function Declarations                               *
 ************************************************************/
static Uint32 LOCAL_writeHeaderAndData(AM335X_NAND_InfoHandle hNandInfo,
		AM335X_NANDWRITER_Boot *nandBoot, FILE *fPtr);


/************************************************************
 * Global Variable Definitions
 ************************************************************/

static Uint8* gNandRx;

/************************************************************
 * Global Function Definitions                                *
 ************************************************************/

Uint32 nandwriter(AM335X_NAND_InfoHandle hNandhandle)
{
	Uint32 numPagesAPP;
	Int32 last;
	AM335X_NANDWRITER_Boot gNandBoot;
	AM335X_NAND_InfoHandle hNandInfo;
	FILE *fPtr;
	Int32 appFileSize = 0;

	hNandInfo = hNandhandle;
	if (hNandInfo == NULL) {
		DEBUG_printString("\tERROR: NAND Initialization failed.\r\n");
		return E_FAIL;
	}

	if (eraseGlobal == 1) {
		printf("\nPerforming Global Nand Erase\n");

		if (AM335X_NAND_globalErase(hNandInfo) != E_PASS) {
			printf("FAIL to Global erase the flash device \n");
			return E_FAIL;
		}

		printf("... done");
		//Erase the Nand Device and exit from flash writer.
		return E_PASS;
	}

	if (operation == 1) {
		printf("Setting the ECC scheme");
		if (hNandInfo->nandhardware.fxnsetECC(hNandInfo->busWidth,app_ecc)
				!= E_PASS) {
			printf("... FAIL !!!\n");
			return E_FAIL;
		}

		printf(".... done\n");
		printf("Preparing to Flash image .... \n");
		if (strcmp(app_fileName, "none") != 0) {
			printf("Opening image ... ");

			// Open an File from the hard drive
			fPtr = fopen(app_fileName, "rb");
			if (fPtr == NULL) {
				DEBUG_printString("\tERROR: File ");
				DEBUG_printString(app_fileName);
				DEBUG_printString(" open failed.\r\n");
				return E_FAIL;
			}

			// Read file size
			fseek(fPtr, 0, SEEK_END);
			appFileSize = ftell(fPtr);
			if (appFileSize == 0) {
				DEBUG_printString("\tERROR: File read failed.. Closing program.\r\n");
				fclose(fPtr);
				return E_FAIL;
			}

			numPagesAPP = 0;
			while ((numPagesAPP * hNandInfo->dataBytesPerPage) <
					(appFileSize)) {
				numPagesAPP++;
			}

			// We want to allocate an even number of pages.
			fseek(fPtr, 0, SEEK_SET);
			printf("done. \n");
			gNandBoot.block = loadblock / (hNandInfo->dataBytesPerPage * hNandInfo->pagesPerBlock);

			// Setting block number from offset of application
			gNandBoot.page = 0;
			gNandBoot.numPage = numPagesAPP;
			last = appFileSize / (hNandInfo->dataBytesPerPage * hNandInfo->pagesPerBlock);
			last++;
			printf("Erasing Required Blocks [start = %d, count = %d]...", 
					gNandBoot.block, last);
			fflush(stdout);
			AM335X_NAND_eraseBlocks(hNandInfo, gNandBoot.block, last);
			printf("Done\n");
			printf("Flashing image ... \n");
			if (LOCAL_writeHeaderAndData(hNandInfo, &gNandBoot, fPtr) != E_PASS) {
				DEBUG_printString("\tERROR: Write Failed\n");
				return E_FAIL;
			}

			printf("Application is successfully flashed\n");
			fclose(fPtr);
		}
	}
	return E_PASS;
}

// Generic function to write a U-BOOT or Application header and the associated data

static Uint32 LOCAL_writeHeaderAndData(AM335X_NAND_InfoHandle hNandInfo,
		AM335X_NANDWRITER_Boot *nandBoot, FILE *fPtr)
{
	Uint32 blockNum; // starting block of data to write
	Uint32 count = 0;
	Uint32 countMask;
	Uint32 numBlks; //No. of blocks require to write data
	Uint8 *dataPtr;
	Uint32 pagesize = hNandInfo->dataBytesPerPage;

	gNandRx = (Uint8 *) AM335X_UTIL_callocMem(AM335X_NAND_MAX_PAGE_SIZE);
	fseek(fPtr, 0, SEEK_SET);
	dataPtr = (Uint8 *) AM335X_UTIL_callocMem(2 * pagesize);
	if (fread(dataPtr, 1, pagesize, fPtr) == 0) {
		printf("\tWARNING: File Size is 0\n");
	}

	numBlks = 0;
	// this can be optimized as numBlks = (nandBoot->numPage)%hNandInfo->pagesPerBlock + 1;
	while ((numBlks * hNandInfo->pagesPerBlock) <
			(nandBoot->numPage + 1)) {
		numBlks++;
	}

	DEBUG_printString("Number of blocks needed for header and data: ");
	DEBUG_printHexInt(numBlks);
	DEBUG_printString("\r\n");

	// Check whether writing U-BOOT or APP (based on destination block)
	blockNum = nandBoot->block;

NAND_WRITE_RETRY:
	if (blockNum > hNandInfo->numBlocks) {
		return E_FAIL;
	}
	DEBUG_printString("Attempting to start write in block number ");
	DEBUG_printHexInt(blockNum);
	DEBUG_printString(".\r\n");

	// Unprotect all needed blocks of the Flash
	if (AM335X_NAND_unProtectBlocks(hNandInfo, blockNum, numBlks)
			!= E_PASS) {
		blockNum++;
		DEBUG_printString("Unprotect failed.\r\n");
		goto NAND_WRITE_RETRY;
	}

	//pageNum = 0;
	// Erase the block where the header goes and the data starts
	if (AM335X_NAND_eraseBlocks(hNandInfo, blockNum, numBlks) != E_PASS) {
		blockNum++;
		DEBUG_printString("Erase failed\n");
		goto NAND_WRITE_RETRY;
	}

	// The following assumes power of 2 pagesPerBlock -  *should* always be valid
	countMask = (Uint32) hNandInfo->pagesPerBlock - 1;
	do {
		printf("Writing image data to Block ");
		printf("%d", blockNum);
		printf(" Page");
		printf("0x%x", count & countMask);
		printf("\r\n");

		// Write the U-BOOT or APP data on a per page basis
		if (AM335X_NAND_writePage(hNandInfo, blockNum,
					(count & countMask), dataPtr) != E_PASS) {
			blockNum++;
			DEBUG_printString("Write failed\n");
			goto NAND_WRITE_RETRY;
		}

		UTIL_waitLoop(200);

		// Verify the page just written
		if (AM335X_NAND_verifyPage(hNandInfo, blockNum,
					(count & countMask), dataPtr, gNandRx) != E_PASS) {
			DEBUG_printString("Verify failed. Attempting to clear page\n");
			AM335X_NAND_reset(hNandInfo);
			AM335X_NAND_eraseBlocks(hNandInfo, blockNum, numBlks);
			blockNum++;
			goto NAND_WRITE_RETRY;
		}

		count++;

		//Read the next page from the file
		memset(dataPtr, 0xff, pagesize);
		fread(dataPtr, pagesize, 1, fPtr);
		if (!(count & countMask)) {
			do {
				blockNum++;
			} while (AM335X_NAND_badBlockCheck(hNandInfo, blockNum) != E_PASS);
		}

	} while (count < (nandBoot->numPage));

WRITEFF:
	memset(dataPtr, 0xff, pagesize);
	while(count & countMask ) {
		printf("Writing dummy data to Block ");
		printf("%d", blockNum);
		printf(" Page");
		printf("0x%x", count & countMask);
		printf("\r\n");


		if (AM335X_NAND_writePage(hNandInfo, blockNum,
					(count & countMask), dataPtr) != E_PASS) {
			blockNum++;
			DEBUG_printString("Write failed\n");
			goto NAND_WRITEFF_RETRY;
		}

		UTIL_waitLoop(200);

		// Verify the page just written
		if (AM335X_NAND_verifyPage(hNandInfo, blockNum,
					(count & countMask), dataPtr, gNandRx) != E_PASS) {
			DEBUG_printString("Verify failed. Attempting to clear page\n");
			AM335X_NAND_reset(hNandInfo);
			AM335X_NAND_eraseBlocks(hNandInfo, blockNum, numBlks);
			blockNum++;
			goto NAND_WRITEFF_RETRY;
		}
		count++;

	}

	AM335X_NAND_protectBlocks(hNandInfo);
	return E_PASS;

NAND_WRITEFF_RETRY:
	if (AM335X_NAND_unProtectBlocks(hNandInfo, blockNum, numBlks)
			!= E_PASS) {
		blockNum++;
		DEBUG_printString("Unprotect failed.\r\n");
		goto NAND_WRITEFF_RETRY;
	}

	//pageNum = 0;
	// Erase the block where the header goes and the data starts
	if (AM335X_NAND_eraseBlocks(hNandInfo, blockNum, numBlks) != E_PASS) {
		blockNum++;
		DEBUG_printString("Erase failed\n");
		goto NAND_WRITEFF_RETRY;
	}
	goto WRITEFF;
	AM335X_NAND_protectBlocks(hNandInfo);
	return E_PASS;

}
