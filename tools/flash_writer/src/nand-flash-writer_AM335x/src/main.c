/*
 * main.c
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
 * FILE      :  main.c
 * PROJECT   :  TI Booting and Flashing Utilities
 * AUTHOR    :  Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		Vinay Agrawal (vinayagw@ti.com)
 * DESC      :  Generic NAND driver file for GPMC peripheral
 * --------------------------------------------------------------------------
 */

/************************************************************
 * Include Files                                             *
 ************************************************************/

// C standard I/O library
#include "stdio.h"
#include "stdlib.h"
#include "tistdtypes.h"
// Different type CCS nand flash modules 
#include "AM335X_nandwriter.h"
// This moduel's headfile 
#include "main.h"
#include "debug.h"
#include "AM335X_device.h"
#include "board_identification.h"

/************************************************************
 * Explicit External Declarations                            *
 ************************************************************/

Int32 operation = 0,
      eraseGlobal = 0, 
      app_ecc = 0, // ECC scheme type for U-BOOT
      uboot_ecc = 0; // ECC scheme type for U-Boot
Int8  uboot_fileName[256] = "none";
Int8  app_fileName[256] = "none";
Int32 loadblock;

/************************************************************
 * Local Macro Declarations                                  *
 ************************************************************/
/************************************************************
 * Local Typedef Declarations                                *
 ************************************************************/

/************************************************************
 * Local Function Declarations                               *
 ************************************************************/
static int getPlatform(void);
static int getEccType(void);
/************************************************************
 * Global Variable Definitions
 ************************************************************/
static pin_muxing_t PAD_Conf_I2C0 [] = {
	{I2C0_SD0_OFF, MODE(0) | RXACTIVE | SLEW_CONTROL_SLOW },
	{I2C0_SCL_OFF, MODE(0) | RXACTIVE | SLEW_CONTROL_SLOW },
	{0xFFFFFFFF},
};

/************************************************************
 * Global Function Definitions                               *
 ************************************************************/
void main() {
	int status,board;

	board = getPlatform();
	printf("\n\nChoose your operation \n");
	printf("Enter 1 ---> To Flash an Image\n");
	printf("Enter 2 ---> To ERASE the whole NAND \n");
	printf("Enter 3 ---> To EXIT\n");
	scanf("%d", &operation); // get the operation
	fflush(stdin);

	switch (operation) {
		case 1:
			printf("Enter image file path \r\n");
			scanf("%s", &app_fileName);
			fflush(stdin);
			printf("Enter offset to be flashed (in hex): \n");
			scanf("%x", &loadblock); /*image flash location */
			fflush(stdin);
			app_ecc = getEccType();
			break;
		case 2:
			eraseGlobal = 1;
			break;
		default: printf("Closing CCS NAND Flash Utility \n");
			exit(1);
	}

	status = AM335X_Start(); //start the AM335X NAND writer

	if (status != E_PASS) {
		printf("\n\nNAND flashing failed!\r\n");
	} else {
		printf("\n\nNAND flashing successful!\r\n");
	}
}
// Use JTAG Id to indentify connected platform

static int getPlatform() {
	int board = -1,profile = -1;

	printf("Welcome to CCS Nand Flash Utility \n");
	PAD_ConfigMux(PAD_Conf_I2C0);
	board   = i2c_daughter_card_detection();
	/*
	if(board > 0)
		if(board == GP_BOARD)
			profile = 1 << profile_identification();
	else if( board == BASE_BOARD)
		return BASE_BOARD;
	else{
		printf("Closing CCS NAND Flash Utility \n");
		printf("NAND FLASH device is not supported.\n");
		printf("board not detected.\n");
		exit(-1);
	}

	switch(board){
			break;
		case GP_BOARD:	
			if( profile != PROFILE_2 ||
					profile != PROFILE_3)
				break;
			else{
				printf("Closing CCS NAND Flash Utility \n");
				printf("NAND FLASH device is not supported.\n");
				printf("board <%d>, Profile <%d>.\n",
						board,profile);
				exit(1);
			}
		case IA_BOARD:
			break;
		case IP_BOARD:
			break;
		default:
			break;
	}
	*/
	return board;
}

static Int32 getEccType(void) {
	Int32 eccType;
	printf("Choose the ECC scheme from given options :\n");
	printf("Enter 1 ---> BCH 8 bit \n");
	printf("Enter 2 ---> HAM  \n");
	printf("Enter 3 ---> T0 EXIT\n");
	DEBUG_printString("Please enter ECC scheme type :\r\n");
	scanf("%d", &eccType);
	return eccType;
}
