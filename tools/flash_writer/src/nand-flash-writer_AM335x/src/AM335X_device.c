/*
 * device.c
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
 * FILE        : device.c
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

// Utility functions
#include "util.h"
// This module's header file
#include "AM335X_device.h"

#include "AM335X_nand.h"
#include "AM335X_ecc.h"
#include "AM335X_nandwriter.h"

// Debug functions for non-ROMed version
#include "debug.h"




/************************************************************
 * Explicit External Declarations                            *
 ************************************************************/

// The device specific ECC info struct
extern AM335X_NAND_ECC_InfoObj AM335X_DEVICE_NAND_ECC_info;

// The device specific BB info struct
extern AM335X_NAND_BB_InfoObj AM335X_DEVICE_NAND_BB_info;

/************************************************************
 * Local Macro Declarations                                  *
 ************************************************************/


/************************************************************
 * Local Typedef Declarations                                *
 ************************************************************/

/************************************************************
* Local Function Declarations                               *
************************************************************/
static void DEVICE_ASYNC_MEM_Init(AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo);
static Uint8 DEVICE_ASYNC_MEM_IsNandReadyPin(AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo);

static Uint32 AM335X_DEVICE_setGPMC(GPMC_Config_t *cfg, Uint32 base_CS);
static void AM335X_DEVICE_setPad();
void GPMC_Write(Uint32 base, Uint32 offset, Uint32 val);
static void setNand_16(Uint32 base_cs);
/************************************************************
* Local Variable Definitions                                *
************************************************************/
AM335X_NAND_PAGE_LayoutObj AM335X_DEVICE_NAND_PAGE_layout = {
	// Data region definition
	{
		NAND_REGION_DATA,                             // Region Type
		NAND_OFFSETS_RELTODATA,                       // Offsets relative type
		AM335X_DEVICE_NAND_MAX_BYTES_PER_OP,          // bytesPerOp
		{ 0 * (512), 1 * (512), 2 * (512), 3 * (512), // dataOffsets
			4 * (512), 5 * (512), 6 * (512), 7 * (512),
			8 * (512), 9 * (512), 10 * (512), 11 * (512),
			12 * (512), 13 * (512), 14 * (512), 15 * (512)}
	},

	// Spare region definition
	{
		NAND_REGION_SPARE,                            // Region Type
		NAND_OFFSETS_RELTOSPARE,                      // Offsets relative type
		AM335X_DEVICE_NAND_MAX_SPAREBYTES_PER_OP,     // bytesPerOp
		{ 0 * 16, 1 * 16, 2 * 16, 3 * 16,             // spareOffsets
			4 * 16, 5 * 16, 6 * 16, 7 * 16,
			8 * 16, 9 * 16, 10 * 16, 11 * 16,
			12 * 16, 13 * 16, 14 * 16, 15 * 16}
	}
};

// Table of ROM supported NAND devices
AM335X_NAND_CHIP_InfoObj DEVICE_NAND_CHIP_infoTable[] = {// devID,  Capacity,  buswidth,  bytesPerPage
	{ 0xE6, 128, 8, 512 + 16},    // 4 MB
	{ 0x33, 128, 8, 512 + 16},    // 4 MB
	{ 0x73, 128, 8, 512 + 16},    // 8 MB
	{ 0x43, 128, 16, 512 + 16},   // 8 MB
	{ 0x53, 128, 16, 512 + 16},   // 8 MB x16
	{ 0x35, 256, 8, 512 + 16},    // 8 MB x16
	{ 0x75, 256, 8, 512 + 16},    // 8 MB
	{ 0x45, 256, 16, 512 + 16},   // 16 MB
	{ 0x55, 256, 16, 512 + 16},   // 16 MB
	{ 0x36, 512, 8, 512 + 16},    // 16 MB x16
	{ 0x76, 512, 8, 512 + 16},    // 16 MB x16
	{ 0x46, 512, 16, 512 + 16},   // 32 MB
	{ 0x56, 512, 16, 512 + 16},   // 32 MB
	{ 0xA2, 512, 8, 2048 + 64},   // 32 MB
	{ 0xF2, 512, 8, 2048 + 64},   // 64 MB
	{ 0xB2, 512, 16, 2048 + 64},  // 64 MB
	{ 0xC2, 512, 16, 2048 + 64},  // 64 MB x16
	{ 0x39, 1024, 8, 512 + 16},   // 64 MB x16
	{ 0x79, 1024, 8, 512 + 16},   // 128 MB
	{ 0x49, 1024, 16, 512 + 16},  // 128 MB
	{ 0x59, 1024, 16, 512 + 16},  // 128 MB x16
	{ 0x78, 1024, 8, 512 + 16},   // 128 MB x16
	{ 0x72, 1024, 16, 512 + 16},  // 256 MB
	{ 0x74, 1024, 16, 512 + 16},  // 128 MB
	{ 0xA1, 1024, 8, 2048 + 64},  // 128 MB x16
	{ 0xF1, 1024, 8, 2048 + 64},  // 128 MB x16
	{ 0xB1, 1024, 16, 2048 + 64}, // 128 MB
	{ 0xC1, 1024, 16, 2048 + 64}, // 256 MB (4th ID byte will be checked)
	{ 0xAA, 1024, 8, 2048 + 64},  // 256 MB x16 (4th ID byte will be checked)
	{ 0xDA, 2048, 8, 2048 + 64},  // 256 MB x16 (4th ID byte will be checked)   //REVISIT
	{ 0xBA, 2048, 16, 2048 + 64}, // 256 MB (4th ID byte will be checked)
	{ 0xCA, 2048, 16, 2048 + 64}, // 512 MB (4th ID byte will be checked)
	{ 0x71, 2048, 8, 512 + 16},   // 512 MB x16 (4th ID byte will be checked)
	{ 0x51, 2048, 16, 512 + 16},  // 512 MB x16 (4th ID byte will be checked)
	{ 0x31, 2048, 8, 512 + 16},   // 512 MB (4th ID byte will be checked)
	{ 0x41, 2048, 16, 512 + 16},  // 1 GB (4th ID byte will be checked)
	{ 0xAC, 4096, 8, 2048 + 64},  // 1 GB x16 (4th ID byte will be checked)
	{ 0xDC, 4096, 8, 2048 + 64},  // 1 GB x16 (4th ID byte will be checked)
	{ 0xBC, 4096, 16, 2048 + 64}, // 1 GB (4th ID byte will be checked)
	{ 0xCC, 4096, 16, 2048 + 64}, // 2 GB (4th ID byte will be checked)
	{ 0xA3, 8192, 8, 2048 + 64},  // 2 GB x16 (4th ID byte will be checked)
	{ 0xD3, 8192, 8, 2048 + 64},  // 2 GB x16 (4th ID byte will be checked)
	{ 0xB3, 8192, 16, 2046 + 64}, // 2 GB (4th ID byte will be checked)
	{ 0xC3, 8192, 16, 2048 + 64}, // 2 GB x16 (4th ID byte will be checked)
	{ 0xA5, 16384, 8, 2048 + 64}, // 2 GB x16 (4th ID byte will be checked)
	{ 0xD5, 16384, 8, 2046 + 64}, // 2 GB (4th ID byte will be checked)
	{ 0xB5, 16384, 16, 2048 + 64},// 2 GB x16 (4th ID byte will be checked)
	{ 0xC5, 16384, 16, 2046 + 64},// 2 GB (4th ID byte will be checked)
	{ 0xA7, 32768, 8, 2048 + 64}, // 2 GB x16 (4th ID byte will be checked)
	{ 0xB7, 32768, 16, 2046 + 64},// 2 GB (4th ID byte will be checked)
	{ 0xAE, 65536, 8, 2048 + 64}, // 2 GB x16 (4th ID byte will be checked)
	{ 0xD5, 65536, 16, 2046 + 64},// 2 GB (4th ID byte will be checked)
	{ 0x00, 0, 0, 0} // Dummy null entry to indicate end of table
};

GPMC_Config_t GPMC_ConfigNANDDefault = {
	/* SysConfig: GPMC_SYSCONFIG : No Idle Mode */
	0x00000008,
	/* IRQStatus: Wait0 edge detection is reset*/
	0x00000100,
	/* IRQEnable: Wait0 edge detection is enabled */
	0x00000200,
	/* TimeOutControl */
	0x00000000,
	/* Config:  WAIT0 active low , WP output pin is high , Limited Address device support*/
	0x00000012,
	{
		// Rom Code Defaults.
		0x00000810,
		0x001e1e00,
		0x001e1e00,
		0x16051807,
		0x00151e1e,
		0x16000f80,
		0x00000048,
	},
};


const AM335X_ASYNC_MEM_DEVICE_InterfaceObj
DEVICE_ASYNC_MEM_interfaces[AM335X_DEVICE_ASYNC_MEM_INTERFACE_CNT] = {
	{
		AM335X_AYSNC_MEM_INTERFACE_TYPE_GPMC,
		NULL, //(void *) AEMIF,
		AM335X_DEVICE_ASYNC_MEM0_REGION_CNT,
		0, //DEVICE_ASYNC_MEM_regionStarts,
		0, //DEVICE_ASYNC_MEM_regionSizes
	}
};

AM335X_ASYNC_MEM_DEVICE_InfoObj AM335X_DEVICE_ASYNC_MEM_info = {
	AM335X_DEVICE_ASYNC_MEM_INTERFACE_CNT, // interfaceCnt
	DEVICE_ASYNC_MEM_interfaces, // interfaces
	&(DEVICE_ASYNC_MEM_Init), // fxnOpen
	&(DEVICE_ASYNC_MEM_IsNandReadyPin) // fxnNandIsReadyPin;
};

static pin_muxing_t PAD_Conf_NANDFlash[] = {
	{ GPMC_AD0_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD1_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD2_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD3_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD4_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD5_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD6_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_AD7_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_WAIT0_OFF, MODE(0) | PULLUP_EN| RXACTIVE},
	{ GPMC_WPN_OFF, MODE(0) | RXACTIVE | PULLUP_EN},
	{ GPMC_CSN0_OFF, MODE(0) | PULLUPDOWN_DISABLE},
	{ GPMC_ADVN_ALE_OFF, MODE(0) | PULLUPDOWN_DISABLE},
	{ GPMC_OEN_REN_OFF, MODE(0) | PULLUPDOWN_DISABLE},
	{ GPMC_WEN_OFF, MODE(0) | PULLUPDOWN_DISABLE},
	{ GPMC_BE0N_CLE_OFF, MODE(0) | PULLUPDOWN_DISABLE},
	{ 0xFFFFFFFF },
};
/************************************************************
* Global Function Definitions                               *
************************************************************/
void PAD_ConfigMux(pin_muxing_t	*pin_mux)
{
	Uint8 i;

	for (i = 0; i < 50; i++) {
		if(pin_mux[i].offset == 0xffffffff)
			break;
		*(volatile Uint32 *)(CONTROL_MODULE + pin_mux[i].offset) = pin_mux[i].val;
	}

}

Uint32 AM335X_Start() {
	DEBUG_printString("Starting AM335X NAND writer");
	AM335X_UTIL_setCurrMemPtr(0);

	if (AM335X_DEVICE_init() != E_PASS) {
		return E_FAIL;
	}

	// Start the Nandwrite after getting NAND Info handle.
	return nandwriter(AM335X_NAND_open(GPMC_base_CS_0, AM335X_DEVICE_BUSWIDTH_8BIT));

}

AM335X_NAND_InfoHandle AM335X_NAND_open(Uint32 base_CS, Uint8 busWidth) {
	AM335X_NAND_InfoHandle hNandInfo;

	// Set NandInfo handle
	hNandInfo = (AM335X_NAND_InfoHandle) AM335X_UTIL_allocMem(sizeof (NAND_InfoObj));

	
	PAD_ConfigMux(PAD_Conf_NANDFlash);
	// Open Async Memory peripheral
	hNandInfo->hAsyncMemInfo = AM335X_ASYNC_MEM_Open(
			AM335X_AYSNC_MEM_TYPE_NAND,
			base_CS,
			busWidth);

	// Set NAND flash base address
	hNandInfo->flashBase = base_CS;

	// Init the current block number and good flag
	hNandInfo->currBlock = -1;
	hNandInfo->isBlockGood = FALSE;

	// Use device specific page layout and ECC layout
	hNandInfo->hPageLayout     = &AM335X_DEVICE_NAND_PAGE_layout;
	hNandInfo->hEccInfo        = &AM335X_DEVICE_NAND_ECC_info;
	hNandInfo->hBbInfo         = &AM335X_DEVICE_NAND_BB_info;
	hNandInfo->hChipInfo       = DEVICE_NAND_CHIP_infoTable;
	hNandInfo->CSOffset        = hNandInfo->hAsyncMemInfo->chipSelectNum;
	hNandInfo->nandaleoffset   = AM335X_DEVICE_NAND_ALE_OFFSET;
	hNandInfo->nandcleoffset   = AM335X_DEVICE_NAND_CLE_OFFSET;
	hNandInfo->nanddataoffset  = AM335X_DEVICE_NAND_DATA_OFFSET;
	hNandInfo->nandhardware.endaddddr         = AM335X_DEVICE_DDR2_END_ADDR;
	hNandInfo->nandhardware.startaddddr       = AM335X_DEVICE_DDR2_START_ADDR;
	hNandInfo->nandhardware.startrbl          = AM335X_DEVICE_NAND_RBL_SEARCH_START_BLOCK;
	hNandInfo->nandhardware.endrbl            = AM335X_DEVICE_NAND_RBL_SEARCH_END_BLOCK;
	hNandInfo->nandhardware.maxbyteperop      = AM335X_DEVICE_NAND_MAX_BYTES_PER_OP; 
	hNandInfo->nandhardware.minsparebyteperop = 3;
	hNandInfo->nandhardware.maxsparebyteperop = 16;
	hNandInfo->nandhardware.timeout           = AM335X_DEVICE_NAND_TIMEOUT;
	hNandInfo->nandhardware.enduboot          = AM335X_DEVICE_NAND_UBOOT_SEARCH_END_BLOCK;
	hNandInfo->nandhardware.fxnsetECC         = &AM335X_Device_setECC;
	hNandInfo->nandhardware.fxnsetNand_16     = &setNand_16;
	hNandInfo->busWidth        = hNandInfo->hAsyncMemInfo->busWidth;

	// Send reset command to NAND
	if (AM335X_NAND_reset(hNandInfo) != E_PASS)
		return NULL;

	// Get and set device details
	if (AM335X_flashGetDetails(hNandInfo) != E_PASS)
		return NULL;

	// Send reset command to NAND
	if (AM335X_NAND_reset(hNandInfo) != E_PASS)
		return NULL;
	return hNandInfo;
}

Uint32 AM335X_DEVICE_init() {
	Uint32 status = E_PASS;

	AM335X_DEVICE_setPad();
	return status;
}

void GPMC_Write(Uint32 base, Uint32 offset, Uint32 val) {
	*(volatile Uint32 *)(base + offset) = val;
}

static Uint32 AM335X_DEVICE_setGPMC(GPMC_Config_t *cfg, Uint32 base_CS) {
	*(unsigned int*)CM_PER_GPMC_CLKCTRL = 0x02; /*Enable GPMC Clock*/   //REVISIT
	while((*(unsigned int*)(CM_PER_GPMC_CLKCTRL ) & 0x3) !=0x2);
	*(unsigned int*)(CM_PER_ELM_CLKCTRL) = 0x02; /*Enable GPMC Clock*/   //REVISIT
			while((*(unsigned int*)(CM_PER_ELM_CLKCTRL ) & 0x3) !=0x2);
	//GPMC_SYSCONFIG : No Idle Mode
	GPMC_Write(GPMC_SYSCONFIG, 0, cfg->SysConfig);
	// IRQStatus: Wait0 edge detection is reset
	GPMC_Write(GPMC_IRQSTATUS, 0, cfg->IrqStatus);
	// GPMC_IRQENABLE :Wait0 edge detection is enabled
	GPMC_Write(GPMC_IRQENABLE, 0, cfg->IrqEnable);
	// GPMC_CONFIG: WAIT0 active low , WP output pin is high , Limited Address device support
	GPMC_Write(GPMC_CONFIG, 0, cfg->Config);

	// Switch off the CS before configure
	GPMC_Write(base_CS, GPMC_CONFIG7, 0);
	// CS Cofiguration Register.
	GPMC_Write(base_CS, GPMC_CONFIG1, cfg->ChipSelectConfig[0]);
	GPMC_Write(base_CS, GPMC_CONFIG2, cfg->ChipSelectConfig[1]);
	GPMC_Write(base_CS, GPMC_CONFIG3, cfg->ChipSelectConfig[2]);
	GPMC_Write(base_CS, GPMC_CONFIG4, cfg->ChipSelectConfig[3]);
	GPMC_Write(base_CS, GPMC_CONFIG5, cfg->ChipSelectConfig[4]);
	GPMC_Write(base_CS, GPMC_CONFIG6, cfg->ChipSelectConfig[5]);
	GPMC_Write(base_CS, GPMC_CONFIG7, cfg->ChipSelectConfig[6]);
	return E_PASS;
}

AM335X_ASYNC_MEM_InfoHandle AM335X_ASYNC_MEM_Open(AM335X_ASYNC_MEM_Type memType,
		Uint32 baseAddress, Uint8 busWidth) {
	AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo;
	hAsyncMemInfo = (AM335X_ASYNC_MEM_InfoHandle)
		AM335X_UTIL_callocMem(sizeof (AM335X_ASYNC_MEM_InfoObj));

	// Fill in structure
	hAsyncMemInfo->hDeviceInfo = &AM335X_DEVICE_ASYNC_MEM_info;
	hAsyncMemInfo->memType = memType;
	hAsyncMemInfo->busWidth = busWidth;
	hAsyncMemInfo->interfaceNum = 0;
	hAsyncMemInfo->chipSelectNum = 0;
	hAsyncMemInfo->regs = NULL;

	if (AM335X_DEVICE_setGPMC(&GPMC_ConfigNANDDefault, baseAddress) != E_PASS) {  //REVISIT
		return NULL;
	}

	return hAsyncMemInfo;
}

/************************************************************
* Local Function Definitions                                *
************************************************************/


static void AM335X_DEVICE_setPad() {
	/* configure pad - pinmuxing */
	// No Pin Configuration required in AM335X 
}

static void DEVICE_ASYNC_MEM_Init(AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo) {
	// do nothing in this program;
}

static Uint8 DEVICE_ASYNC_MEM_IsNandReadyPin(AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo) {
	Uint32 *ptr = (Uint32 *) GPMC_STATUS; // The GPMC_STATUS register to check wait 0 status
	return ((*ptr >> 8)& 1); // check the 8 bit of the status register.
}
static void setNand_16(Uint32 base_cs) {
	Uint32 *ptr = (Uint32 *) (base_cs + GPMC_CONFIG1); //GPMC_CONFIG1 :
	*ptr = *ptr | 0x00001000; // Make GPMC_CONFIG1_0 (Device Size) 16bit.
}
