/*
 * nand.h
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
 * FILE      :  nand.h
 * PROJECT   :  TI Booting and Flashing Utilities
 * AUTHOR    :  Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		Vinay Agrawal (vinayagw@ti.com)
 * DESC      : Generic low-level NAND driver header
 * --------------------------------------------------------------------------
 */

#ifndef _NAND_H_
#define _NAND_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/

// An assumption is made that ARM-based devices use Linux and
// therefore use the last four blocks of the NAND as space for
// an MTD standard bad block table. DSP-based are assumed to
// not run Linux, and bad blocks are managed in some other fashion.
#if ( defined(__TMS470__) | defined(__GNUC__) )
#define AM335X_NAND_NUM_BLOCKS_RESERVED_FOR_BBT     (4)
#else
#define AM335X_NAND_NUM_BLOCKS_RESERVED_FOR_BBT     (0)
#endif

// NAND flash commands
#define AM335X_NAND_LO_PAGE          (0x00)
#define AM335X_NAND_HI_PAGE          (0x01)
#define AM335X_NAND_EXTRA_PAGE       (0x50)

#define AM335X_NAND_LOCK             (0x2A)
#define AM335X_NAND_UNLOCK_START     (0x23)
#define AM335X_NAND_UNLOCK_END       (0x24)

#define AM335X_NAND_READ_PAGE        (0x00)
#define AM335X_NAND_READ_30H         (0x30)
#define AM335X_NAND_RANDOM_READ_PAGE (0x05)
#define AM335X_NAND_RANDOM_READ_E0H  (0xE0)

#define	AM335X_NAND_RDID             (0x90)
#define AM335X_NAND_RDID2            (0x65)
#define AM335X_NAND_RDIDADD          (0x00)

#define	AM335X_NAND_RESET            (0xFF)
#define	AM335X_NAND_STATUS           (0x70)
#define AM335X_NAND_RDY              (0x40)

#define	AM335X_NAND_PGRM_START       (0x80)
#define	AM335X_NAND_RANDOM_PGRM      (0x85)
#define AM335X_NAND_PGRM_END         (0x10)
#define	AM335X_NAND_PGM_FAIL         (0x01)

#define	AM335X_NAND_BERASEC1         (0x60)
#define	AM335X_NAND_BERASEC2         (0xD0)


// NAND ONFI commands
#define AM335X_NAND_ONFIRDIDADD        (0x20)
#define AM335X_NANDONFI_STRING         (0x49464E4F)
#define AM335X_NANDONFI_RDPARAMPAGE    (0xEC)
#define AM335X_NANDONFI_

// Status Output
#define AM335X_NAND_NANDFSR_READY      (0x01)
#define AM335X_NAND_STATUS_WRITEREADY  (0xC0)
#define AM335X_NAND_STATUS_ERROR       (0x01)
#define AM335X_NAND_STATUS_READY       (0x40)
#define AM335X_NAND_STATUS_PROTECTED   (0x80)

#define AM335X_NAND_MAX_PAGE_SIZE          (8192)
#define AM335X_NAND_MAX_BYTES_PER_OP       AM335X_DEVICE_NAND_MAX_BYTES_PER_OP
#define AM335X_NAND_MIN_SPAREBYTES_PER_OP  AM335X_DEVICE_NAND_MIN_SPAREBYTES_PER_OP    // Min Spare Bytes per operation

// Macro gets the page size in bytes without the spare bytes
#define AM335X_NANDFLASH_PAGESIZE(x) ( ( x >> 8 ) << 8 )

/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

typedef enum AM335X_DEVICE_BUSWIDTH_ {
	AM335X_DEVICE_BUSWIDTH_8BIT = BUS_8BIT,
	AM335X_DEVICE_BUSWIDTH_16BIT = BUS_16BIT
}AM335X_DEVICE_BusWidth;

typedef enum AM335X_ASYNC_MEM_TYPE_ {
	AM335X_AYSNC_MEM_TYPE_SRAM = 0x01,
	AM335X_AYSNC_MEM_TYPE_NOR = 0x02,
	AM335X_AYSNC_MEM_TYPE_NAND = 0x03,
	AM335X_AYSNC_MEM_TYPE_ONENAND = 0x04
}AM335X_ASYNC_MEM_Type;

// ASYNC_MEM_INFO structure - holds pertinent info for open driver instance

typedef struct AM335X_ASYNC_MEM_NFO_ {
	AM335X_ASYNC_MEM_Type memType;  // Type of memory
	Uint8 busWidth;                 // Operating bus width
	Uint8 interfaceNum;             // Async Memory Interface Number
	Uint8 chipSelectNum;            // Operating chip select
	void *regs;                     // Configuration register overlay
	struct AM335X_ASYNC_MEM_DEVICE_INFO_ *hDeviceInfo;
}AM335X_ASYNC_MEM_InfoObj, *AM335X_ASYNC_MEM_InfoHandle;

// Supported asynchronous memory interface type hAsyncMemInfo->hDeviceInfo->interfaces[i]

typedef enum AM335X_ASYNC_MEM_DEVICE_INTERFACE_TYPE_ {
	AM335X_AYSNC_MEM_INTERFACE_TYPE_EMIF2 = 0x01,
	AM335X_AYSNC_MEM_INTERFACE_TYPE_GPMC = 0x02
}AM335X_ASYNC_MEM_DEVICE_InterfaceType;

typedef struct AM335X_ASYNC_MEM_DEVICE_INTERFACE_ {
	const AM335X_ASYNC_MEM_DEVICE_InterfaceType type;
	const void *regs;
	const Uint32 regionCnt;
	const Uint32 *regionStarts;
	const Uint32 *regionSizes;
}AM335X_ASYNC_MEM_DEVICE_InterfaceObj, *AM335X_ASYNC_MEM_DEVICE_InterfaceHandle;

typedef void (*AM335X_ASYNC_MEM_DEVICE_Init)(AM335X_ASYNC_MEM_InfoHandle AM335X_hAsyncMemInfo);
typedef void (*AM335X_ASYNC_MEM_DEVICE_Close)(AM335X_ASYNC_MEM_InfoHandle AM335X_hAsyncMemInfo);
typedef Uint8(*AM335X_ASYNC_MEM_DEVICE_IsNandReadyPin)(AM335X_ASYNC_MEM_InfoHandle AM335X_hAsyncMemInfo);

typedef struct AM335X_ASYNC_MEM_DEVICE_INFO_ {
	const Uint8 interfaceCnt;
	const AM335X_ASYNC_MEM_DEVICE_InterfaceObj *interfaces;
	const AM335X_ASYNC_MEM_DEVICE_Init fxnInit;
	const AM335X_ASYNC_MEM_DEVICE_IsNandReadyPin fxnNandIsReadyPin;
}AM335X_ASYNC_MEM_DEVICE_InfoObj, *AM335X_ASYNC_MEM_DEVICE_InfoHandle;

// Region Types

typedef enum AM335X_NAND_REGION_TYPE_ {
	NAND_REGION_DATA = 0x0, // Data Region
	NAND_REGION_SPARE = 0x1 // Spare Region
}AM335X_NAND_RegionType;

// Offset Types

typedef enum AM335X_NAND_OFFSET_TYPE_ {
	NAND_OFFSETS_RELTODATA = 0x0, // Offsets are relative to start of the data region of the page
	NAND_OFFSETS_RELTOSPARE = 0x1 // Offsets are relative to start of spare bytes region
}AM335X_NAND_OffsetType;

typedef void (*setNand16b)(Uint32 base_cs); // typedef of pointer to fuction which set GPMC parameter for NAND 16bit.
typedef Uint32(*setECC)(Uint32 busWidth, Uint32 eccType);

typedef struct AM335X_NAND_HARDWARE_INFO_ {
	Int32 timeout;
	Int32 maxbyteperop;
	Int32 maxsparebyteperop;
	Int32 minsparebyteperop;
	Uint32 startaddddr;
	Uint32 endaddddr;
	Int32 startrbl;
	Int32 endrbl;
	Int32 enduboot;
	setNand16b fxnsetNand_16;
	setECC fxnsetECC;
} AM335X_NAND_HARDWARE_InfoObj, *AM335X_NAND_HARDWARE_InfoHandle;

// NAND_INFO structure - holds pertinent info for open driver instance

typedef struct AM335X_NAND_INFO_ {
	AM335X_ASYNC_MEM_InfoHandle hAsyncMemInfo;
	AM335X_NAND_HARDWARE_InfoObj nandhardware;
	Uint32 flashBase;             // Base address of CS memory space where NAND is connected
	Uint8 busWidth;               // NAND bus width
	Uint8 manfID;                 // NAND manufacturer ID (just for informational purposes)
	Uint8 devID;                  // NAND_TABLE_Devices index (device ID)
	Uint32 numBlocks;             // block count per device
	Uint16 pagesPerBlock;         // page count per block
	Uint16 dataBytesPerPage;      // Number of bytes in a page
	Uint16 spareBytesPerPage;     // Number of spare bytes in a page
	Uint16 dataBytesPerOp;        // Number of bytes per operation
	Uint16 spareBytesPerOp;       // Number of spare bytes per operation
	Uint8 numOpsPerPage;          // Number of operations to complete a page read/write
	Uint8 numColAddrBytes;        // Number of Column address cycles
	Uint8 numRowAddrBytes;        // Number of Row address cycles
	Uint8 CSOffset;               // 0 for CS2 space, 1 for CS3 space, 2 for CS4 space, 3 for CS5 space
	Bool isLargePage;             // TRUE = Big block device, FALSE = small block device
	Bool isONFI;                  // TRUE = ONFI-compatible device, FALSE = non-ONFI device
	Int32 currBlock;              // current Block in use
	Bool isBlockGood;             // TRUE=current block is good, FALSE=block is bad
	Uint32 nandcleoffset;
	Uint32 nandaleoffset;
	Uint32 nanddataoffset;
	struct AM335X_NAND_PAGE_LAYOUT_ *hPageLayout;
	struct AM335X_NAND_ECC_INFO_ *hEccInfo;
	struct AM335X_NAND_BB_INFO_ *hBbInfo;
	struct AM335X_NAND_CHIP_INFO_ *hChipInfo;

}NAND_InfoObj, *AM335X_NAND_InfoHandle;

//NAND_DEVICE structure - holds supported devices' params

typedef struct AM335X_NAND_CHIP_INFO_ {
	const Uint8 devID;         // DeviceID
	const Uint32 capacity;     // number of blocks in device
	const Uint8 devicewidth;   // page count per block
	const Uint16 bytesPerPage; // byte count per page (include spare)
} AM335X_NAND_CHIP_InfoObj, *AM335X_NAND_CHIP_InfoHandle;

// Region object

typedef struct AM335X_NAND_REGION_ {
	const AM335X_NAND_RegionType regionType;
	const AM335X_NAND_OffsetType offsetType;
	const Uint16 bytesPerOp;
	const Uint16 offsets[16];
}AM335X_NAND_RegionObj, *AM335X_NAND_RegionHandle;

// Device specific page layout struct, consisting of two region objects

typedef struct AM335X_NAND_PAGE_LAYOUT_ {
	AM335X_NAND_RegionObj dataRegion;
	AM335X_NAND_RegionObj spareRegion;
}AM335X_NAND_PAGE_LayoutObj, *AM335X_NAND_PAGE_LayoutHandle;

// NAND ECC function typedefs
typedef void (*AM335X_NAND_ECC_Calculate)(AM335X_NAND_InfoHandle hNandInfo, Uint8 *data,
		Uint8 *calcECC);
typedef void (*AM335X_NAND_ECC_Store)(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes,
		Uint8 opNum, Uint8 *calcECC);
typedef void (*AM335X_NAND_ECC_Enable)(AM335X_NAND_InfoHandle hNandInfo);
typedef void (*AM335X_NAND_ECC_Disable)(AM335X_NAND_InfoHandle hNandInfo);
typedef void (*AM335X_NAND_ECC_Writeset)(void);
typedef void (*AM335X_NAND_ECC_Readset)(void);
typedef Uint32(*AM335X_NAND_ECC_Correct)(AM335X_NAND_InfoHandle hNandInfo, Uint32 pageLoc,
		Uint8 *spareBytes, Uint8 *data, Uint32 opNum);

// Device specific ECC info struct

typedef struct AM335X_NAND_ECC_INFO_ {
	Bool ECCEnable;                          // Error correction enable (should be on by default)
	Uint16 storedECCByteCnt;                 // Count of ECC bytes per op as stored in the NAND page
	Uint32 offset;                           // Offset require in spare byte regoin of the page.
	AM335X_NAND_ECC_Calculate fxnCalculate;  // this definations usage and implementation need to be understand.
	AM335X_NAND_ECC_Store fxnStore;
	AM335X_NAND_ECC_Enable fxnEnable;
	AM335X_NAND_ECC_Disable fxnDisable;
	AM335X_NAND_ECC_Correct fxnCorrect;
	AM335X_NAND_ECC_Writeset fxnWriteset;
	AM335X_NAND_ECC_Readset fxnReadset;
}AM335X_NAND_ECC_InfoObj, *AM335X_NAND_ECC_InfoHandle;

// NAND Bad Block function typedefs
typedef void (*AM335X_NAND_BB_MarkSpareBytes)(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes);
typedef Uint32(*AM335X_NAND_BB_CheckSpareBytes)(AM335X_NAND_InfoHandle hNandInfo, Uint8 *spareBytes);

// Device specific Bad block info struct

typedef struct AM335X_NAND_BB_INFO_ {
	const Bool BBMarkEnable;                          // We can mark bad blocks
	const Bool BBCheckEnable;                         // We can check for bad blocks
	const AM335X_NAND_BB_MarkSpareBytes fxnBBMark;    // Function pointer to mark spare bytes of page
	const AM335X_NAND_BB_CheckSpareBytes fxnBBCheck;  // Function pointer to check spare bytes of page
}AM335X_NAND_BB_InfoObj, *AM335X_NAND_BB_InfoHandle;

typedef union {
	Uint8 c;
	Uint16 w;
	Uint32 l;
}NAND_Data;

typedef union {
	VUint8 *cp;
	VUint16 *wp;
	VUint32 *lp;
}NAND_Ptr;


/************************************************************
* Global Function Declarations                              *
************************************************************/

extern Uint32 AM335X_NAND_reset(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_NAND_badBlockCheck(AM335X_NAND_InfoHandle hNandInfo, Uint32 block);
extern Uint32 AM335X_NAND_readPage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block,
		Uint32 page, Uint8 *dest);
extern Uint32 AM335X_NAND_readSpareBytesOfPage(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8 *dest);

#ifndef USE_IN_ROM
extern Bool AM335X_NAND_isWriteProtected(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_NAND_badBlockMark(AM335X_NAND_InfoHandle hNandInfo, Uint32 block);
extern Uint32 AM335X_NAND_writePage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block,
		Uint32 page, Uint8 *src);
extern Uint32 AM335X_NAND_writePage_uboot_header(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8 *src);
extern Uint32 AM335X_NAND_writeOnlySpareBytesOfPage(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8* spareBytes);
extern Uint32 AM335X_NAND_verifyPage(AM335X_NAND_InfoHandle hNandInfo, Uint32 block,
		Uint32 page, Uint8 *src, Uint8* dest);

extern Uint32 AM335X_NAND_globalErase(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_NAND_eraseBlocks(AM335X_NAND_InfoHandle hNandInfo, Uint32 startBlkNum,
		Uint32 blkCount);
extern Uint32 AM335X_NAND_globalErase_with_bb_check(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_NAND_eraseBlocks_with_bb_check(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 startBlkNum, Uint32 blkCnt);

extern Uint32 AM335X_NAND_verifyBlockErased(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint8* dest);

extern Uint32 AM335X_NAND_unProtectBlocks(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 startBlkNum, Uint32 endBlkNum);
extern void AM335X_NAND_protectBlocks(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_flashGetDetails(AM335X_NAND_InfoHandle hNandInfo);
extern Uint32 AM335X_NAND_readBCHSpare(AM335X_NAND_InfoHandle hNandInfo, Uint32 pageLoc,
		Uint8 *spareBytes, Uint32 opNum, Uint32 count);
#if defined(NAND_ECC_TEST)
extern Uint32 AM335X_NAND_writePageWithSpareBytes(AM335X_NAND_InfoHandle hNandInfo,
		Uint32 block, Uint32 page, Uint8 *src, Uint8 *spareBytes);
#endif

#endif

extern void Check_write(AM335X_NAND_InfoHandle hNandInfo, Uint32 block, Uint32 page);

/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif //_NAND_H_
