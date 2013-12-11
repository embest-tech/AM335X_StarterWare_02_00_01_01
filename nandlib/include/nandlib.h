/**
 *  \file   nandlib.h
 *
 *  \brief  Definitions used for NAND Abstractin Library.
 *
 *   This file contains the macros, structures, function prototypes used by 
 *   nand application, abstraction layer, and platform spcific layer.
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


#ifndef _NANDLIB_H_
#define _NANDLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*                         MACRO & ENUM DEFINITIONS
*******************************************************************************/

/*****************************************************************************/
/*
** Macros which defines the number of NAND devices can be connected to system 
*/
#define NAND_MAX_CHIP_SELECTS                         (4)

/*****************************************************************************/
/*
** Macros which can be used as 'chip select' to the NANDInfoInit, 
** NANDCtrlInfoInit and NANDDevInfoInit functions.
*/
#define NAND_CHIP_SELECT_0                            (0)
#define NAND_CHIP_SELECT_1                            (1)
#define NAND_CHIP_SELECT_2                            (2)
#define NAND_CHIP_SELECT_3                            (3)
#define NAND_CHIP_SELECT_4                            (4)
#define NAND_CHIP_SELECT_5                            (5)
#define NAND_CHIP_SELECT_6                            (6)
#define NAND_CHIP_SELECT_7                            (7)

/*****************************************************************************/
/*
** Macros which defines the ECC offset in OOB areaa, ecc byte count(size)
** of diffrent ECC schemes.
**
*/
#define NAND_ECC_1BIT_HAMMINGCODE_OOB_OFFSET          (1)
#define NAND_ECC_1BIT_HAMMINGCODE_BYTECNT             (3)

#define NAND_ECC_RS_4BIT_OOB_OFFSET                   (0)
#define NAND_ECC_RS_4BIT_UNUSED_BYTECNT               (6)
#define NAND_ECC_RS_4BIT_BYTECNT                      (10)

#define NAND_ECC_BCH_8BIT_OOB_OFFSET                  (2)
#define NAND_ECC_BCH_8BIT_BYTECNT                     (14)
#define NAND_ECC_BCH_8BIT_UNUSED_BYTECNT              (2)
#define NAND_ECC_BCH_8BIT_NIBBLECNT                   (26)

/*****************************************************************************/
/*
** Macros which defines the last data and ecc bit in diffrent ECC schemes.
**
*/
/* 512 bytes of data plus 13 bytes ECC */
#define NAND_ECC_BCH_8BIT_LASTDATABIT                 ((512 + 13) * 8)
/* 13 bytes of ECC data plus 1 byte of ECC for ECC data */
#define NAND_ECC_BCH_8BIT_LASTECCBIT                  ((13 + 1) * 8)

/*****************************************************************************/
/*
** Macro which defines the number of bytes sent/received per transfer.
**
*/
#define NAND_BYTES_PER_TRNFS                          (512)
#define NAND_MAX_ECC_BYTES_PER_TRNFS                  (16)


/*****************************************************************************/
/*
** Macro which defines the values stored in OOB as bad block info.
**
*/
#define NAND_BLK_GOOD_MARK                            (0xFF)
#define NAND_BLK_BAD_MARK                             (0)

/*****************************************************************************/
/*
** Macros which can be used as 'busWidth' in NANDDevInfo structure.
** 
*/
typedef enum _NAND_BUSWIDTH_
{
    NAND_BUSWIDTH_INVALID   = (0xFF),
    NAND_BUSWIDTH_8BIT      = (0x00),
    NAND_BUSWIDTH_16BIT     = (0x01)
}
NandBusWidth_t;

/*****************************************************************************/
/*
** Nand ECC alogorithm typedef
**
*/
typedef enum _NAND_ECC_ALGO_
{
    NAND_ECC_ALGO_NONE          =   (0x00),
    NAND_ECC_ALGO_HAMMING_1BIT  =   (0x01),
    NAND_ECC_ALGO_RS_4BIT       =   (0x02),
    NAND_ECC_ALGO_BCH_4BIT      =   (0x04),
    NAND_ECC_ALGO_BCH_8BIT      =   (0x08),
    NAND_ECC_ALGO_BCH_16BIT     =   (0x10)
}
NandEccAlgo_t;

/*****************************************************************************/
/*
** NandPageSize typedef
**
*/
typedef enum _NAND_PAGE_SIZE_
{
    NAND_PAGESIZE_INVALID   =   ((0)*(512)),
    NAND_PAGESIZE_512BYTES  =   ((1)*(512)),
    NAND_PAGESIZE_2048BYTES =   ((4)*(512)),
    NAND_PAGESIZE_4096BYTES =   ((8)*(512)),
    NAND_PAGESIZE_8192BYTES =   ((16)*(512))
}
NandPageSize_t;

/*****************************************************************************/
/*
** NandBlockSize typedef
**
*/
typedef enum _NAND_BLOCK_SIZE_
{
    NAND_BLOCKSIZE_INVALID   =   ((0)*(1024)),
    NAND_BLOCKSIZE_64KB      =   ((64)*(1024)),
    NAND_BLOCKSIZE_128KB     =   ((128)*(1024)),
    NAND_BLOCKSIZE_256KB     =   ((256)*(1024)),
    NAND_BLOCKSIZE_512KB     =   ((512)*(1024))
}
NandBlockSize_t;

/*****************************************************************************/
/*
** NandStatus typedef - returned from most NandLib APIs
**
*/
typedef enum _NAND_STATUS_
{
    NAND_STATUS_PASSED                          =  (0x001),
    NAND_STATUS_FAILED                          =  (0x002),
    NAND_STATUS_NOT_FOUND                       =  (0x004),    
    NAND_STATUS_DEVBUSY                         =  (0x008),
    NAND_STATUS_DEVWRPROTECT                    =  (0x010),
    NAND_STATUS_WAITTIMEOUT                     =  (0x020),
    NAND_STATUS_READWRITE_DMA_FAIL              =  (0x040),
    NAND_STATUS_ECC_UNSUPPORTED                 =  (0x080),
    NAND_STATUS_READ_ECC_ERROR_CORRECTED        =  (0x100),
    NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR    =  (0x200)
}
NandStatus_t;

/*****************************************************************************/
/*
** NandBlockStatus typedef - returned from back block checking routine
**
*/
typedef enum _NAND_BLOCK_STATUS_
{
    NAND_BLOCK_GOOD                              = (0),
    NAND_BLOCK_BAD                               = (1),
    NAND_BLOCK_SPARE_AREA_READ_FAILED            = (2)
}
NandBlockStatus_t;

/*****************************************************************************/
/*
** Typedef which is used as 'dir' to DMAXfer, DMAXferSetup funcitons to indicate
** the transfer direction. 
**
*/
typedef enum _NAND_DMA_DIR_
{
    NAND_DMA_DIR_READ   =  (0),
    NAND_DMA_DIR_WRITE  =  (1)
}
NandDmaDir_t;

/*****************************************************************************/
/*
** Typedef which can be used as 'opMode' in NandInfo_t structure.
**
*/
typedef enum _NAND_XFER_MODE_
{
    NAND_XFER_MODE_CPU  =   (0),
    NAND_XFER_MODE_DMA  =   (1)
}
NandXFerMode_t;


/*******************************************************************************
*                           STRUCTURES DECLARION
*******************************************************************************/


/* A consolidated structure which is the main object instantiated by the user. 
** This contains all the other information for a NAND device/controller 
** transaction. It contains the info like --
**      1)NAND Device information that is connected.
**      2)NAND Controller information that is being used.
**      3)Function pointers for control/config/transfer.
**
*/
typedef struct _NAND_INFO_
{
    /* Array of memory controller chip selects in use */
    int chipSelects[NAND_MAX_CHIP_SELECTS];

    /* Opmode. i.e CPU or DMA  */
    NandXFerMode_t opMode;
    
    /* ECC algorithm required by the NAND device */
    NandEccAlgo_t eccType;

    /* Manufacturer ID  */
    unsigned char manId;
    /* Device ID        */
    unsigned char devId;
    
    /* Count of chip selects the nand device uses */
    int chipSelectCnt;
    /* NAND die cnt - how many die in package */
    int dieCnt;
    /* Bus width of the device. i,e 8 bit or 16 bit */
    NandBusWidth_t busWidth;
    /* Page size of the device */
    NandPageSize_t pageSize;
    /* Blk size (with out spare area) of the device */
    NandBlockSize_t blkSize;
    /* Pages per block */
    unsigned int pagesPerBlk;

    /* DATA Register address */
    unsigned int dataRegAddr;
    /* ADDR Register address */
    unsigned int addrRegAddr;
    /* CMD Register address */
    unsigned int cmdRegAddr;

    /* Pointer to Memory Controller Structure */
    struct _NAND_CTRL_INFO_ *hNandCtrlInfo;
    /* Pointer to ECC Structure */
    struct _NAND_ECC_INFO_ *hNandEccInfo;
    /* Pointer to DMA Structure */
    struct _NAND_DMA_INFO_ *hNandDmaInfo;
}
NandInfo_t;


/* Contains the controller information like ---
**      1) chip select
**      2) Location for data, command and address registers
**      3) ECC supported by the controller and the default ecc to use.
**      4) Wait pin onformation
**      5) DMA related information and so on
*/
typedef struct _NAND_CTRL_INFO_
{
    /* Function to initialize the NAND controller before accessing the NAND */
    NandStatus_t (*CtrlInit)(NandInfo_t *nandInfo);
    /* Function to get the wait pin status */
    unsigned int (*WaitPinStatusGet)(NandInfo_t *nandInfo);
    /* Function to get the GPMC FIFO status */
    unsigned int (*WriteBufReady) (NandInfo_t *nandInfo);
    /* Timing info for the device and the controller */
    void *hNandTimingInfo;
    /* Base address of the controller */
    unsigned int baseAddr;
    /* ECC supported by the controller */
    unsigned int eccSupported;
    /* Wait pin where NAND dev R/B pin is connected  */
    unsigned int waitPin;
    /* Wait pin polarity  */
    unsigned int waitPinPol;
    /* Write protect pin polarity  */
    unsigned int wpPinPol;
    /* Chip select base address  */    
    unsigned int chipSelectBaseAddr[NAND_MAX_CHIP_SELECTS];
    /* Chip select region size  */
    unsigned int chipSelectRegionSize[NAND_MAX_CHIP_SELECTS];    
    /* Curr chip select in use by the memory controller */
    int currChipSelect;
}
NandCtrlInfo_t;

typedef struct _NAND_ECC_INFO_
{
    /* Base address of the ECC engine (ELM in GPMC for BCH ECC) */
    unsigned int baseAddr;
    /* Offset of the page from where ECC has to store.*/
    unsigned int eccOffSet;
    /* Total number of ecc bytes. */
    unsigned int eccByteCnt;
    /* Function to initialize the controller w.r.t ECC */
    NandStatus_t (*ECCInit)(NandInfo_t *nandInfo);
    /* Function to enable the ECC */
    void (*ECCEnable)(NandInfo_t *nandInfo);
    /* Function to disable the ECC */
    void (*ECCDisable)(NandInfo_t *nandInfo);
    /* Function which does the ECC setting for write */
    void (*ECCWriteSet)(NandInfo_t *nandInfo);
    /* Function which does the ECC setting for read */
    void (*ECCReadSet)(NandInfo_t *nandInfo);
    /* Function to caluclate/read the ECC */
    void (*ECCCalculate)(NandInfo_t *nandInfo, unsigned char *eccData);
    /* Function to check for the ECC errors, and correct if any errors  */
    NandStatus_t (*ECCCheckAndCorrect)(NandInfo_t *nandInfo,
                                       unsigned char *readEcc,
                                       unsigned char *data);
}
NandEccInfo_t;

typedef struct _NAND_DMA_INFO_
{
    /* Function to Initialize DMA */
    void (*DMAInit)(void *unused);
    /* Function to transfer the Data through DMA */
    NandStatus_t (*DMAXfer)(NandInfo_t *nandInfo,
                            unsigned char *data,
                            unsigned int len, 
                            NandDmaDir_t dir);
    /* Function to setup the DMA transfer */
    void (*DMAXferSetup)(NandInfo_t *nandInfo,
                         unsigned char *data, unsigned int len,
                         NandDmaDir_t dir);
    /* Function to get the status of the DMA transfer */
    unsigned int (*DMAXferStatusGet)();
}
NandDmaInfo_t;


/*******************************************************************************
*                           FUNCTION PROTOTYPE DECLARATION
*******************************************************************************/

/* NAND APIs */
extern NandStatus_t         NANDOpen(NandInfo_t *nandInfo);
extern NandStatus_t         NANDReadId(NandInfo_t *nandInfo);
extern NandStatus_t         NANDReset(NandInfo_t *nandInfo);
extern NandStatus_t         NANDWaitUntilReady(NandInfo_t *nandInfo);

extern NandBlockStatus_t    NANDBadBlockCheck(NandInfo_t *nandInfo, unsigned int blkNum);
extern NandStatus_t         NANDMarkBlockAsBad(NandInfo_t *nandInfo, unsigned int blkNum);

extern NandStatus_t         NANDBlockErase(NandInfo_t *nandInfo, unsigned int blkNum);
extern NandStatus_t         NANDPageWrite(NandInfo_t *nandInfo, unsigned int blkNum,
                                          unsigned int pageNum,
                                          volatile unsigned char *txData,
                                          unsigned char *eccData);
extern NandStatus_t         NANDPageRead(NandInfo_t *nandInfo, unsigned int blkNum,
                                         unsigned int pageNum,
                                         volatile unsigned char *rxData,
                                         unsigned char *eccData);

#ifdef __cplusplus
}
#endif

#endif /* _NANDLIB_H_ */
