/**
 *  \file   nandReadWrite.c
 *
 *  \brief  The application writes default data pattern to the user
 *          specified block, page for number of pages and read the data
 *          and checks for the data integrity. If the macro
 *          NAND_DATAINTEGRITY_TEST_WITH_FIXED_ADDR is defined, application
 *          does the erase, write and read for default block and pages.
 *
 *          Application Configuration:
 *            Modules Used:
 *              GPMC
 *              NAND
 *              UART0
 *              EDMA
 *              Interrupt Controller
 *
 *              Configurable Parameters(Runtime)
 *                  None
 *
 *              Hard coded configurations(compile time)
 *                  Mode of Operation - DMA, Polling
 *                  ECC Type - Hamming, BCH-8bit
 *
 *            Application Use Cases:
 *               1. This application demonstartes writing and reading from
 *                  a specified block, Page in the NAND flash.
 *               2. The application performs Data Integrity test by verifying
 *                  the data read from the device is the same as the data
 *                  written to it.
 *
 *            Running the example:
 *               1. A serial terminal application should be running on the host.
 *                  On Running it displays NAND(MT29F2G08AB) Device Info such as 
 *                  MANUFACTURER ID, Device ID, PAGE SIZE, BLOCK SIZE etc..
 *               2. User is requested to provide input for block Number,
 *                  page number etc.. which user wants to write/read to.
 *               3. The application checks whether a block is bad or not,
 *                  if not it writes the data and reads it.
 *               4. After reading a Page it checks for ECC errors, if any
 *                  uncorrectable ECC error is found it prints an error message.
 *               5. If no ECC errors are found then it performs Data Integrity
 *                  Test and prints the result.
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*
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

#include "consoleUtils.h"
#include "hw_gpmc.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "gpmc.h"
#include "nandlib.h"
#include "nand_gpmc.h"
#include "nandDma.h"
#include "evmAM335x.h"
#include "cache.h"
#include "mmu.h"


/*******************************************************************************
*                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR                          (0x80000000u)
#define START_ADDR_DEV                          (0x44000000u)
#define START_ADDR_OCMC                         (0x40300000u)
#define START_ADDR_SRAM                         (0x402F0000u)

#define NUM_SECTIONS_DDR                        (512u)
#define NUM_SECTIONS_DEV                        (960u)
#define NUM_SECTIONS_OCMC                       (1u)
#define NUM_SECTIONS_SRAM                       (1u)

/* #define NAND_DATAINTEGRITY_TEST_WITH_FIXED_ADDR */

/*****************************************************************************/
/*
** Macros which defines attached device info like num of pages per block, 
** number of blocks, device ID and manufacturer ID.
**
*/
#define NAND_DATA_XFER_MODE                     (NAND_XFER_MODE_CPU)
#define NAND_BUSWIDTH                           (NAND_BUSWIDTH_8BIT)
#define NAND_CHIP_SELECT                        (NAND_CHIP_SELECT_0)
#define NAND_PAGE_SIZE_IN_BYTES                 (NAND_PAGESIZE_2048BYTES)
#define NAND_BLOCK_SIZE_IN_BYTES                (NAND_BLOCKSIZE_128KB)
#define NAND_NUMOF_BLK                          (2048)
#define NAND_MANUFATURER_MICRON_ID              (0x2C)
#define NAND_DEVICE_ID                          (0xDA)
#define MAX_VALUE_OF_CHAR                       (256)

/*****************************************************************************/
/*
** Macros which defines the read write size, buffer size and number of transfers
**
*/
#define NAND_DATA_BUFF_SIZE                     (NAND_PAGE_SIZE_IN_BYTES)
#define NAND_ECC_BUFF_SIZE                      ((NAND_PAGE_SIZE_IN_BYTES/NAND_BYTES_PER_TRNFS) \
                                                 * NAND_MAX_ECC_BYTES_PER_TRNFS)
/*****************************************************************************/
/*
** Macros which defines default block, page and num of pages for read/write.
**
*/
#define NAND_DEFAULT_START_PAGE                 (0)
#define NAND_DEFAULT_BLK                        (5)
#define NAND_DEFAULT_NMBR_OF_PAGES              (1)

/*****************************************************************************/
/*
** Macros which defines the data integrity status.
**
*/
#define NAND_DATA_INTEGRITY_PASS                (0)
#define NAND_DATA_INTEGRITY_FAIL                (1)

/*****************************************************************************/
/*
** Macros which defines the NAND timing info.
**
*/
#define NAND_CSWROFFTIME                        (30)
#define NAND_CSRDOFFTIME                        (31)
#define NAND_CSONTIME                           (0)

#define NAND_ADVONTIME                          (0)
#define NAND_ADVAADMUXONTIME                    (0)
#define NAND_ADVRDOFFTIME                       (31)
#define NAND_ADVWROFFTIME                       (31)
#define NAND_ADVAADMUXRDOFFTIME                 (0)
#define NAND_ADVAADMUXWROFFTIME                 (0)

#define NAND_WEOFFTIME                          (31)
#define NAND_WEONTIME                           (3)
#define NAND_OEAADMUXOFFTIME                    (31)
#define NAND_OEOFFTIME                          (31)
#define NAND_OEAADMUXONTIME                     (3)
#define NAND_OEONTIME                           (1)

#define NAND_RDCYCLETIME                        (31)
#define NAND_WRCYCLETIME                        (31)
#define NAND_RDACCESSTIME                       (28)
#define NAND_PAGEBURSTACCESSTIME                (0)

#define NAND_BUSTURNAROUND                      (0)
#define NAND_CYCLE2CYCLEDIFFCSEN                (0)
#define NAND_CYCLE2CYCLESAMECSEN                (1)
#define NAND_CYCLE2CYCLEDELAY                   (0)
#define NAND_WRDATAONADMUXBUS                   (15)
#define NAND_WRACCESSTIME                       (22)

/*****************************************************************************/
/*
** Macros which defines the chip select base address and cs region size.
**
*/
#define NAND_CS0_BASEADDR                       (0x10000000)
#define NAND_CS0_REGIONSIZE                     (GPMC_CS_SIZE_256MB)


/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

#if defined(__IAR_SYSTEMS_ICC__)

#pragma data_alignment=SOC_CACHELINE_SIZE_MAX
volatile unsigned char txData[NAND_DATA_BUFF_SIZE];
#pragma data_alignment=SOC_CACHELINE_SIZE_MAX
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#elif defined(__TMS470__) || defined(_TMS320C6X)

#pragma DATA_ALIGN(txData, SOC_CACHELINE_SIZE_MAX);
volatile unsigned char txData[NAND_DATA_BUFF_SIZE];
#pragma DATA_ALIGN(rxData, SOC_CACHELINE_SIZE_MAX);
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#else

volatile unsigned char  __attribute__ ((aligned (SOC_CACHELINE_SIZE_MAX)))
                        txData[NAND_DATA_BUFF_SIZE];
volatile unsigned char  __attribute__ ((aligned (SOC_CACHELINE_SIZE_MAX)))
                        rxData[NAND_DATA_BUFF_SIZE];

#endif

unsigned char eccData[NAND_ECC_BUFF_SIZE];

/* Page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif


/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/


/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  This function prints the device ID info of NAND.\n                  *
*                                                                             *
* \param  nandInfo        Pointer to structure which conatins controller and  *
*                         device information.                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDDeviceIdInfoPrint(NandInfo_t *nandInfo)
{
    ConsoleUtilsPrintf(" ****************** NAND DEVICE INFO ************");
    ConsoleUtilsPrintf("****** \r\n");
    ConsoleUtilsPrintf("    MANUFACTURER ID    : ");
    ConsoleUtilsPrintf("%x", nandInfo->manId);
    ConsoleUtilsPrintf("\r\n");
    ConsoleUtilsPrintf("    DEVICE ID          : ");
    ConsoleUtilsPrintf("%x", nandInfo->devId);
    ConsoleUtilsPrintf("\r\n");
    ConsoleUtilsPrintf("    PAGESIZE           : ");
    ConsoleUtilsPrintf("%d", nandInfo->pageSize);
    ConsoleUtilsPrintf(" Bytes\r\n");
    ConsoleUtilsPrintf("    BLK SIZE           : ");
    ConsoleUtilsPrintf("%d", nandInfo->blkSize);
    ConsoleUtilsPrintf(" Bytes\r\n");
    ConsoleUtilsPrintf("    PAGES PER BLOCK    : ");
    ConsoleUtilsPrintf("%u", nandInfo->pagesPerBlk);
    ConsoleUtilsPrintf("\r\n");
    ConsoleUtilsPrintf(" ***********************************************");
    ConsoleUtilsPrintf("******* \r\n");
}

/******************************************************************************
*                                                                             *
* \brief  This function initializes the read, write and ecc buffers.\n        *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none                                                                *
*                                                                             *
******************************************************************************/
static void NANDBuffersInit()
{
    unsigned int byteCnt;

    txData[0]  = 'S';
    txData[1]  = 'T';
    txData[2]  = 'A';
    txData[3]  = 'R';
    txData[4]  = 'T';
    txData[5]  = 'E';
    txData[6]  = 'R';
    txData[7]  = 'W';
    txData[8]  = 'A';
    txData[9]  = 'R';
    txData[10] = 'E';

    for(byteCnt = 11; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        txData[byteCnt]= ((byteCnt) % MAX_VALUE_OF_CHAR);
    }

    for(byteCnt = 0; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        rxData[byteCnt]= 0x00;
    }

    for(byteCnt = 0; byteCnt < NAND_ECC_BUFF_SIZE; byteCnt++)
    {
        eccData[byteCnt] = 0;
    }
}

/******************************************************************************
*                                                                             *
* \brief  This function checks for the data integrity of tx and rx buffers.\n *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return Data inegrity status i.e.                                           *
*                                                                             *
*        NAND_DATA_INTEGRITY_PASS -- On pass                                  *
*        NAND_DATA_INTEGRITY_FAIL -- On fail                                  *
*                                                                             *
******************************************************************************/
static unsigned int NANDDataIntegrityCheck()
{
    unsigned int status = NAND_DATA_INTEGRITY_PASS;
    unsigned int byteCnt;

    for(byteCnt = 0; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        if(rxData[byteCnt] != txData[byteCnt])
        {
            status = NAND_DATA_INTEGRITY_FAIL;
            break;
        }
    }
    return (status);
}

/******************************************************************************
*                                                                             *
*                                                                             *
* \brief  Function to initalize the GPMC NAND timing and base addr info.      *
*                                                                             *
* \param  nandTimimgInfo : Pointer to structure containing                    *
*                          NAND timing info.                                  *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDTimingInfoInit(void *TimingInfo)
{

    GPMCNANDTimingInfo_t *nandTimingInfo;
    nandTimingInfo = (GPMCNANDTimingInfo_t * )TimingInfo;

    nandTimingInfo->CSWrOffTime               = NAND_CSWROFFTIME;
    nandTimingInfo->CSRdOffTime               = NAND_CSRDOFFTIME;
    nandTimingInfo->CSExtDelayFlag            = GPMC_CS_EXTRA_NODELAY;
    nandTimingInfo->CSOnTime                  = NAND_CSONTIME;

    nandTimingInfo->ADVAADMuxWrOffTime        = NAND_ADVAADMUXWROFFTIME;
    nandTimingInfo->ADVAADMuxRdOffTime        = NAND_ADVAADMUXRDOFFTIME;
    nandTimingInfo->ADVWrOffTime              = NAND_ADVWROFFTIME;
    nandTimingInfo->ADVRdOffTime              = NAND_ADVRDOFFTIME;
    nandTimingInfo->ADVExtDelayFlag           = GPMC_ADV_EXTRA_NODELAY;
    nandTimingInfo->ADVAADMuxOnTime           = NAND_ADVAADMUXONTIME;
    nandTimingInfo->ADVOnTime                 = NAND_ADVONTIME;

    nandTimingInfo->WEOffTime                 = NAND_WEOFFTIME;
    nandTimingInfo->WEExtDelayFlag            = GPMC_WE_EXTRA_NODELAY;
    nandTimingInfo->WEOnTime                  = NAND_WEONTIME;
    nandTimingInfo->OEAADMuxOffTime           = NAND_OEAADMUXOFFTIME;
    nandTimingInfo->OEOffTime                 = NAND_OEOFFTIME;
    nandTimingInfo->OEExtDelayFlag            = GPMC_OE_EXTRA_NODELAY;
    nandTimingInfo->OEAADMuxOnTime            = NAND_OEAADMUXONTIME;
    nandTimingInfo->OEOnTime                  = NAND_OEONTIME;

    nandTimingInfo->rdCycleTime               = NAND_RDCYCLETIME;
    nandTimingInfo->wrCycleTime               = NAND_WRCYCLETIME;
    nandTimingInfo->rdAccessTime              = NAND_RDACCESSTIME;
    nandTimingInfo->pageBurstAccessTime       = NAND_PAGEBURSTACCESSTIME;

    nandTimingInfo->cycle2CycleDelay          = NAND_CYCLE2CYCLEDELAY;
    nandTimingInfo->cycle2CycleDelaySameCSCfg = NAND_CYCLE2CYCLESAMECSEN;
    nandTimingInfo->cycle2CycleDelayDiffCSCfg = NAND_CYCLE2CYCLEDIFFCSEN;
    nandTimingInfo->busTAtime                 = NAND_BUSTURNAROUND;
}


/******************************************************************************
*                                                                             *
* \brief  Function to initialize the device and controller info.              *
*                                                                             *
* \param  nandInfo      : Pointer to structure containing controller and      *
*                         device information.                                 *
*                                                                             *
* \param  csNum         : Chip select where device is interfaced.             *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDInfoInit(NandInfo_t *nandInfo, unsigned int cs)
{
    NandCtrlInfo_t *hNandCtrlInfo = nandInfo->hNandCtrlInfo;
    NandDmaInfo_t  *hNandDmaInfo  = nandInfo->hNandDmaInfo;
    NandEccInfo_t  *hNandEccInfo  = nandInfo->hNandEccInfo;

    /* Init the NAND Device Info */
    nandInfo->opMode                        = NAND_DATA_XFER_MODE;
    nandInfo->eccType                       = NAND_ECC_ALGO_BCH_8BIT;

    nandInfo->chipSelectCnt                 = 1;
    nandInfo->dieCnt                        = 1;
    nandInfo->chipSelects[0]                = cs;
    nandInfo->busWidth                      = NAND_BUSWIDTH;
    nandInfo->pageSize                      = NAND_PAGE_SIZE_IN_BYTES;
    nandInfo->blkSize                       = NAND_BLOCK_SIZE_IN_BYTES;
    nandInfo->manId                         = NAND_MANUFATURER_MICRON_ID;
    nandInfo->devId                         = NAND_DEVICE_ID;
    nandInfo->dataRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_DATA(GPMC_CHIP_SELECT_0));
    nandInfo->addrRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_ADDRESS(GPMC_CHIP_SELECT_0));
    nandInfo->cmdRegAddr                    = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_COMMAND(GPMC_CHIP_SELECT_0));
    /* Init the NAND Controller Info struct */
    hNandCtrlInfo->CtrlInit                 = GPMCNANDInit;
    hNandCtrlInfo->WaitPinStatusGet         = GPMCNANDWaitPinStatusGet;
    hNandCtrlInfo->WriteBufReady            = GPMCNANDWriteBufReady;
    hNandCtrlInfo->currChipSelect           = cs;
    hNandCtrlInfo->baseAddr                 = SOC_GPMC_0_REGS;
    hNandCtrlInfo->eccSupported             = (NAND_ECC_ALGO_HAMMING_1BIT |
                                          NAND_ECC_ALGO_BCH_4BIT |
                                          NAND_ECC_ALGO_BCH_8BIT |
                                          NAND_ECC_ALGO_BCH_16BIT );

    hNandCtrlInfo->waitPin                  = GPMC_WAIT_PIN0;
    hNandCtrlInfo->waitPinPol               = GPMC_WAIT_PIN_POLARITY_LOW;
    hNandCtrlInfo->wpPinPol                 = GPMC_WP_PIN_LEVEL_HIGH;
    hNandCtrlInfo->chipSelectBaseAddr[0]    = NAND_CS0_BASEADDR;
    hNandCtrlInfo->chipSelectRegionSize[0]  = NAND_CS0_REGIONSIZE;
    NANDTimingInfoInit(hNandCtrlInfo->hNandTimingInfo);


    /* Init the NAND Ecc Info */
    hNandEccInfo->baseAddr                  = SOC_ELM_0_REGS;
    hNandEccInfo->ECCInit                   = GPMCNANDECCInit;
    hNandEccInfo->ECCEnable                 = GPMCNANDECCEnable;
    hNandEccInfo->ECCDisable                = GPMCNANDECCDisable;
    hNandEccInfo->ECCWriteSet               = GPMCNANDECCWriteSet;
    hNandEccInfo->ECCReadSet                = GPMCNANDECCReadSet;
    hNandEccInfo->ECCCalculate              = GPMCNANDECCCalculate;
    hNandEccInfo->ECCCheckAndCorrect        = GPMCNANDECCCheckAndCorrect;

    /* Init the NAND DMA info */
    hNandDmaInfo->DMAXfer                   = GPMCNANDDMAXfer;
    hNandDmaInfo->DMAInit                   = GPMCNANDEdmaInit;
    hNandDmaInfo->DMAXferSetup              = GPMCNANDXferSetup;
    hNandDmaInfo->DMAXferStatusGet          = GPMCNANDXferStatusGet;
}


/******************************************************************************
*                                                                             *
* \brief  Function to setup MMU.                                              *
*         The function maps four regions                                      *
*         1. DDR                                                              *
*         2. OCMC                                                             *
*         3. SRAM                                                             *
*         4. Device memory                                                    *
*                                                                             *
* \param  none                                                                *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };


    /*
    ** Define SRAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionSram = {
                         MMU_PGTYPE_SECTION, START_ADDR_SRAM, NUM_SECTIONS_SRAM,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionSram);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}


/******************************************************************************
**                       GLOBAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  Main Function.\n                                                    *
*                                                                             *
******************************************************************************/
int main(void)
{
    int blkNum;
    int pageNum;
    int numOfPages;
    unsigned int retVal;
    unsigned int eraseBlkFlg;

    /* NAND structure allocations for this application */
    NandInfo_t              nandInfo;
    NandCtrlInfo_t          nandCtrlInfo;
    NandEccInfo_t           nandEccInfo;
    NandDmaInfo_t           nandDmaInfo;
    GPMCNANDTimingInfo_t    nandTimingInfo;

    /* Configure and enable the MMU. */
    MMUConfigAndEnable();

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    ConsoleUtilsPrintf("\r\n ************* StarterWare NAND Application ****");
    ConsoleUtilsPrintf("********\r\n\r\n");

    /* Pin mux and clock setting */
    NANDPinMuxSetup();
    GPMCClkConfig();
    EDMAModuleClkConfig();
             
    /* Initialize the nandInfo struct */
    nandCtrlInfo.hNandTimingInfo = (void *) &nandTimingInfo;    
    nandInfo.hNandCtrlInfo = &nandCtrlInfo;
    nandInfo.hNandEccInfo = &nandEccInfo;
    nandInfo.hNandDmaInfo = &nandDmaInfo;
    NANDInfoInit(&nandInfo, NAND_CHIP_SELECT);

    /* Open the NAND device */
    retVal = NANDOpen(&nandInfo);
    if (retVal & NAND_STATUS_FAILED)
    {
        ConsoleUtilsPrintf("\r\n*** ERROR : NAND Open Failed... ");
        while(1);    
    }
    else if (retVal & NAND_STATUS_WAITTIMEOUT)
    {
        ConsoleUtilsPrintf("\r\n*** ERROR : Device Is Not Ready...!!!\r\n");
        while(1);
    }
    else if (retVal & NAND_STATUS_NOT_FOUND)
    {
        ConsoleUtilsPrintf("\r\n*** ERROR : DEVICE MAY NOT BE ACCESSABLE OR");
        ConsoleUtilsPrintf(" NOT PRESENT.\r\n");
        while(1);
    }
    else if(nandInfo.devId != NAND_DEVICE_ID)
    {
        /* Check if detected ID matches supplied ID */
        ConsoleUtilsPrintf("\r\n*** ERROR : INVALID DEVICE ID.");
        while(1);
    }
    else
    {
        /* Print The Device ID info */
        NANDDeviceIdInfoPrint(&nandInfo);
    }
    
#ifdef NAND_DATAINTEGRITY_TEST_WITH_FIXED_ADDR
    /* Do read/write for predefined address */
    pageNum = NAND_DEFAULT_START_PAGE;
    blkNum = NAND_DEFAULT_BLK;
    numOfPages = NAND_DEFAULT_NMBR_OF_PAGES;   
#else
    /* Take the read/write address from the user */
    ConsoleUtilsPrintf("\r\n Please Enter The Block Number(0 - ");
    ConsoleUtilsPrintf("%u", (NAND_NUMOF_BLK - 1));
    ConsoleUtilsPrintf(")\r\n");
    ConsoleUtilsScanf("%d", &blkNum);
    ConsoleUtilsPrintf("\r\n Please Enter The Page Start Page Number(0 - ");
    ConsoleUtilsPrintf("%u", (nandInfo.pagesPerBlk - 1));
    ConsoleUtilsPrintf(")\r\n");
    ConsoleUtilsScanf("%d", &pageNum);
    ConsoleUtilsPrintf("\r\n Please Enter The Number Of Pages To Write\r\n");
    ConsoleUtilsScanf("%d", &numOfPages);
#endif
    eraseBlkFlg = 1;
    
    if( (pageNum < 0 ) || (pageNum > (nandInfo.pagesPerBlk - 1))
        || (blkNum < 0 || blkNum  > (NAND_NUMOF_BLK - 1)) || (numOfPages <= 0) )
    {
        ConsoleUtilsPrintf("\r\n *** ERROR : Wrong Input(s) Entered...!!!\r\n");
        while(1);
    }
    else if( ( blkNum * (nandInfo.pagesPerBlk )
              + pageNum + numOfPages ) >
              ( NAND_NUMOF_BLK * nandInfo.pagesPerBlk))
    {
        ConsoleUtilsPrintf("\r\n *** ERROR : Requsted Page(s) For Read/Write ");
        ConsoleUtilsPrintf("Does Not Exist...!!!\r\n");
        while(1);
    }    
   
    while( numOfPages > 0 )
    {
        if( eraseBlkFlg )
        {
            retVal = NANDBadBlockCheck(&nandInfo, blkNum);
            if(retVal == NAND_BLOCK_BAD)
            {
                ConsoleUtilsPrintf("\r\n Block Is Bad, Can't Continue ...!!! ");
                while(1);
            }
            if(retVal == NAND_BLOCK_SPARE_AREA_READ_FAILED)
            {
                ConsoleUtilsPrintf("\r\n Spare Area Read Failed While ");
                ConsoleUtilsPrintf("Checking ");
                ConsoleUtilsPrintf(" For Bad Block ");
                while(1);
            }

            ConsoleUtilsPrintf("\r\n Erasing The Block ");
            ConsoleUtilsPrintf("%d", blkNum);
            ConsoleUtilsPrintf("                         :");

            retVal = NANDBlockErase(&nandInfo, blkNum);
            if( retVal == NAND_STATUS_PASSED )
            {
                ConsoleUtilsPrintf(" Succeeded.");
            }
            else
            {
                ConsoleUtilsPrintf(" Failed.");
                ConsoleUtilsPrintf("\r\n Marking The Block As Bad.\r\n ");
                ConsoleUtilsPrintf("Read/Write Test");
                ConsoleUtilsPrintf(" Will Be Continued On Next block. ");
                NANDMarkBlockAsBad(&nandInfo, blkNum);
                blkNum++;
                pageNum = 0;
                eraseBlkFlg = 1;
                continue;
            }
            eraseBlkFlg = 0;
        }

        NANDBuffersInit();
        ConsoleUtilsPrintf("\r\n Writing To Page ");
        ConsoleUtilsPrintf("%d", pageNum);
        ConsoleUtilsPrintf(" Of Block ");
        ConsoleUtilsPrintf("%d", blkNum);
        ConsoleUtilsPrintf("                :");

        retVal = NANDPageWrite(&nandInfo, blkNum, pageNum, &txData[0],
                               &eccData[0]);
        if( (retVal & NAND_STATUS_WAITTIMEOUT) )
        {
            ConsoleUtilsPrintf(" Failed.(Device Is Busy).");
            while(1);
        }
        else if( (retVal & NAND_STATUS_DEVWRPROTECT) )
        {
            ConsoleUtilsPrintf(" Failed.(Device Is Write Protected).");
            while(1);
        }
        else if( (retVal & NAND_STATUS_READWRITE_DMA_FAIL) )
        {
            ConsoleUtilsPrintf(" Failed.(EDMA Transfer Failed.).");
            while(1);
        }
        else
        {
            ConsoleUtilsPrintf(" Succeeded.");
        }

        /* As eccData, is filled by NANDPageWrite fun, reinit the same */
        NANDBuffersInit();

        ConsoleUtilsPrintf("\r\n Reading From Page ");
        ConsoleUtilsPrintf("%d", pageNum);
        ConsoleUtilsPrintf(" Of Block ");
        ConsoleUtilsPrintf("%d", blkNum);
        ConsoleUtilsPrintf("              :");
        retVal= NANDPageRead(&nandInfo, blkNum, pageNum, &rxData[0],
                             &eccData[0]);

        if( (retVal & NAND_STATUS_READ_ECC_ERROR_CORRECTED) )
        {
            ConsoleUtilsPrintf(" Succeeded With ECC Errors And Corrected.");
        }
        else if( (retVal & NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR) )
        {
            ConsoleUtilsPrintf(" Failed.(Uncorrectable ECC errors) ");
            while(1);
        }
        else if( (retVal & NAND_STATUS_READWRITE_DMA_FAIL) )
        {
            ConsoleUtilsPrintf(" Failed.(EDMA Transfer Failed.)");
            while(1);
        }
        else
        {
            ConsoleUtilsPrintf(" Succeeded.");
        }

        ConsoleUtilsPrintf("\r\n NAND Data Integrity Test                    :");
        retVal = NANDDataIntegrityCheck();
        if(retVal == NAND_DATA_INTEGRITY_PASS)
        {
            ConsoleUtilsPrintf(" Passed\r\n");
        }
        else
        {
            ConsoleUtilsPrintf(" Failed....!!!\r\n");
        }

        pageNum++;
        numOfPages--;
        if( pageNum == ((nandInfo.pagesPerBlk) ) )
        {
            pageNum = 0;
            eraseBlkFlg = 1;
            blkNum++;
        }
    }

    ConsoleUtilsPrintf("\r\n ******************************************");
    ConsoleUtilsPrintf("************ ");
    while(1);    
}


/******************************************************************************
**                              END OF FILE
*******************************************************************************/
