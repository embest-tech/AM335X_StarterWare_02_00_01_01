/**
 * \file  bl_hsmmcsd.c
 *
 * \brief HSMMCSD support for StarterWare bootloader.  Initialization functions
 *        and a funciton to copy data from card to the given address.
 *
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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
*/

#include "ff.h"
#include "hs_mmcsd.h"
#include "bl.h"
#include "bl_platform.h"
#include "bl_copy.h"
#include "mmcsd_proto.h"
#include "hs_mmcsdlib.h"
#include "uartStdio.h"
#include "string.h"


/******************************************************************************
**                      MACRO DEFINITIONS
*******************************************************************************/

/*****************************************************************************
Defines the size of the buffer that holds the command line.
******************************************************************************/
#define CMD_BUF_SIZE    512


/*****************************************************************************
A macro to make it easy to add result codes to the table.
******************************************************************************/
#define FRESULT_ENTRY(f)        { (f), (#f) }

/*****************************************************************************
A macro that holds the number of result codes.
******************************************************************************/
#define NUM_FRESULT_CODES (sizeof(g_sFresultStrings) / sizeof(tFresultString))

#define HSMMCSD_INT_STATUS_FLAG       (0xFFFFFFFF)

/*****************************************************************************
Defines the size of the buffers that hold the path, or temporary data from
the memory card.  There are two buffers allocated of this size.  The buffer
size must be large enough to hold the longest expected full path name,
including the file name, and a trailing null character.
******************************************************************************/
#define PATH_BUF_SIZE   512
#define DATA_BUF_SIZE   512

/******************************************************************************
**                      TYPE DEFINITIONS
*******************************************************************************/

/* Fat devices registered */
#ifndef fatDevice
typedef struct _fatDevice
{
    /* Pointer to underlying device/controller */
    void *dev;

    /* File system pointer */
    FATFS *fs;
}fatDevice;
#endif
extern fatDevice fat_devices[2];
extern unsigned int entryPoint;

/******************************************************************************
**              GLOBAL VARIABLE DEFINITIONS
******************************************************************************/


/* Global flags for interrupt handling */
volatile unsigned int xferPend = 0;
volatile unsigned int cmdCompFlag = 0;
volatile unsigned int cmdTimeout = 0;
volatile unsigned int errFlag = 0;
volatile unsigned int sdBlkSize = MMCSD_BLK_SIZE;
unsigned int hsmmcsd_dataLen = 0;
volatile unsigned char *hsmmcsd_buffer = NULL;

/*****************************************************************************
The buffer that holds the command line.
******************************************************************************/

volatile unsigned int g_sPState = 0;
volatile unsigned int g_sCState = 0;

/* SD card info structure */
#ifdef __IAR_SYSTEMS_ICC__

#pragma data_alignment=32
mmcsdCardInfo sdCard;
#pragma data_alignment=32
mmcsdCtrlInfo ctrlInfo;
#pragma data_alignment=32
static char g_cTmpBuf[DATA_BUF_SIZE];

#elif defined(__TMS470__)

#pragma DATA_ALIGN(sdCard, 32);
mmcsdCardInfo sdCard;
#pragma DATA_ALIGN(ctrlInfo, 32);
mmcsdCtrlInfo ctrlInfo;
#pragma DATA_ALIGN(g_cTmpBuf, 32);
static char g_cTmpBuf[DATA_BUF_SIZE];

#else

mmcsdCardInfo sdCard __attribute__ ((aligned (32)));
mmcsdCtrlInfo  ctrlInfo __attribute__ ((aligned (32)));

/*****************************************************************************
A temporary data buffer used when manipulating file paths, or reading data
from the memory card.
******************************************************************************/
static char g_cTmpBuf[DATA_BUF_SIZE] __attribute__ ((aligned (32)));

#endif

/*****************************************************************************
Current FAT fs state.
******************************************************************************/
static FATFS g_sFatFs;
static FIL g_sFileObject;

/*****************************************************************************
A structure that holds a mapping between an FRESULT numerical code,
and a string representation.  FRESULT codes are returned from the FatFs
FAT file system driver.

******************************************************************************/
typedef struct
{
    FRESULT fresult;
    char *pcResultStr;
}
tFresultString;

/*****************************************************************************
A table that holds a mapping between the numerical FRESULT code and
it's name as a string.  This is used for looking up error codes for
printing to the console.
******************************************************************************/
tFresultString g_sFresultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_RW_ERROR),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_MKFS_ABORTED)
};

extern void HSMMCSDFsMount(unsigned int driveNum, void *ptr);
extern void HSMMCSDFsProcessCmdLine(void);

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static unsigned int HSMMCSDStatusGet(void);

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/

/**
*\brief This function Check the command status.\n
*
* \param - ctrl - MMCSD controller info.\n
*
* \return command status.\n
*
*/

static unsigned int HSMMCSDCmdStatusGet(mmcsdCtrlInfo *ctrl)
{
    unsigned int status = 0;

     while ((cmdCompFlag == 0) && (cmdTimeout == 0))
    {
        status = HSMMCSDStatusGet();
    }

    if (cmdCompFlag)
    {
        HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_CMDCOMP);
        status = 1;
        cmdCompFlag = 0;
    }

    if (cmdTimeout)
    {
        HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_CMDTIMEOUT);
        status = 0;
        cmdTimeout = 0;
    }

    return status;
}

/**
*\brief This function gets the Xfer status.\n
*
* \param - ctrl - MMCSD controller info.\n
*
* \return Xfer status.\n
*
*/
static unsigned int HSMMCSDXferStatusGet(mmcsdCtrlInfo *ctrl)
{
    volatile unsigned int status = 0;
    volatile unsigned int temp;
    unsigned int i;

    while(1)
    {
        status = HSMMCSDStatusGet();

        if (status & HS_MMCSD_STAT_BUFRDRDY)
        {
            HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_BUFRDRDY);

            if (hsmmcsd_buffer != NULL)
            {
                for (i = 0; i < hsmmcsd_dataLen; i+=4)
                {
                    temp = HWREG(MMCSD_BASE + MMCHS_DATA);
                    hsmmcsd_buffer[i] = *((char*)&temp);
                    hsmmcsd_buffer[i+1] = *((char*)&temp + 1);
                    hsmmcsd_buffer[i+2] = *((char*)&temp + 2);
                    hsmmcsd_buffer[i+3] = *((char*)&temp + 3);
                }
            }
        }

        if (status & HS_MMCSD_STAT_BUFWRRDY)
        {
            HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_BUFRDRDY);

            if (hsmmcsd_buffer != NULL)
            {
                for (i = 0; i < hsmmcsd_dataLen; i+=4)
                {
                    *((char*)&temp) = hsmmcsd_buffer[i];
                    *((char*)&temp + 1) = hsmmcsd_buffer[i+1];
                    *((char*)&temp + 2) = hsmmcsd_buffer[i+2];
                    *((char*)&temp + 3) = hsmmcsd_buffer[i+3];
                    HWREG(MMCSD_BASE + MMCHS_DATA) = temp;
                }
            }
        }

        if (status & HS_MMCSD_STAT_DATATIMEOUT)
        {
            HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_DATATIMEOUT);
            status = 0;
            xferPend = 0;
            break;
        }

        if (status & HS_MMCSD_STAT_TRNFCOMP)
        {
            HSMMCSDIntrStatusClear(ctrlInfo.memBase,
                                   HS_MMCSD_STAT_TRNFCOMP);
            status = 1;
            xferPend = 0;
            break;
        }
    }

    return status;
}

/**
*\brief This function setup the controller and DMA for Xfer.\n
*
* \param - ctrl    - MMCSD controller info.\n
*
* \param - rwFlag  - Read/Write flag.\n
*
* \param - blkSize - Block Size.\n
*
* \param - nBlks   - Number of Blocks.\n
*
* \return none.\n
*
*/

static void HSMMCSDXferSetup(mmcsdCtrlInfo *ctrl, unsigned char rwFlag, void *ptr,
                             unsigned int blkSize, unsigned int nBlks)
{
    HSMMCSDIntrStatusClear(ctrl->memBase, HS_MMCSD_INTR_TRNFCOMP);

    if (rwFlag == 1)
    {
        HSMMCSDIntrStatusClear(ctrl->memBase, HS_MMCSD_INTR_BUFRDRDY);
        HSMMCSDIntrStatusEnable(ctrl->memBase, HS_MMCSD_INTR_BUFRDRDY);
        HSMMCSDIntrStatusDisable(ctrl->memBase, HS_MMCSD_INTR_BUFWRRDY);
    }
    else
    {
        HSMMCSDIntrStatusClear(ctrl->memBase, HS_MMCSD_INTR_BUFWRRDY);
        HSMMCSDIntrStatusEnable(ctrl->memBase, HS_MMCSD_INTR_BUFWRRDY);
        HSMMCSDIntrStatusDisable(ctrl->memBase, HS_MMCSD_INTR_BUFRDRDY);
    }

    HSMMCSDBlkLenSet(ctrl->memBase, blkSize);
    hsmmcsd_dataLen = (nBlks * blkSize);
    hsmmcsd_buffer = ptr;
    xferPend = 1;
}

/**
*\brief This function Check the command status.\n
*
* \param - ctrl - MMCSD controller info.\n
*
* \return command status.\n
*
*/
static unsigned int HSMMCSDStatusGet(void)
{
    volatile unsigned int status = 0;

    status = HSMMCSDIntrStatusGet(ctrlInfo.memBase, HSMMCSD_INT_STATUS_FLAG);

    if (status & HS_MMCSD_STAT_CMDCOMP)
    {
        cmdCompFlag = 1;
    }

    if (status & HS_MMCSD_STAT_ERR)
    {
        errFlag = status & 0xFFFF0000;

        if (status & HS_MMCSD_STAT_CMDTIMEOUT)
        {
            cmdTimeout = 1;
        }
    }

    return status;
}

/**
*\brief This function initializes the controller struct.\n
*
* \param - none.\n
*
* \return - none.\n
*
*/
static void HSMMCSDControllerSetup(void)
{
    ctrlInfo.memBase = MMCSD_BASE;
    ctrlInfo.ctrlInit = HSMMCSDControllerInit;
    ctrlInfo.xferSetup = HSMMCSDXferSetup;
    ctrlInfo.cmdStatusGet = HSMMCSDCmdStatusGet;
    ctrlInfo.xferStatusGet = HSMMCSDXferStatusGet;
    ctrlInfo.cardPresent = HSMMCSDCardPresent;
    ctrlInfo.cmdSend = HSMMCSDCmdSend;
    ctrlInfo.busWidthConfig = HSMMCSDBusWidthConfig;
    ctrlInfo.busFreqConfig = HSMMCSDBusFreqConfig;
    ctrlInfo.intrMask = (HS_MMCSD_INTR_CMDCOMP | HS_MMCSD_INTR_CMDTIMEOUT |
                            HS_MMCSD_INTR_DATATIMEOUT | HS_MMCSD_INTR_TRNFCOMP);
    ctrlInfo.intrEnable = HSMMCSDIntEnable;
    ctrlInfo.busWidth = (SD_BUS_WIDTH_1BIT | SD_BUS_WIDTH_4BIT);
    ctrlInfo.highspeed = 1;
    ctrlInfo.ocr = MMCSD_OCR;
    ctrlInfo.card = &sdCard;
    ctrlInfo.ipClk = MMCSD_IN_FREQ;
    ctrlInfo.opClk = MMCSD_INIT_FREQ;
    sdCard.ctrl = &ctrlInfo;

    cmdCompFlag = 0;
    cmdTimeout = 0;
}

/**
*\brief This function mounts the devices.\n
*
* \param - driveNum - Drive number.\n
*
* \param - prt      - Device pointer.\n
*
* \return none.\n
*
*/
void HSMMCSDFsMount(unsigned int driveNum, void *ptr)
{
    f_mount(driveNum, &g_sFatFs);
    fat_devices[0].dev = ptr;
    fat_devices[0].fs = &g_sFatFs;
}

/**
*\brief This function copies the application image to the DDR by reading the
*       header info.\n
*
* \param - none.\n
*
* \return none.\n
*
*/
unsigned int HSMMCSDImageCopy(void)
{
    FRESULT fresult;
    unsigned short usBytesRead = 0;
    ti_header imageHdr;
    unsigned char *destAddr;
    char *fname = "/app";

    /*
    ** Open the file for reading.
    */
    fresult = f_open(&g_sFileObject, fname, FA_READ);

    /*
    ** If there was some problem opening the file, then return an error.
    */
    if(fresult != FR_OK)
    {
        UARTPuts("\r\n Unable to open application file\r\n", -1);
        return 0;
    }
    else
    {
        UARTPuts("Copying application image from MMC/SD card to RAM\r\n", -1);
        fresult = f_read(&g_sFileObject, (unsigned char *)&imageHdr, 8,
                         &usBytesRead);
        if(fresult != FR_OK)
        {
            UARTPuts("\r\n Error reading application file\r\n", -1);
            return 0;
        }

        if(usBytesRead != 8)
        {
            return 0;
        }
        destAddr = (unsigned char*)imageHdr.load_addr;
        entryPoint = imageHdr.load_addr;
    }

    /*
    ** Enter a loop to repeatedly read data from the file and display it, until
    ** the end of the file is reached.
    */
    do
    {
        /*
        ** Read a block of data from the file.  Read as much as can fit in the
        ** temporary buffer, including a space for the trailing null.
        */
        fresult = f_read(&g_sFileObject, g_cTmpBuf, sizeof(g_cTmpBuf) - 1,
                         &usBytesRead);

        /*
        ** If there was an error reading, then print a newline and return the
        ** error to the user.
        */
        if(fresult != FR_OK)
        {
            UARTPuts("\r\n Error reading application file\r\n", -1);
            return 0;
        }

        if(usBytesRead >= sizeof(g_cTmpBuf))
        {
            return 0;
        }

        /*
        ** Null terminate the last block that was read to make it a null
        ** terminated string that can be used with printf.
        */
        g_cTmpBuf[usBytesRead] = 0;

        /*
        ** Read the last chunk of the file that was received.
        */
        memcpy(destAddr, g_cTmpBuf, (sizeof(g_cTmpBuf) - 1));
        destAddr += (sizeof(g_cTmpBuf) - 1);
        /*
        ** Continue reading until less than the full number of bytes are read.
        ** That means the end of the buffer was reached.
        */
    }
    while(usBytesRead == sizeof(g_cTmpBuf) - 1);

    /*
    ** Close the file.
    */
    fresult = f_close(&g_sFileObject);

    /*
    ** Return success.
    */
    return 1;
}

/**
*\brief This function initializes the MMCSD controller and mounts the device.
*
* \param - none.\n
*
* \return none.\n
*
*/
void HSMMCSDInit(void)
{

    /* Basic controller initializations */
    HSMMCSDControllerSetup();

    /* First check, if card is insterted */
    while(1)
    {

        if (MMCSDCardPresent(&ctrlInfo) == 0)
        {
            UARTPuts("MMC/SD Card not found\n\r", -1);
        }
        else
        {
            break;
        }
    }

    /* Initialize the MMCSD controller */
    MMCSDCtrlInit(&ctrlInfo);

    MMCSDIntEnable(&ctrlInfo);

    HSMMCSDFsMount(0, &sdCard);
}
