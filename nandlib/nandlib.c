/**
 *  \file   nandlib.c
 *
 *  \brief  This file contains the NAND prtocol abstraction layer 
 *          macro definitions and function definitions.
 *
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


#include "nandlib.h"
#include "hw_types.h"

/*******************************************************************************
*                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/*****************************************************************************/
/*
** Macros which defines the NAND commands.
*/
#define NAND_CMD_READ                           (0x00u)
#define NAND_CMD_READ_CYC2                      (0x30u)
#define NAND_CMD_READID                         (0x90u)
#define NAND_CMD_RESET                          (0xFFu)
#define NAND_CMD_PROGRAM                        (0x80u)
#define NAND_CMD_PROGRAM_CYC2                   (0x10u)
#define NAND_CMD_ERASE                          (0x60u)
#define NAND_CMD_ERASE_CYC2                     (0xD0u)
#define NAND_CMD_READ_STATUS                    (0x70u)
#define NAND_CMD_READ_RANDOM                    (0x05u)
#define NAND_CMD_READ_RANDOM_CYC2               (0xE0u)

/*****************************************************************************/
/*
** NAND RB wait timeout.
*/
#define NAND_RBWAIT_TIMEOUT                     (0xFFFFu)

/*****************************************************************************/
/*
** NAND command status mask  defintions.
*/
#define NAND_CMD_STATUS_PASSFAIL_MASK          (0x01)
#define NAND_CMD_STATUS_DEVREADY_MASK          (0x20)
#define NAND_CMD_STATUS_WRPROTECT_MASK         (0x80)

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/**
* \brief  Delay function for NAND device interface. \n
*
*         This function provides a delay functionality for the NAND device
*         interface module. The parameter specified as delay is not related
*         to any time. It is just a count to execute a dummy loop.\n
*
* \param  delay         : Number of cycles in the delay loop.\n
*
* \return none.\n
*/
static void NANDDelay(volatile unsigned int delay)
{
    while (delay-- > 0u);
}

/**
* \brief  Function to write command to the NAND command register.\n
*
* \param  cmdRegAddr    : Command register address.\n
*
* \param  cmd           : Command to write.\n
*
* \return none.\n
*/
static void NANDCommandWrite(unsigned int cmdRegAddr, unsigned int cmd)
{
    (*(volatile unsigned char*)(cmdRegAddr)) = cmd;
}

/**
* \brief  Function to write the address to the NAND address register.\n
*
* \param  addrRegAddr   : Address register address.\n
*
*         addr          : Address to write.\n
*
* \return none.\n
*/
static void NANDAddressWrite(unsigned int addrRegAddr, unsigned int addr)
{
    (*(volatile unsigned char*)(addrRegAddr)) = (unsigned char)addr;
}

/**
* \brief  Function to read the data from NAND data register.\n
*
* \param  dataRegAddr   : data register address.\n
*
* \return Data from nand data register.\n
*/
static unsigned char NANDDataReadByte(unsigned int dataRegAddr)
{
    unsigned char dataByte;

    dataByte = (unsigned char)(*(volatile unsigned char*)(dataRegAddr));

    return(dataByte);
}

/**
* \brief  Function to read the NAND previous command status.\n
*
*         This function retrives the status of previous command.\n
*
* \param  nandInfo   :  Pointer to structure which contains  
*                       device information.\n
*
* \return NAND status containing below info.\n
*
*         NAND_STATUS_PASSED         : If the previous command is passed.\n
*         NAND_STATUS_FAILED         : If the previous command is failed.\n
*         NAND_STATUS_DEVBUSY        : If the device is busy.\n
*         NAND_STATUS_DEVWRPROTECT   : If the device is write protect.\n
*         NAND_STATUS_WAITTIMEOUT    : If the RB pin inidcating device 
*                                      busy status.\n
*
*/
static NandStatus_t NANDDeviceStatusGet(NandInfo_t *nandInfo)
{
    unsigned char nandStatus;
    NandStatus_t retVal = NAND_STATUS_PASSED;

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_READ_STATUS);
    NANDDelay(10);

    nandStatus = NANDDataReadByte(nandInfo->dataRegAddr);
    if(nandStatus & NAND_CMD_STATUS_PASSFAIL_MASK)
    {
        retVal |= NAND_STATUS_FAILED;
    }
    if( !(nandStatus & NAND_CMD_STATUS_DEVREADY_MASK) )
    {
        retVal |= NAND_STATUS_DEVBUSY;
    }
    if( !(nandStatus & NAND_CMD_STATUS_WRPROTECT_MASK) )
    {
        retVal |= NAND_STATUS_DEVWRPROTECT;
    }

    return retVal;
}

/**
* \brief  Function to write data to the NAND for specified number of bytes.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  txData        :  Pointer to the array containing the data to write.\n
*
* \param  size          :  Transfer count.\n
*
* \return none.\n
*/
static void NANDDataWrite(NandInfo_t *nandInfo, volatile unsigned char *txData,
                   unsigned int size)
{
    unsigned short data;
    unsigned int bufStatus;
    
    if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
    {
        unsigned short *ptrData = (unsigned short *)txData;
        while(size)
        {
            data = *ptrData;
           (*(volatile unsigned short*)(nandInfo->dataRegAddr)) =
                                        data;
            ptrData++;
            size   -= 2;
        }
    }
    else
    {
        while(size)
        {
            do
            {
                /* Check the status of the buffer */
                bufStatus=((nandInfo->hNandCtrlInfo->WriteBufReady)(nandInfo));
            }
            while(!bufStatus);
            (*(volatile unsigned char*)(nandInfo->dataRegAddr)) =
                                       *txData;
            txData++;
            size--;
        }
    }
}

/**
* \brief  Function to read the data from NAND for specified number of bytes.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                          device information.\n
*
* \param  rxData        :  Pointer to the array where read data has to place.\n
*
* \param  size          :  Transfer count.\n
*
* \return none.\n
*/
static void NANDDataRead(NandInfo_t *nandInfo, volatile unsigned char *rxData,
                  unsigned int size)
{
    unsigned short data;
    unsigned short *ptrData;

    if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
    {
        ptrData = (unsigned short *)rxData;
        while(size)
        {
            data = (*(volatile unsigned short*)
                     (nandInfo->dataRegAddr));
            *ptrData   = data;
            size   -= 2;
            ptrData++;
        }
    }
    else
    {
        while(size)
        {
            *rxData = (*(volatile unsigned char*)
                        (nandInfo->dataRegAddr));
            rxData++;
            size--;
        }
    }
}

/**
* \brief  Function to start the page write command sequence.\n
*
*       This function forms the address, based on  the block, page and column.
*       Then the command for Page write is sent along with the address.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  blkNum        : Blk Number.\n
*
* \param  pageNum       : Page Number.\n
*
* \param  columnAddr    : Column Addr.\n
*
* \return none.\n
**/
static void NANDPageWriteCmdStart(NandInfo_t *nandInfo, unsigned int blkNum,
                           unsigned int pageNum, unsigned int columnAddr)
{
    unsigned int pageAddr;
    unsigned int count;
    unsigned int blkSize = nandInfo->blkSize;
    unsigned int pageSize = nandInfo->pageSize;

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_PROGRAM);
    /* Write 2 bytes of column addr */
    NANDAddressWrite(nandInfo->addrRegAddr,
                     (unsigned char)(columnAddr & 0xFF));
    NANDAddressWrite(nandInfo->addrRegAddr,
                     (unsigned char)((columnAddr >> 8) & 0xFF));

    pageAddr = ((blkNum * (blkSize/pageSize)) + pageNum);

    /* Write the row address. */
    for (count = 0; count < 3; count++)
    {
        NANDAddressWrite(nandInfo->addrRegAddr, (unsigned char)pageAddr);
        pageAddr = pageAddr >> 8u;
    }
}

/**
*\brief  Function to end the page write sequence.\n
*
*       This function forms the address, based on  the block, page and column.
*       Then the command for Page write is sent along with the address.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \return Write command status.\n
*
*         NAND_STATUS_PASSED        : If the write command is passed.\n
*         NAND_STATUS_FAILED        : If the write command is failed.\n
*         NAND_STATUS_DEVBUSY       : If the device is busy.\n
*         NAND_STATUS_DEVWRPROTECT  : If the device is write protect.\n
*         NAND_STATUS_WAITTIMEOUT   : If the RB pin inidcating device 
*                                     busy status.\n
*
*/
static NandStatus_t NANDPageWriteCmdEnd(NandInfo_t *nandInfo)
{
    NandStatus_t retVal;

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_PROGRAM_CYC2);
    retVal = NANDWaitUntilReady(nandInfo);
    if( (retVal & NAND_STATUS_WAITTIMEOUT) )
    {
        retVal = NANDDeviceStatusGet(nandInfo);
    }

    return (retVal);
}

/**
*\brief  Function to start the page read command sequence.\n
*
*       This function forms the address, based on  the block, page and column.
*       Then the command for Page read is sent along with the address.\n
*
* \param blkNum         : Blk Number.\n
*
* \param pageNum        : Page Number.\n
*
* \param columnAddr     : Column Addr.\n
*
* \return NAND device status.\n
*
*         NAND_STATUS_PASSED      : If device is ready.\n
*         NAND_STATUS_WAITTIMEOUT : If device is not ready.\n
*
**/
static NandStatus_t NANDPageReadCmdSend(NandInfo_t *nandInfo, unsigned int blkNum,
                                 unsigned int pageNum, unsigned int columnAddr)
{
    NandStatus_t retVal;
    unsigned int count;
    unsigned int pageAddr;
    unsigned int blkSize = nandInfo->blkSize;
    unsigned int pageSize = nandInfo->pageSize;

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_READ);
    /* Write 2 bytes of column addr */
    NANDAddressWrite(nandInfo->addrRegAddr,
                     (unsigned char)(columnAddr & 0xFF));
    NANDAddressWrite(nandInfo->addrRegAddr,
                     (unsigned char)((columnAddr >> 8) & 0xFF));

    pageAddr = ((blkNum * (blkSize/pageSize)) + pageNum);
    /* Write the row address. */
    for (count = 0; count < 3; count++)
    {
        NANDAddressWrite(nandInfo->addrRegAddr,
                         (unsigned char)pageAddr);
        pageAddr = pageAddr >> 8u;
    }

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_READ_CYC2);
    retVal = NANDWaitUntilReady(nandInfo);

    return retVal;
}


/******************************************************************************
**                       GLOBAL FUNCTION DEFINITIONS
*******************************************************************************/

/**
* \brief  Function to Initialize the NAND and associated memory controller.\n
*
*       This function calls the registered controller init function.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \return
*        NAND_STATUS_PASSED          : On success.\n
*        NAND_STATUS_FAILED          : On failure.\n
*        NAND_STATUS_NOT_FOUND       : On NAND not found in the system.\n
*        NAND_STATUS_WAITTIMEOUT     : On NAND response timed out.\n
*
**/
NandStatus_t NANDOpen(NandInfo_t *nandInfo)
{
    NandStatus_t retVal;

    /* Initialize the controller */
    retVal = (nandInfo->hNandCtrlInfo->CtrlInit)(nandInfo);
    if(retVal & NAND_STATUS_FAILED)
    {
        return retVal;
    }

    /* Init the ECC hardware/structures */
    if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
    {
        retVal = nandInfo->hNandEccInfo->ECCInit(nandInfo);
        if (retVal & NAND_STATUS_FAILED)
        {
            return retVal;
        }
    }

    /* Init the DMA hardware/structures */
    if (NAND_XFER_MODE_DMA == nandInfo->opMode)
    {
        void *unused = NULL;
        (nandInfo->hNandDmaInfo->DMAInit)(unused);
    }

    /* Reset the device first */
    retVal = NANDReset(nandInfo);
    if(retVal & NAND_STATUS_WAITTIMEOUT)
    {
        return retVal;
    }
    
    /*
     * Read the device ID (will update NAND device parameters if current
     * parameters are invalid)
     */
    retVal = NANDReadId(nandInfo);

    return retVal;    
}


/**
* \brief  Function to wait till NAND is Ready. \n
*
*         This function Waits untill NAND device enters the  ready stateIt
*         reads the status of the NAND's ready/busy status by reading the
*         wait  status from the NANDFSR register. It also implements a   
*         timeout functionality to avoid indefinte lockup.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                          device information.\n
*
* \return NAND device status.\n
*
*         NAND_STATUS_PASSED      : If device is ready.\n
*         NAND_STATUS_WAITTIMEOUT : If device is not ready.\n
*/
NandStatus_t NANDWaitUntilReady(NandInfo_t *nandInfo)
{
    NandStatus_t retVal;
    unsigned int waitPinStatus;
    volatile int timeout;

    retVal  = NAND_STATUS_PASSED;
    timeout = NAND_RBWAIT_TIMEOUT;

    /* This function is called immediatly after issuing commands    *
     * to the NAND flash. Since the NAND flash takes sometime to    *
     * pull the R/B line low,it would be safe to introduce a delay  *
     * before checking the ready/busy status.                       */
    NANDDelay(0xFFF);
    /* Keep checking the status of ready/busy line.Also maintain a  * 
     * timeout count. If the NAND is not ready during the timeout   *
     * period, stop checking the ready/busy status                  */
    while (timeout > 0u)
    {
        /* Check the Ready/Busy status                              */
        waitPinStatus = ((nandInfo->hNandCtrlInfo->WaitPinStatusGet)(nandInfo));
        if (waitPinStatus != 0)
        /* Note: Check "should" be (waitPinStatus != 1); Need to investigate why
         * wait pin (~R/B pin) is not toggled on AM335x platform. */
        {
            /* NAND flash is ready. Break from the loop. */
            break;
        }
        timeout = timeout - 1u;
    }
    /* Determine if the wait for ready status ended due to timeout. */
    if (0 == timeout)
    {
        retVal = NAND_STATUS_WAITTIMEOUT;
    }
    return (retVal);
}

/**
* \brief  Function to reset the NAND.\n
*
*         This function Resets the NAND device by issuing the RESET command
*         (0xFF) to the NAND device. It then waits until the NAND device is.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                          device information.\n
*
* \return NAND device status.\n
*
*         NAND_STATUS_PASSED      : If device is ready.\n
*         NAND_STATUS_WAITTIMEOUT : If device is not ready.\n
*
*/
NandStatus_t NANDReset(NandInfo_t *nandInfo)
{
    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_RESET);
    return NANDWaitUntilReady(nandInfo);
}

/**
* \brief  Function to read the NAND Device ID.\n
*
*         This function reads the NAND device ID . It issues the
*         NAND read ID command and reads a 4-byte NAND device ID.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                          device information.\n
*
* \return Updates the device id info in the nandDevInfo structure in nandInfo.\n
*/
 NandStatus_t NANDReadId(NandInfo_t *nandInfo)
{
    NandStatus_t retVal = NAND_STATUS_PASSED;
    unsigned int devId;
    unsigned int blkSize = nandInfo->blkSize;
    unsigned int pageSize = nandInfo->pageSize;

    /* Send the read ID command */
    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_READID);

    /* Write one cycle address with address as zero */
    NANDAddressWrite(nandInfo->addrRegAddr, 0x00);
    NANDDelay(10);

    NANDWaitUntilReady(nandInfo);

    /* Read the 4-byte device ID */
    nandInfo->manId = NANDDataReadByte(nandInfo->dataRegAddr);
    nandInfo->devId = NANDDataReadByte(nandInfo->dataRegAddr);
    
    if( (nandInfo->devId==0x00) || (nandInfo->devId==0xFF) )
    {
      retVal = NAND_STATUS_NOT_FOUND;
    }

    NANDDataReadByte(nandInfo->dataRegAddr);
    devId = NANDDataReadByte (nandInfo->dataRegAddr);

    /* Only try to detect device info from 4th ID byte if no valid  */
    /* values were given at initialization                          */
    if ( (nandInfo->pageSize == NAND_PAGESIZE_INVALID) ||
         (nandInfo->blkSize == NAND_BLOCKSIZE_INVALID) )
    {
        nandInfo->pageSize      = (NandPageSize_t) (1024 << (devId & 0x03));
        nandInfo->blkSize       = (NandBlockSize_t) ((64 << ((devId >> 4) & 0x3))*1024);
    }
    /* Only try to detect bus width info from 4th ID byte if no valid  */
    /* values were given at initialization                             */    
    if (nandInfo->busWidth == NAND_BUSWIDTH_INVALID)
    {
        if(devId >> 6 & 0x01)
        {
            nandInfo->busWidth = NAND_BUSWIDTH_16BIT;
        }
        else
        {
            nandInfo->busWidth = NAND_BUSWIDTH_8BIT;
        }
    }
    
    /* Calculate the pagesPerBlock */
    nandInfo->pagesPerBlk   = (blkSize/pageSize);
    
    return retVal;
}

/**
* \brief  Function to Erases The block.\n
*
*         This function erases the block specified as argument.\n
*
* \param  nandInfo      :  Pointer to structure which conatins controller and 
*                          device information.\n
*
* \param  blkNum        :  Blk Number.\n
*
* \return Erase command status.\n
*
*         NAND_STATUS_PASSED         : If the erase command is passed.\n
*         NAND_STATUS_FAILED         : If the erase command is failed.\n
*         NAND_STATUS_DEVBUSY        : If the device is busy.\n
*         NAND_STATUS_DEVWRPROTECT   : If the device is write protect.\n
*         NAND_STATUS_WAITTIMEOUT    : If the RB pin inidcating device
*                                      busy status.\n
*
**/
NandStatus_t NANDBlockErase(NandInfo_t *nandInfo, unsigned int blkNum)
{
    unsigned int firstpageOfBlk;
    unsigned int count;
    NandStatus_t retVal;

    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_ERASE);
    /* Calculate linear page number of the first page of the block. */
    firstpageOfBlk = (blkNum * (nandInfo->pagesPerBlk));
    /* Write the row address. */
    for (count = 0; count < 3; count++)
    {
        NANDAddressWrite(nandInfo->addrRegAddr,
                         (unsigned char)firstpageOfBlk);
        firstpageOfBlk = firstpageOfBlk >> 8u;
    }
    NANDCommandWrite(nandInfo->cmdRegAddr, NAND_CMD_ERASE_CYC2);

    retVal = NANDWaitUntilReady(nandInfo);
    if(retVal != NAND_STATUS_WAITTIMEOUT)
    {
        retVal = NANDDeviceStatusGet(nandInfo);
    }

    return (retVal);
}

/**
* \brief  Function to write a page to NAND.\n
*
* \param  nandInfo        Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  blkNum        :  Blk Number.\n
*
* \param  pageNum       :  Page Number.\n
*
* \param  data          :  Data to write.\n
*
* \return Write command status.\n
*
*         NAND_STATUS_PASSED              : If the write command is passed.\n
*         NAND_STATUS_FAILED              : If the write command is failed.\n
*         NAND_STATUS_DEVBUSY             : If the device is busy.\n
*         NAND_STATUS_DEVWRPROTECT        : If the device is write protect.\n
*         NAND_STATUS_WAITTIMEOUT         : If the RB pin inidcating device
*                                           busy status.\n
*         NAND_STATUS_READWRITE_DMA_FAIL  : If DMA transfer is failed.\n
*
**/
NandStatus_t NANDPageWrite(NandInfo_t *nandInfo, unsigned int blkNum,
                           unsigned int pageNum, volatile unsigned char *txData,
                           unsigned char *eccData)
{
    NandStatus_t retVal;
    unsigned int trnsCnt;
    unsigned int columnAddr;
    unsigned char *eccDataTmp;
    
    columnAddr = 0;
    eccDataTmp = eccData;
    NANDPageWriteCmdStart(nandInfo, blkNum, pageNum, columnAddr);
    retVal = NAND_STATUS_PASSED;
    
    if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
    {
        (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
        (nandInfo->hNandEccInfo->ECCWriteSet)(nandInfo);
        (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
    }
    for(trnsCnt = 0; trnsCnt < ((nandInfo->pageSize) / NAND_BYTES_PER_TRNFS);
        trnsCnt++)
    {
        if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
        {    
            (nandInfo->hNandEccInfo->ECCEnable)(nandInfo);
        }
        if(nandInfo->opMode == NAND_XFER_MODE_DMA)
        {
            retVal = (nandInfo->hNandDmaInfo->DMAXfer)(nandInfo, (unsigned char*)txData,
                                 NAND_BYTES_PER_TRNFS,
                                 NAND_DMA_DIR_WRITE);
            if(retVal != NAND_STATUS_PASSED)
            {
                break;
            }
        }
        else
        {
            NANDDataWrite(nandInfo ,txData, NAND_BYTES_PER_TRNFS);
        }
        if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
        {    
            (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
            (nandInfo->hNandEccInfo->ECCCalculate)(nandInfo, eccData);
            eccData += nandInfo->hNandEccInfo->eccByteCnt;        
        }
        txData += NAND_BYTES_PER_TRNFS;
    }
    if(retVal == NAND_STATUS_PASSED)
    {
        retVal = NANDPageWriteCmdEnd(nandInfo);    
        
        if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
        {    
            eccData = eccDataTmp;
            if(retVal == NAND_STATUS_PASSED)
            {
                columnAddr = nandInfo->hNandEccInfo->eccOffSet;
                if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
                {
                    columnAddr = columnAddr/2;
                }
                NANDPageWriteCmdStart(nandInfo, blkNum, pageNum, columnAddr);
                NANDDataWrite(nandInfo, eccData, (nandInfo->hNandEccInfo->eccByteCnt)*((nandInfo->pageSize)/NAND_BYTES_PER_TRNFS));
                retVal = NANDPageWriteCmdEnd(nandInfo);
            }
        }
    }

    return (retVal);
}

/**
* \brief  Function to read a page from NAND.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  blkNum        : Blk Number.\n
*
* \param  pageNum       : Blk Number.\n
*
* \param  data          : Pointer to Data where read data has to place.\n
*
* \return Read command status.\n
*
*    NAND_STATUS_PASSED                       : Page read is succssfull without any 
*                                               ECC error.\n
*    NAND_STATUS_READ_ECC_ERROR_CORRECTED     : Page read is sucssfull with ECC errors
*                                               and are corrected.\n
*    NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR : Uncurrectable ECC errors occured during
*                                               page read.\n
*    NAND_STATUS_WAITTIMEOUT                  : Device is busy.\n
*
*    NAND_STATUS_READWRITE_DMA_FAIL           : If DMA transfer is failed.\n
*
**/
NandStatus_t NANDPageRead(NandInfo_t *nandInfo,unsigned int blkNum,
                          unsigned int pageNum, volatile unsigned char *rxData,
                          unsigned char *eccData)
{
    NandStatus_t retVal;
    unsigned int trnsCnt;
    unsigned int eccCorFlag;
    unsigned int columnAddr;
    NandEccInfo_t *nandEccInfo = nandInfo->hNandEccInfo;

    retVal = NAND_STATUS_PASSED;
    eccCorFlag = 0;

    (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
    columnAddr = nandEccInfo->eccOffSet;
    if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
    {
        columnAddr = columnAddr/2;
    }
    retVal = NANDPageReadCmdSend(nandInfo, blkNum, pageNum, columnAddr);
    if(retVal == NAND_STATUS_WAITTIMEOUT)
    {
        return (retVal);
    }

    /* Read the ECC Data from spare area */
    NANDDataRead(nandInfo, eccData, (nandEccInfo->eccByteCnt)*((nandInfo->pageSize)/NAND_BYTES_PER_TRNFS));

    /* Set the column addr to start of the page */
    columnAddr = 0;

    retVal = NANDPageReadCmdSend(nandInfo, blkNum, pageNum, columnAddr);
    if(retVal == NAND_STATUS_WAITTIMEOUT)
    {
        return (retVal);
    }

    if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
    {
        (nandInfo->hNandEccInfo->ECCReadSet)(nandInfo);
        (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
    }
    for(trnsCnt = 0; trnsCnt < ((nandInfo->pageSize)/ NAND_BYTES_PER_TRNFS) ; trnsCnt++)
    {
        if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
        {
            (nandInfo->hNandEccInfo->ECCEnable)(nandInfo);
        }
        /* Read the sub-page from the data area. */
        if(nandInfo->opMode == NAND_XFER_MODE_DMA)
        {
            retVal = (nandInfo->hNandDmaInfo->DMAXfer)(nandInfo, (unsigned char *)rxData,
                                 NAND_BYTES_PER_TRNFS,
                                 NAND_DMA_DIR_READ);
            if(retVal != NAND_STATUS_PASSED)
            {
                break;
            }
        }
        else
        {
            NANDDataRead(nandInfo, rxData, NAND_BYTES_PER_TRNFS);
        }
                
        if (NAND_ECC_ALGO_NONE != nandInfo->eccType)
        {        
            if( nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT )
            {
                /* Read the ECC data, as BCH algo reguires this for syndrome 
                 *  calculation.
                 */
                columnAddr = (nandEccInfo->eccOffSet) + (trnsCnt * nandEccInfo->eccByteCnt);
                if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
                {
                    columnAddr = columnAddr/2;
                }
                retVal = NANDPageReadCmdSend(nandInfo, blkNum, pageNum, columnAddr);
                if(retVal == NAND_STATUS_WAITTIMEOUT)
                {
                    return (retVal);
                }
                /* Read the ECC Data from spare area */
                NANDDataRead(nandInfo, eccData, (nandEccInfo->eccByteCnt - 1));
            }        
            
            /* Check for ECC errors and correct if any errors */
            retVal = (nandInfo->hNandEccInfo->ECCCheckAndCorrect)(nandInfo, eccData,
                                            (unsigned char *)rxData);
            (nandInfo->hNandEccInfo->ECCDisable)(nandInfo);
            if( (retVal & NAND_STATUS_READ_ECC_ERROR_CORRECTED) )
            {
                eccCorFlag = 1;
            }
            else if( (retVal & NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR) )
            {
                break;
            }
            if( nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT )
            {
                /* Reset the colomn pointer to appropariate position.  */

                columnAddr = ((trnsCnt + 1) * NAND_BYTES_PER_TRNFS);
                if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
                {
                    columnAddr = columnAddr/2;
                }
                NANDCommandWrite(nandInfo->cmdRegAddr,
                                 NAND_CMD_READ_RANDOM);
                /* Write 2 bytes of column addr */
                NANDAddressWrite(nandInfo->addrRegAddr,
                                (unsigned char)(columnAddr & 0xFF));
                NANDAddressWrite(nandInfo->addrRegAddr,
                                 (unsigned char)((columnAddr >> 8) & 0xFF));
                NANDCommandWrite(nandInfo->cmdRegAddr,
                                 NAND_CMD_READ_RANDOM_CYC2);
                retVal = NANDWaitUntilReady(nandInfo);
                if(retVal == NAND_STATUS_WAITTIMEOUT)
                {
                    return (retVal);
                }
            }
            eccData += nandEccInfo->eccByteCnt;
        }
        rxData += NAND_BYTES_PER_TRNFS;
    }
    if( eccCorFlag && (!(retVal & NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR)) )
    {
        retVal = NAND_STATUS_READ_ECC_ERROR_CORRECTED;
    }

    return (retVal);
}

/**
* \brief  This function Check whether Block is bad or Not.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  blkNum        : Blk Number.\n
*
* \return Bad block status.\n
*
*       NAND_BLOCK_BAD                    - If Block is bad.\n
*       NAND_BLOCK_GOOD                   - If Block is Good.\n
*       NAND_BLOCK_SPARE_AREA_READ_FAILED - Error while reading the spare area.\n
*
**/
NandBlockStatus_t NANDBadBlockCheck(NandInfo_t *nandInfo, unsigned int blkNum)
{
    NandBlockStatus_t retVal = NAND_BLOCK_GOOD;
    unsigned int pageNum = 0;    
    unsigned int columnAddr;

    columnAddr  = nandInfo->pageSize;
    
    /* Adjust column address for 16-bit devices */
    if( NAND_BUSWIDTH_16BIT == nandInfo->busWidth )
    {
        columnAddr = columnAddr >> 1;
    }    

   /* Read the spare area of 1st, 2nd and last page of the block    */
    for (pageNum = 0; pageNum < 3u; pageNum++)
    {
        unsigned char badBlkMark[2];
        
        /* Last page number of the block */
        if (pageNum == 2)
        {
            pageNum = (nandInfo->pagesPerBlk - 1u);
        }
        
        if( NAND_STATUS_WAITTIMEOUT == 
            NANDPageReadCmdSend(nandInfo, blkNum, pageNum, columnAddr))
        {
            retVal = NAND_BLOCK_SPARE_AREA_READ_FAILED;
            break;
        }

        /* Read the ECC Data from spare area */
        NANDDataRead(nandInfo, &badBlkMark[0], 2);
        
        /* Check for bad block marker */
        if(badBlkMark[0] != NAND_BLK_GOOD_MARK)
        {
            retVal = NAND_BLOCK_BAD;
            break;
        }
    }
    return (retVal);
}

/**
* \brief  This function Marks the block as BAD by writing non 0xFF value in to
*         the spare area.\n
*
* \param  nandInfo      : Pointer to structure which conatins controller and 
*                         device information.\n
*
* \param  blkNum        : Blk Number.\n
*
*
* \return
*
*         NAND_STATUS_PASSED        : If the write command is passed.\n
*         NAND_STATUS_FAILED        : If the write command is failed.\n
*         NAND_STATUS_DEVBUSY       : If the device is busy.\n
*         NAND_STATUS_DEVWRPROTECT  : If the device is write protect.\n
*         NAND_STATUS_WAITTIMEOUT   : If the RB pin inidcating device 
*                                     busy status.\n
*
**/
NandStatus_t NANDMarkBlockAsBad(NandInfo_t *nandInfo, unsigned int blkNum)
{
    unsigned int pageNum;
    unsigned int columnAddr;
    NandStatus_t retVal = NAND_STATUS_PASSED;

    columnAddr  = nandInfo->pageSize;

    /* Adjust column address for 16-bit devices */    
    if(nandInfo->busWidth == NAND_BUSWIDTH_16BIT)
    {
        columnAddr = columnAddr >> 1;
    }

    /* First erase the block before marking it as bad. The result of *
     * marking this as bad can be ignored.                           */
    NANDBlockErase(nandInfo, blkNum);

    /* Read the spare area of 1st, 2nd and last page of the block */
    for (pageNum = 0; pageNum < 3u; pageNum++)
    {
        unsigned char badBlkMark[2];
    
        /* Last page number of the block */
        if (pageNum == 2)
        {
            pageNum = (nandInfo->pagesPerBlk - 1u);
        }
        badBlkMark[0] = NAND_BLK_BAD_MARK;
        NANDPageWriteCmdStart(nandInfo, blkNum, pageNum, columnAddr);
        NANDDataWrite(nandInfo, badBlkMark, 2);
        retVal = NANDPageWriteCmdEnd(nandInfo);
        if (retVal != NAND_STATUS_PASSED)
        {
            break;
        }
    }
    return (retVal);
}

/******************************************************************************
**                              END OF FILE
*******************************************************************************/
