/**
 *  \file   nand_gpmc.c
 *
 *  \brief  This file contains the NAND GPMC controller specific functions 
 *          and these are provided as platform specific code 
 *          to achieve the required functionality.
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


#include "gpmc.h"
#include "elm.h"
#include "nandlib.h"
#include "nand_gpmc.h"
#include "hw_types.h"

/*******************************************************************************
*                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/*****************************************************************************/
/*
** Macro which defines the shift value for chip select base address.
*/
#define NAND_BASE_ADDR_SHIFT                    (24)

/*****************************************************************************/
/*
** Macros which defines the ECC size values.
*/
#define GPMC_ECC_SIZE0_VAL                      (0xFF)
#define GPMC_ECC_SIZE1_VAL                      (0xFF)

/*****************************************************************************/
/*
** Macros which defines the FIFO threshold for prefetch engine.
*/
#define GPMC_NAND_PREFETCH_FIFO_THRLD           (64)

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/**
*\brief This function does the 1-bit hamming code ECC related initializes 
*       to the NAND controller.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \return 
*        NAND_STATUS_PASSED : On success.\n
*        NAND_STATUS_FAILED : On failure.\n
*
*/
static NandStatus_t GPMCHammingCodeECCInit(NandInfo_t *nandInfo)
{
    unsigned int baseAddr;
    unsigned int cs;
    NandStatus_t retVal;

    retVal = NAND_STATUS_PASSED;
    cs = nandInfo->hNandCtrlInfo->currChipSelect;
    baseAddr = nandInfo->hNandCtrlInfo->baseAddr;

    GPMCECCAlgoSelect(baseAddr, GPMC_ECC_ALGORITHM_HAMMINGCODE);
    GPMCECCColumnSelect(baseAddr, GPMC_ECC_COLUMNS_8);
    GPMCECCCSSelect(baseAddr, cs);
    GPMCECCDisable(baseAddr);
    GPMCECCResultRegSelect(baseAddr, GPMC_ECCPOINTER_RESULT_1);
    GPMCECCResultRegClear(baseAddr);
    GPMCECCSizeValSet(baseAddr, GPMC_ECC_SIZE_0, GPMC_ECC_SIZE0_VAL);
    GPMCECCSizeValSet(baseAddr, GPMC_ECC_SIZE_1, GPMC_ECC_SIZE1_VAL);
    GPMCECCResultSizeSelect(baseAddr, GPMC_ECC_RESULT_1, GPMC_ECC_SIZE_0);

    nandInfo->hNandEccInfo->eccOffSet  = nandInfo->pageSize + NAND_ECC_1BIT_HAMMINGCODE_OOB_OFFSET;
    nandInfo->hNandEccInfo->eccByteCnt = NAND_ECC_1BIT_HAMMINGCODE_BYTECNT;

    return(retVal);
}

/**
*\brief  This Function does the Hamming code ECC related setting for write.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
static void GPMCHammingCodeWriteSet(NandInfo_t *nandInfo)
{
    /* No special setting required in Hamming code for write */
}

/**
*\brief  This Function does the Hamming code ECC related setting for Read.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
static void GPMCHammingCodeReadSet(NandInfo_t *nandInfo)
{
    /* No special setting required in Hamming code for Read */
}

/**
*\brief This function reads/calculates the 1-bit hamming code ECC values.\n
*
* \param  baseAddr      : Base address of the GPMC controller.\n
*
* \param  eccResReg     : ECC Result register value.\n
*
* \param  eccRead       : Pointer where read ECC data has to store.\n
*
* \return none.\n
*
*/
static void GPMCHammingCodeECCCalculate(unsigned int baseAddr, unsigned eccResReg,
                                 unsigned char *ptrEccData)
{
    unsigned int eccVal;

    eccVal  = GPMCECCResultGet(baseAddr, eccResReg);
   /* Squeeze 4 bytes ECC into 3 bytes by removing RESERVED bits
    * and shifting. RESERVED bits are 31 to 28 and 15 to 12. */
    eccVal = (eccVal & 0x00000fff) | ((eccVal & 0x0fff0000) >> 4);
    /* Invert so that erased block ECC is correct */
    eccVal = ~eccVal;

    *ptrEccData++ = (unsigned char)eccVal;
    *ptrEccData++ = (unsigned char)eccVal >>  8;
    *ptrEccData++ = (unsigned char)eccVal >> 16;
}

/**
* \brief This function checks for ECC errors using 1-bit hamming code algorithm
*        and correct if any ECC errors.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \param   eccRead      : Pointer to the ECC data which is read from the spare
*                         area.\n
*
* \param   data         : Pointer to the data, where if an ecc error need to 
*                         correct.\n
*
* \return ECC correction Status.\n
*    NAND_STATUS_PASSED                        : If no ecc errors.\n
*    NAND_STATUS_READ_ECC_ERROR_CORRECTED      : If error are corrected.\n
*    NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR  : If errors are uncorrectable.\n
*
*/
static NandStatus_t GPMCHammingCodeECCCheckAndCorrect(NandInfo_t *nandInfo,
                                                      unsigned char *eccRead,
                                                      unsigned char *data)
{
    NandStatus_t retVal;
    unsigned char eccCalc[4];
    unsigned int readEccVal;
    unsigned int calcEccVal;
    unsigned int eccDiffVal;
    unsigned int bitPos;
    unsigned int bytePos;

    retVal = NAND_STATUS_PASSED;

    (nandInfo->hNandEccInfo->ECCCalculate)(nandInfo, &eccCalc[0]);
    readEccVal = eccRead[0] | (eccRead[1] << 8) | (eccRead[2] << 16);
    calcEccVal = eccCalc[0] | (eccCalc[1] << 8) | (eccCalc[2] << 16);
    eccDiffVal = readEccVal ^ calcEccVal;

    if(eccDiffVal)
    {
        /*
         * No error              : The ecc diff value (eccDiffVal) is 0.
         * Correctable error     : For 512-byte inputs, ecc diff value has
         *                         12 bits at 1. For 256 byte ecc diff value has
         *                         11 bits at 1.
         * ECC error             : The ecc diff value has only 1 bit at 1.
         * Non-correctable error : The ecc diff value provides all other results
         */

        /*
         * Beow condition checks for number of 1's in eccDiffValu.
         * Since Total ecc has 3bytes = 24 bits. Make 2 halfs and XOR.
         * If eccDiffVal has  12 1's, it produces the result 0xFFF.
        */
        if ((((eccDiffVal >> 12) ^ eccDiffVal) & 0xfff) == 0xfff)
        {
             /* Correctable error */
            /* Check bytePos is within NAND_BYTES_PER_TRNFS i.e 512 */
            if ((eccDiffVal >> (12 + 3)) < NAND_BYTES_PER_TRNFS)
            {
                bitPos  = 1 << ((eccDiffVal >> 12) & 7);
                bytePos = eccDiffVal >> (12 + 3);
                data[bytePos] ^= bitPos;
                retVal = NAND_STATUS_READ_ECC_ERROR_CORRECTED;
            }
            else
            {
                retVal = NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR;
            }
        }
        else if(!(eccDiffVal & (eccDiffVal - 1)))
        {
            /* Single bit ECC error in the ECC itself,nothing to fix */
            retVal = NAND_STATUS_READ_ECC_ERROR_CORRECTED;
        }
        else
        {
            retVal = NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR;
        }
    }

    return (retVal);
}

/**
*\brief This function does the BCH ECC related initializes to the NAND 
*       controller.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \return
*        NAND_STATUS_PASSED          : On success.\n
*        NAND_STATUS_FAILED          : On failure.\n
*        NAND_STATUS_ECC_UNSUPPORTED : If unsupported ECC is used.\n
*
*/
static NandStatus_t GPMCBCHECCInit(NandInfo_t *nandInfo)
{
    unsigned int baseAddr;
    unsigned int elmBaseAddr;
    unsigned int cs;
    volatile unsigned int timeOut;
    NandStatus_t retVal;

    timeOut = 0xFFF;
    retVal = NAND_STATUS_PASSED;
    cs = nandInfo->hNandCtrlInfo->currChipSelect;
    baseAddr = nandInfo->hNandCtrlInfo->baseAddr;
    elmBaseAddr = nandInfo->hNandEccInfo->baseAddr;

    GPMCECCDisable(baseAddr);
    GPMCECCAlgoSelect(baseAddr, GPMC_ECC_ALGORITHM_BCH);
    if(nandInfo->eccType != NAND_ECC_ALGO_BCH_8BIT)
    {
        retVal = NAND_STATUS_ECC_UNSUPPORTED;
    }
    else
    {
        GPMCECCBCHErrCorrectionCapSelect(baseAddr,
                                         GPMC_ECC_BCH_ERRCORRCAP_UPTO_8BITS);
        GPMCECCColumnSelect(baseAddr, nandInfo->busWidth);
        GPMCECCCSSelect(baseAddr, cs);
        GPMCECCBCHNumOfSectorsSelect(baseAddr, GPMC_ECC_BCH_NUMOFSECTS_1);
        GPMCECCBCHWrapModeValSet(baseAddr, 1);

        GPMCECCResultRegSelect(baseAddr, GPMC_ECCPOINTER_RESULT_1);
        GPMCECCResultRegClear(baseAddr);

        GPMCECCSizeValSet(baseAddr, GPMC_ECC_SIZE_0, GPMC_ECC_SIZE0_VAL);
        GPMCECCSizeValSet(baseAddr, GPMC_ECC_SIZE_1, GPMC_ECC_SIZE1_VAL);
        GPMCECCResultSizeSelect(baseAddr, GPMC_ECC_RESULT_1, GPMC_ECC_SIZE_0);

        /* ELM Module configuration */
        ELMModuleReset(elmBaseAddr);
        while((ELMModuleResetStatusGet(elmBaseAddr) != 1) && (timeOut != 0))
        {
            timeOut--;
        }
        if(timeOut == 0)
        {
            retVal = NAND_STATUS_FAILED;
        }
        else
        {
            ELMCAutoGatingConfig(elmBaseAddr, ELM_AUTOGATING_OCP_FREE);
            ELMCIdleModeSelect(elmBaseAddr, ELM_IDLEMODE_NOIDLE);
            ELMOCPClkActivityConfig(elmBaseAddr, ELM_CLOCKACTIVITYOCP_OCP_ON);
            ELMIntStatusClear(elmBaseAddr, ELM_LOC_VALID_0_STATUS);
            ELMIntConfig(elmBaseAddr, ELM_LOC_VALID_0_STATUS, ELM_INT_ENALBLE);
            ELMErrCorrectionLevelSet(elmBaseAddr, ELM_ECC_BCH_LEVEL_8BITS);
            ELMECCSizeSet(elmBaseAddr, 0x7FF);
            ELMModeSet(elmBaseAddr, ELM_MODE_PAGE, ELM_PAGEMODE_SECTOR_0);
            nandInfo->hNandEccInfo->eccOffSet  = NAND_ECC_BCH_8BIT_OOB_OFFSET + nandInfo->pageSize;
            nandInfo->hNandEccInfo->eccByteCnt = NAND_ECC_BCH_8BIT_BYTECNT;
        }
    }

    return (retVal);
}

/**
*\brief  This Function does the BCH ECC related setting for write.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
static void GPMCBCHWriteSet(NandInfo_t *nandInfo)
{
    unsigned int size1;
    unsigned int size0;

    size1 = 0;
    size0 = 0;

    if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT)
    {
       /* Not Supported */
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        size1 = (NAND_ECC_BCH_8BIT_BYTECNT * 2);
        size0 = 0;
    }
    GPMCECCSizeValSet(nandInfo->hNandCtrlInfo->baseAddr, GPMC_ECC_SIZE_0, size0);
    GPMCECCSizeValSet(nandInfo->hNandCtrlInfo->baseAddr, GPMC_ECC_SIZE_1, size1);
}

/**
*\brief  This Function does the BCH ECC related setting for read.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
static void GPMCBCHReadSet(NandInfo_t *nandInfo)
{
    unsigned int size1;
    unsigned int size0;

    size1 = 0;
    size0 = 0;

    if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT)
    {
       /* Not Supported */
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        size0 = ((NAND_ECC_BCH_8BIT_BYTECNT * 2) -
                  NAND_ECC_BCH_8BIT_UNUSED_BYTECNT);
        size1 = NAND_ECC_BCH_8BIT_UNUSED_BYTECNT;
    }
    GPMCECCSizeValSet(nandInfo->hNandCtrlInfo->baseAddr, GPMC_ECC_SIZE_0, size0);
    GPMCECCSizeValSet(nandInfo->hNandCtrlInfo->baseAddr, GPMC_ECC_SIZE_1, size1);
}

/**
*\brief This function reads/calculates the BCH 4-bit andc 8-bit ECC values.\n
*
* \param  baseAddr      : Base address of the GPMC controller.\n
*
* \param  eccRead       : Pointer where read ECC data has to store.\n
*
* \param  eccType       : BCH ECC type.\n
*                         This can take one of the following values :\n
*                         NAND_ECC_ALGO_BCH_4BIT -- For 4-bit BCH ECC.\n
*                         NAND_ECC_ALGO_BCH_8BIT -- For 8-bit BCH ECC.\n
* \return none.\n
*
*/
static void GPMCBCHECCCalculate(NandInfo_t *nandInfo, unsigned char *ptrEccData)
{
    unsigned int eccRes;
    unsigned int baseAddr;
    unsigned int eccType;
    unsigned int cs;

    cs = nandInfo->hNandCtrlInfo->currChipSelect;
    baseAddr = nandInfo->hNandCtrlInfo->baseAddr;
    eccType  = nandInfo->eccType;

    if(eccType == NAND_ECC_ALGO_BCH_4BIT)
    {
       /* Not Supported */
    }
    else if(eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        eccRes = GPMCECCBCHResultGet(baseAddr, GPMC_BCH_RESULT_3, cs);
        ptrEccData[0] = (eccRes & 0xFF);
        eccRes = GPMCECCBCHResultGet(baseAddr, GPMC_BCH_RESULT_2, cs);
        ptrEccData[1] = ((eccRes >> 24) & 0xFF);
        ptrEccData[2] = ((eccRes >> 16) & 0xFF);
        ptrEccData[3] = ((eccRes >> 8) & 0xFF);
        ptrEccData[4] = (eccRes & 0xFF);
        eccRes = GPMCECCBCHResultGet(baseAddr, GPMC_BCH_RESULT_1, cs);
        ptrEccData[5] = ((eccRes >> 24) & 0xFF);
        ptrEccData[6] = ((eccRes >> 16) & 0xFF);
        ptrEccData[7] = ((eccRes >> 8) & 0xFF);
        ptrEccData[8] = (eccRes & 0xFF);
        eccRes = GPMCECCBCHResultGet(baseAddr, GPMC_BCH_RESULT_0, cs);
        ptrEccData[9] = ((eccRes >> 24) & 0xFF);
        ptrEccData[10] = ((eccRes >> 16) & 0xFF);
        ptrEccData[11] = ((eccRes >> 8) & 0xFF);
        ptrEccData[12] = (eccRes & 0xFF);
    }
}

/**
* \brief This function checks for ECC errors using BCH algorithm and corrects
*        if any ECC errors. \n
*
* \pam  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \param   eccRead      : Pointer to the ECC data which is read from the spare
*                         area.\n
*
* \param   data         : Pointer to the data, where if an ecc error need to 
*                         correct.\n
*
* \return ECC correction Status.\n
*    NAND_STATUS_PASSED                        : If no ecc errors.\n
*    NAND_STATUS_READ_ECC_ERROR_CORRECTED      : If error are corrected.\n
*    NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR  : If errors are uncorrectable.\n
*
*/
static NandStatus_t GPMCBCHECCCheckAndCorrect(NandInfo_t *nandInfo,
                                              unsigned char *eccRead,
                                              unsigned char *data)
{
    NandStatus_t retVal;
    unsigned int elmBaseAddr;
    unsigned int intStatus;
    unsigned int eccVal;
    unsigned int i;
    unsigned int errNum;
    unsigned int numOfErrs;
    unsigned int errLoc;
    unsigned int lastECCBit;
    unsigned int lastDataBit;
    unsigned int errBitMask;
    unsigned int errBytePos;
    unsigned int numBytes;
    unsigned char eccCalc[NAND_ECC_BCH_8BIT_BYTECNT * 4];
    unsigned char syndrome[NAND_ECC_BCH_8BIT_BYTECNT * 4];
    unsigned int j;
    unsigned int result;

    retVal = NAND_STATUS_PASSED;
    elmBaseAddr = nandInfo->hNandEccInfo->baseAddr;
    intStatus = 0;
    j = 0;
    numBytes = 0;

    if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT)
    {
       /* Not Supported */
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        numBytes    = ((NAND_ECC_BCH_8BIT_BYTECNT) - 1);
        lastECCBit  = NAND_ECC_BCH_8BIT_LASTECCBIT;
        lastDataBit = NAND_ECC_BCH_8BIT_LASTDATABIT;
    }
    (nandInfo->hNandEccInfo->ECCCalculate)(nandInfo, &eccCalc[0]);
    /* while reading ECC result we read it in big endian.
     * Hence while loading to ELM we have rotate to get the right endian.
     */

    /* Rotate the syndrome bytes */
    for (i = 0, j = (numBytes-1); i < numBytes; i++, j--)
          syndrome[i] = eccCalc[j];

    /* Load the BCH syndrome */
    eccVal = (syndrome[0] | (syndrome[1] << 8) | (syndrome[2] << 16) |
              (syndrome[3] << 24));
    ELMSyndromeFrgmtSet(elmBaseAddr, ELM_SYNDROME_FRGMT_0, eccVal);

    eccVal = (syndrome[4] | (syndrome[5] << 8) | (syndrome[6] << 16) |
              (syndrome[7] << 24));
    ELMSyndromeFrgmtSet(elmBaseAddr, ELM_SYNDROME_FRGMT_1, eccVal);

    if(nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        eccVal = (syndrome[8] | (syndrome[9] << 8) | (syndrome[10] << 16) |
                 (syndrome[11] << 24));
        ELMSyndromeFrgmtSet(elmBaseAddr, ELM_SYNDROME_FRGMT_2, eccVal);

        eccVal = (syndrome[12] | (syndrome[13] << 8) | (syndrome[14] << 16) |
                 (syndrome[15] << 24));
        ELMSyndromeFrgmtSet(elmBaseAddr, ELM_SYNDROME_FRGMT_3, eccVal);
    }

    ELMErrLocProcessingStart(elmBaseAddr);

    while(intStatus == 0)
    {
        intStatus = ELMIntStatusGet(elmBaseAddr, ELM_LOC_VALID_0_STATUS);
    }

    ELMIntStatusClear(elmBaseAddr, ELM_LOC_VALID_0_STATUS);

    result = ELMErrLocProcessingStatusGet(elmBaseAddr);

    if(result == 0)
    {
        retVal = NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR;
    }
    else
    {
        numOfErrs = ELMNumOfErrsGet(elmBaseAddr);
        if(numOfErrs == 0)
        {
            retVal = NAND_STATUS_PASSED;
        }
        else
        {
            errNum = ELM_ERROR_NUM_0;
             /* Get the error location and correct the same */
            for(i=0; i < numOfErrs; i++)
            {
                errLoc = ELMErrLocBitAddrGet(elmBaseAddr, errNum);
                if (errLoc >= (lastECCBit - 1))
                {
                   /* Error is at the Data bytes */
                    errBytePos = ((lastDataBit - 1) - errLoc) / 8;
                    /* Error Bit mask */
                    errBitMask = 0x1 << (errLoc % 8);
                    /* Toggle the error bit to make the correction. */
                    data[errBytePos] ^= errBitMask;
                    retVal = NAND_STATUS_READ_ECC_ERROR_CORRECTED;
                }
                else
                {
                    /* Error is at the ECC bytes which we are not handling */
                }
                errNum++;
            }
        }
    }

    return (retVal);
}


/******************************************************************************
**                       GLOBAL FUNCTION DEFINITIONS
*******************************************************************************/

/**
* \brief  Function to get the wait pin status.\n
*
* \param  nandInfo      : Pointer to structure containing controller and 
*                       : device information.\n
*
* \return Wait pin status.\n
*
*         0 : If the status is active low.\n
*         1 : If the status is active high.\n
*
*/
unsigned int GPMCNANDWaitPinStatusGet(NandInfo_t *nandInfo)
{
    unsigned int pinStatus;

    pinStatus = GPMCWaitPinStatusGet(nandInfo->hNandCtrlInfo->baseAddr,
                                     nandInfo->hNandCtrlInfo->waitPin);
    return(pinStatus);
}

/**
* \brief  Function to get the GPMC FIFO status.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                       : device information.\n
*
* \return GPMC FIFO buffer status.\n
*
*         0 : If the FIFO is full.\n
*         1 : If the FIFO is empty and ready to accept data.\n
*
*/
unsigned int GPMCNANDWriteBufReady(NandInfo_t *nandInfo)
{
    unsigned int status;

    status = GPMCEmptyWriteBuffStatusGet(nandInfo->hNandCtrlInfo->baseAddr);

    return(status);
}

/**
* \brief  Function to initialize the NAND controller.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return 
*
*        NAND_STATUS_PASSED          : On success.\n
*        NAND_STATUS_FAILED          : On failure.\n
*
*/
NandStatus_t GPMCNANDInit(NandInfo_t *nandInfo)
{
    unsigned int conf;
    unsigned int baseAddr;
    unsigned int cs;
    volatile unsigned int timeOut;

    GPMCNANDTimingInfo_t *nandTimingInfo;

    cs = nandInfo->hNandCtrlInfo->currChipSelect;
    nandTimingInfo = (GPMCNANDTimingInfo_t *) nandInfo->hNandCtrlInfo->hNandTimingInfo;

    conf = 0;
    timeOut =0xFFF;
    baseAddr = nandInfo->hNandCtrlInfo->baseAddr;

    /* GPMC Module configuration */
    GPMCModuleSoftReset(baseAddr);
    while((GPMCModuleResetStatusGet(baseAddr) != 1) && (timeOut != 0))
    {
        timeOut--;
    }
    if(timeOut == 0)
    {
        return (NAND_STATUS_FAILED);
    }

    GPMCIdleModeSelect(baseAddr, GPMC_IDLEMODE_NOIDLE);

    /* Disable all interrupts */
    GPMCIntDisable(baseAddr, GPMC_FIFOEVENT_INT);
    GPMCIntDisable(baseAddr, GPMC_TERMINALCOUNT_INT);
    GPMCIntDisable(baseAddr, GPMC_WAIT0EDGEDETECTION_INT);
    GPMCIntDisable(baseAddr, GPMC_WAIT1EDGEDETECTION_INT);

    /* Timeout control disable */
    GPMCTimeOutFeatureConfig(baseAddr, GPMC_TIMEOUTFEATURE_DISABLE);

    /* Set the wait pin polarity */
    GPMCWaitPinSelect(baseAddr, cs, nandInfo->hNandCtrlInfo->waitPin);
    GPMCWaitPinPolaritySelect(baseAddr, nandInfo->hNandCtrlInfo->waitPin,
                              nandInfo->hNandCtrlInfo->waitPinPol);
    GPMCWriteProtectPinLevelCtrl(baseAddr, nandInfo->hNandCtrlInfo->wpPinPol);
    GPMCLimitedAddrDevSupportConfig(baseAddr,
                                    GPMC_LIMITEDADDRESS_SUPPORT_ENABLE);

    GPMCCSConfig(baseAddr, cs, GPMC_CS_DISABLE);
    GPMCTimeParaGranularitySelect(baseAddr, cs, GPMC_TIMEPARAGRANULARITY_X2);
    GPMCDevTypeSelect(baseAddr, cs, GPMC_DEVICETYPE_NANDLIKE);
    if(nandInfo->busWidth == NAND_BUSWIDTH_8BIT)
    {
        GPMCDevSizeSelect(baseAddr, cs, GPMC_DEVICESIZE_8BITS);
    }
    else
    {
        GPMCDevSizeSelect(baseAddr, cs, GPMC_DEVICESIZE_16BITS);
    }
    GPMCAddrDataMuxProtocolSelect(baseAddr, cs, GPMC_MUXADDDATA_NOMUX);

    GPMCWriteTypeSelect(baseAddr, cs, GPMC_WRITETYPE_ASYNC);
    GPMCReadTypeSelect(baseAddr, cs, GPMC_READTYPE_ASYNC);
    GPMCAccessTypeSelect(baseAddr, cs, GPMC_MODE_WRITE, GPMC_ACCESSTYPE_SINGLE);
    GPMCAccessTypeSelect(baseAddr, cs, GPMC_MODE_READ, GPMC_ACCESSTYPE_SINGLE);

    GPMCBaseAddrSet(baseAddr, cs, (nandInfo->hNandCtrlInfo->chipSelectBaseAddr[cs]) >>
                                   NAND_BASE_ADDR_SHIFT);
    GPMCMaskAddrSet(baseAddr, cs, (nandInfo->hNandCtrlInfo->chipSelectRegionSize[cs]));

    conf = GPMC_CS_TIMING_CONFIG(nandTimingInfo->CSWrOffTime,
                                 nandTimingInfo->CSRdOffTime,
                                 nandTimingInfo->CSExtDelayFlag,
                                 nandTimingInfo->CSOnTime);
    GPMCCSTimingConfig(baseAddr, cs, conf);

    conf = GPMC_ADV_TIMING_CONFIG(nandTimingInfo->ADVAADMuxWrOffTime,
                                  nandTimingInfo->ADVAADMuxRdOffTime,
                                  nandTimingInfo->ADVWrOffTime,
                                  nandTimingInfo->ADVRdOffTime,
                                  nandTimingInfo->ADVExtDelayFlag,
                                  nandTimingInfo->ADVAADMuxOnTime,
                                  nandTimingInfo->ADVOnTime);
    GPMCADVTimingConfig(baseAddr, cs, conf);

    conf = GPMC_WE_OE_TIMING_CONFIG(nandTimingInfo->WEOffTime,
                                    nandTimingInfo->WEExtDelayFlag,
                                    nandTimingInfo->WEOnTime,
                                    nandTimingInfo->OEAADMuxOffTime,
                                    nandTimingInfo->OEOffTime,
                                    nandTimingInfo->OEExtDelayFlag,
                                    nandTimingInfo->OEAADMuxOnTime,
                                    nandTimingInfo->OEOnTime);
    GPMCWEAndOETimingConfig(baseAddr, cs, conf);

    conf = GPMC_RDACCESS_CYCLETIME_TIMING_CONFIG(
                                   nandTimingInfo->rdCycleTime,
                                   nandTimingInfo->wrCycleTime,
                                   nandTimingInfo->rdAccessTime,
                                   nandTimingInfo->pageBurstAccessTime);
    GPMCRdAccessAndCycleTimeTimingConfig(baseAddr, cs, conf);

    conf = GPMC_CYCLE2CYCLE_BUSTURNAROUND_TIMING_CONFIG(
                                  nandTimingInfo->cycle2CycleDelay,
                                  nandTimingInfo->cycle2CycleDelaySameCSCfg,
                                  nandTimingInfo->cycle2CycleDelayDiffCSCfg,
                                  nandTimingInfo->busTAtime);
    GPMCycle2CycleAndTurnArndTimeTimingConfig(baseAddr, cs, conf);

    GPMCWrAccessAndWrDataOnADMUXBusTimingConfig(baseAddr, cs,
                                                nandTimingInfo->wrAccessTime,
                                                nandTimingInfo->wrDataOnADMux);
    GPMCCSConfig(baseAddr, cs, GPMC_CS_ENABLE);

    return(NAND_STATUS_PASSED);
}

/**
*\brief This function does the ECC related initializes to the NAND controller 
*       depending on the ecc type.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \return 
*        NAND_STATUS_PASSED : On success.\n
*        NAND_STATUS_FAILED : On failure.\n
*
*/
NandStatus_t GPMCNANDECCInit(NandInfo_t *nandInfo)
{
    NandStatus_t retVal;

    retVal = NAND_STATUS_PASSED;

    if(nandInfo->eccType == NAND_ECC_ALGO_HAMMING_1BIT)
    {
        retVal = GPMCHammingCodeECCInit(nandInfo);
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT ||
            nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        retVal = GPMCBCHECCInit(nandInfo);
    }

    return(retVal);
}

/**
*\brief This function enable the ECC.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
void GPMCNANDECCEnable(NandInfo_t *nandInfo)
{
    unsigned int baseAddr = nandInfo->hNandCtrlInfo->baseAddr;

    GPMCECCResultRegClear(baseAddr);
    GPMCECCEnable(baseAddr);
}

/**
*\brief This function disable the ECC.\n
*
* \param  nandCtrlInfo  : Pointer to structure containing controller info.\n
*
* \return none.\n
*
*/
void GPMCNANDECCDisable(NandInfo_t *nandInfo)
{
    unsigned int baseAddr = nandInfo->hNandCtrlInfo->baseAddr;

    GPMCECCDisable(baseAddr);
}

/**
*\brief  This Function does the ECC setting for write.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \return none.\n
*
*/
void GPMCNANDECCWriteSet(NandInfo_t *nandInfo)
{

    if(nandInfo->eccType == NAND_ECC_ALGO_HAMMING_1BIT)
    {
        GPMCHammingCodeWriteSet(nandInfo);
    }
    else
    {
        GPMCBCHWriteSet(nandInfo);
    }
}

/**
*\brief  This Function does the ECC setting for read.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \return none.\n
*
*/
void GPMCNANDECCReadSet(NandInfo_t *nandInfo)
{
    if(nandInfo->eccType == NAND_ECC_ALGO_HAMMING_1BIT)
    {
        GPMCHammingCodeReadSet(nandInfo);
    }
    else
    {
        GPMCBCHReadSet(nandInfo);
    }
}

/**
*\brief This function read the ecc data.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \param   eccRead      : Pointer where read ECC data has to store.\n
*
*
* \return none.\n
*
*/
void GPMCNANDECCCalculate(NandInfo_t *nandInfo, unsigned char *ptrEccData)
{
    if(nandInfo->eccType == NAND_ECC_ALGO_HAMMING_1BIT)
    {
        GPMCHammingCodeECCCalculate(nandInfo->hNandCtrlInfo->baseAddr,
                                    GPMC_ECC_RESULT_1, ptrEccData);
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT ||
            nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)
    {
        GPMCBCHECCCalculate(nandInfo, ptrEccData);
    }
}

/**
*\brief This function checks and corrects ECC errors.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \param   eccRead      : Pointer to the ECC data which is read from the spare 
*                         area.\n
*
* \param   data         : Pointer to the data, where if an ecc error need to 
*                         correct.\n
*
* \return ECC correction Status.\n
*    NAND_STATUS_PASSED                        : If no ecc errors.\n
*    NAND_STATUS_READ_ECC_ERROR_CORRECTED      : If error are corrected.\n
*    NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR  : If errors are uncorrectable.\n
*
*/
NandStatus_t GPMCNANDECCCheckAndCorrect(NandInfo_t *nandInfo,
                                        unsigned char *eccRead,
                                        unsigned char *data)
{
    NandStatus_t retVal;

    retVal = NAND_STATUS_PASSED;

    if(nandInfo->eccType == NAND_ECC_ALGO_HAMMING_1BIT)
    {
        retVal = GPMCHammingCodeECCCheckAndCorrect(nandInfo, eccRead, data);
    }
    else if(nandInfo->eccType == NAND_ECC_ALGO_BCH_4BIT ||
            nandInfo->eccType == NAND_ECC_ALGO_BCH_8BIT)

    {
        retVal = GPMCBCHECCCheckAndCorrect(nandInfo, eccRead, data);
    }

    return(retVal);
}

/**
*\brief This function transfer the data to NAND through the DMA depeding on
*       the direction.\n
*
* \param  nandInfo      : Pointer to structure containing controller and
*                         device information.\n
*
* \param   data         : Pointer to the data.\n 
*
* \param   len          : Transfer length.\n 
*
* \param   dir          : Direction of transfer.\n
*                         This can take one of the following values :\n
*                         NAND_EDMA_READ  -- Read from NAND.\n
*                         NAND_EDMA_WRITE -- Write to NAND.\n
*
* \return EDMA transfer status.\n
*
*         NAND_STATUS_PASSED             : if transfer is successful.\n
*         NAND_STATUS_READWRITE_DMA_FAIL : if transfer is failed.\n
*/
NandStatus_t GPMCNANDDMAXfer(NandInfo_t *nandInfo, unsigned char *data,
                             unsigned int len, NandDmaDir_t dir)
{
    unsigned int result;
    unsigned int baseAddr;
    NandStatus_t retVal;
    unsigned int timeOut;

    baseAddr = nandInfo->hNandCtrlInfo->baseAddr;
    result   = 0;
    retVal   = NAND_STATUS_PASSED;
    timeOut  = 0xFFFFF;

    /* Do the PREFETCH engine setting depeding on the direction */
    if( GPMCPrefetchEngineStatusGet(baseAddr) == 0 )
    {
        if(dir == NAND_DMA_DIR_READ)
        {
            GPMCPrefetchAccessModeSelect(baseAddr,
                                         GPMC_PREFETCH_ACCESSMODE_READ);
        }
        else
        {
            GPMCPrefetchAccessModeSelect(baseAddr,
                                         GPMC_PREFETCH_ACCESSMODE_WRITE);
        }
        GPMCPrefetchFifoThrldValSet(baseAddr, GPMC_NAND_PREFETCH_FIFO_THRLD);
        GPMCPrefetchTrnsCntValSet(baseAddr, len);
        GPMCPrefetchSyncModeConfig(baseAddr, GPMC_PREFETCH_ACCESSCS_AT_START);
        GPMCPrefetchAccessCycleOptConfig(baseAddr,
                                        GPMC_PREFETCH_OPTIMIZED_ACCESS_DISABLE);
        GPMCPrefetchCycleOptValSet(baseAddr, 0);
        GPMCPrefetchSyncTypeSelect(baseAddr, GPMC_PREFETCH_SYNCTYPE_DMA);
        (nandInfo->hNandDmaInfo->DMAXferSetup)(nandInfo, (unsigned char *)data, len, dir);
        GPMCPrefetchEngineEnable(baseAddr);
        GPMCPrefetchEngineStart(baseAddr);

        /* Wait till EDMA transfer to complete. */
        while((result == 0) && (--timeOut != 0 ))
        {
            result = (nandInfo->hNandDmaInfo->DMAXferStatusGet)(nandInfo);
        }
        if(timeOut == 0)
        {
            retVal = NAND_STATUS_READWRITE_DMA_FAIL;
        }
        /* EDMA trnasfer is complete, reset the prefetch ENGINE */
        GPMCPrefetchEngineStop(baseAddr);
        GPMCPrefetchEngineDisable(baseAddr);
      }
      else
      {
          retVal = NAND_STATUS_READWRITE_DMA_FAIL;
      }

      return (retVal);
}


/******************************************************************************
**                              END OF FILE
*******************************************************************************/
