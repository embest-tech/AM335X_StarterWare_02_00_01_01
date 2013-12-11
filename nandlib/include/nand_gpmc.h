/**
 *  \file   nand_gpmc.h
 *
 *  \brief  This file contains the NAND GPMC controller specific struct 
 *          definition, macros and function prototypes.
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


#ifndef __NAND_GPMC_H_
#define __NAND_GPMC_H__

#include "nandlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*                           STRUCTURES DECLARION
*******************************************************************************/

/*****************************************************************************/
/*
** Contains the timing params and base address info for the GPMC NAND access.
*/
typedef struct _GPMC_NAND_TIMING_INFO_ 
{

    unsigned int CSWrOffTime;
    unsigned int CSRdOffTime;
    unsigned int CSExtDelayFlag;
    unsigned int CSOnTime;

    unsigned int ADVAADMuxWrOffTime;
    unsigned int ADVAADMuxRdOffTime;
    unsigned int ADVWrOffTime;
    unsigned int ADVRdOffTime;
    unsigned int ADVExtDelayFlag;
    unsigned int ADVAADMuxOnTime;
    unsigned int ADVOnTime;

    unsigned int WEOffTime;
    unsigned int WEExtDelayFlag;
    unsigned int WEOnTime;
    unsigned int OEAADMuxOffTime;
    unsigned int OEOffTime;
    unsigned int OEExtDelayFlag;
    unsigned int OEAADMuxOnTime;
    unsigned int OEOnTime;

    unsigned int rdCycleTime;
    unsigned int wrCycleTime;
    unsigned int rdAccessTime;
    unsigned int pageBurstAccessTime;

    unsigned int cycle2CycleDelay;
    unsigned int cycle2CycleDelaySameCSCfg;
    unsigned int cycle2CycleDelayDiffCSCfg;
    unsigned int busTAtime;

    unsigned int wrAccessTime;
    unsigned int wrDataOnADMux;


}GPMCNANDTimingInfo_t;

/*******************************************************************************
*                           FUNCTION PROTOTYPE DECLARATION
*******************************************************************************/

/* Generic EMIFA/NAND APIs */
extern NandStatus_t     GPMCNANDInit(NandInfo_t *nandInfo);
extern unsigned int     GPMCNANDWaitPinStatusGet(NandInfo_t *nandInfo);
extern unsigned int     GPMCNANDWriteBufReady(NandInfo_t *nandInfo);
extern NandStatus_t     GPMCNANDDMAXfer( NandInfo_t *nandInfo,
                                          unsigned char *data,
                                          unsigned int len, 
                                          NandDmaDir_t dir);


/* EMIFA/NAND ECC related APIs */
extern NandStatus_t     GPMCNANDECCInit(NandInfo_t *nandInfo);
extern void             GPMCNANDECCEnable(NandInfo_t *nandInfo);
extern void             GPMCNANDECCDisable(NandInfo_t *nandInfo);
extern void             GPMCNANDECCReadSet(NandInfo_t *nandInfo);
extern void             GPMCNANDECCWriteSet(NandInfo_t *nandInfo);
extern void             GPMCNANDECCCalculate( NandInfo_t *nandInfo, 
                                              unsigned char *ptrEccData);
extern NandStatus_t     GPMCNANDECCCheckAndCorrect( NandInfo_t *nandInfo,
                                                    unsigned char *eccRead,
                                                    unsigned char *data);


#ifdef __cplusplus
}
#endif
#endif
