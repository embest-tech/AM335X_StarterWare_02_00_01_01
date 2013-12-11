/**
 *  \file   nandDma.c
 *
 *  \brief  This file contains the NAND DMA related functions definitions.
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


#include "hw_types.h"
#include "interrupt.h"
#include "gpmc.h"
#include "edma.h"
#include "soc_AM335x.h"
#include "nandlib.h"
#include "nandDma.h"

/*******************************************************************************
*                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/*****************************************************************************/
/*
** Macros which defines the DMA related info like event Q num, TCC num, channel
** num etc.
*/
#define GPMC_EDMA_EVENT_Q_NUM                   (0)
#define GPMC_EDMA_TCC_NUM                       (52)
#define REGION_NUMBER                           (0)
#define GPMC_EDMA_CHANNEL_NUM                   (52)
#define GPMC_NAND_PREFETCH_FIFO_THRLD           (64)


/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

static volatile unsigned int xferCompFlag = 0;


/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  This function is used as a callback from EDMA3 Completion Handler.  *
*                                                                             *
* \param  tccNum    TCC number.\n                                             *
*                                                                             *
* \param  status    EDMA tranfer status.\n                                    *
*                                                                             *
* \param  appData   Application Data.\n                                       *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void callback(unsigned int tccNum, unsigned int status, void *appData)
{
    if(EDMA3_XFER_COMPLETE == status)
    {
        /* Transfer completed successfully                           */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM,
                             EDMA3_TRIG_MODE_EVENT);
        EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM);
        xferCompFlag = 1;
    }
}

/******************************************************************************
*                                                                             *
* \brief   Interrupt ISR for Channel controller error.                        *
*                                                                             *
* \note    This function first disables its own interrupt to make it non-     *
*          entrant.                                                           *
*                                                                             *
* \return  None.                                                              *
*                                                                             *
******************************************************************************/
static void Edma3CCErrorIsr()
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int evtqueNum = 0;
    volatile unsigned int isIPRH = 0;
    volatile unsigned int isIPR = 0;
    volatile unsigned int Cnt = 0u;
    volatile unsigned int index;


    pendingIrqs = 0u;
    index = 1u;

    isIPR  = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
    isIPRH = EDMA3IntrStatusHighGet(SOC_EDMA30CC_0_REGS);


    if((isIPR | isIPRH ) || (EDMA3QdmaGetErrIntrStatus(SOC_EDMA30CC_0_REGS) != 0)
        || (EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS) != 0))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time,
         * breaks when no pending interrupt is found
         */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT)
                    && (index != 0u))
        {
            index = 0u;

            if(isIPR)
            {
                   pendingIrqs = EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS);
            }
            else
            {
                   pendingIrqs = EDMA3ErrIntrHighStatusGet(SOC_EDMA30CC_0_REGS);
            }

            while (pendingIrqs)
            {
                   /*Process all the pending interrupts*/
                   if(TRUE == (pendingIrqs & 1u))
                   {
                        /* Write to EMCR to clear the corresponding EMR bits. */
                        /* Clear any SER */

                        if(isIPR)
                        {
                             EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, index);
                        }
                        else
                        {
                             EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, index + 32);
                        }
                   }
                   ++index;
                   pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = EDMA3QdmaGetErrIntrStatus(SOC_EDMA30CC_0_REGS);
            while (pendingIrqs)
            {
                /* Process all the pending interrupts */
                if(TRUE == (pendingIrqs & 1u))
                {
                    /* Here write to QEMCR to clear the corresponding QEMR bits */
                    /* Clear any QSER */
                    EDMA3QdmaClrMissEvt(SOC_EDMA30CC_0_REGS, index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;


            pendingIrqs = EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS);
            if (pendingIrqs != 0u)
            {
            /* Process all the pending CC error interrupts. */
            /* Queue threshold error for different event queues.*/
            for (evtqueNum = 0u; evtqueNum < SOC_EDMA3_NUM_EVQUE; evtqueNum++)
                {
                if((pendingIrqs & (1u << evtqueNum)) != 0u)
                {
                        /* Clear the error interrupt. */
                        EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS, (1u << evtqueNum));
                    }
                }

            /* Transfer completion code error. */
            if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
            {
                EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS,
                                      (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT));
            }
                ++index;
            }
            Cnt++;
        }
    }
}

/******************************************************************************
*                                                                             *
* \brief   ISR for successful transfer completion.                            *
*                                                                             *
* \note    This function first disables its own interrupt to make it non-     *
*          entrant.                                                           *
*                                                                             *
* \return  None.                                                              *
*                                                                             *
******************************************************************************/
static void Edma3CompletionIsr(void)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIPR = 0;
    volatile unsigned int isIPRH = 0;

    unsigned int indexl;
    unsigned int Cnt = 0;

    indexl = 1u;
    isIPR  = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
    isIPRH = EDMA3IntrStatusHighGet(SOC_EDMA30CC_0_REGS);

    if(isIPR | isIPRH)
    {
        while ((Cnt < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (indexl != 0u))
        {
              indexl = 0u;

              if(isIPR)
              {
                   pendingIrqs = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
              }
              else
              {
                   pendingIrqs = EDMA3IntrStatusHighGet(SOC_EDMA30CC_0_REGS);
              }
              while (pendingIrqs)
              {
                   if(TRUE == (pendingIrqs & 1u))
                   {
                        /**
                         * If the user has not given any callback function
                         * while requesting the TCC, its TCC specific bit
                         * in the IPR register will NOT be cleared.
                         */
                         if(isIPR)
                         {
                              /* Here write to ICR to clear the corresponding 
                                 IPR bits */
                              EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, indexl);
                              callback(indexl, EDMA3_XFER_COMPLETE, NULL);
                         }
                         else
                         {
                              /* Here write to ICR to clear the corresponding 
                                 IPR bits */
                              EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, indexl + 32);
                              callback(indexl, EDMA3_XFER_COMPLETE, NULL);
                        }
                   }
              ++indexl;
              pendingIrqs >>= 1u;
             }
             Cnt++;
         }
     }
}

/******************************************************************************
*                                                                             *
* \brief  This function configs the DMA for read.\n                           *
*                                                                             *
* \param  csBaseAddr    Chipselect base address                               *
*                                                                             *
* \param  data          Data pointer for read data                            *
*                                                                             *
* \param  len           Byte count to read                                    *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static unsigned int GPMCNANDRxDmaConfig(unsigned int csBaseAddr, unsigned char *data,
                                unsigned int len)
{
    EDMA3CCPaRAMEntry paramSet;
    /* Fill the PaRAM Set with transfer specific information    */
    paramSet.aCnt       = GPMC_NAND_PREFETCH_FIFO_THRLD;
    paramSet.bCnt       = (len/GPMC_NAND_PREFETCH_FIFO_THRLD);
    paramSet.bCntReload = 0u;
    paramSet.cCnt       = 1u;
    paramSet.destAddr   = (unsigned int )(data);
    paramSet.destBIdx   = GPMC_NAND_PREFETCH_FIFO_THRLD;
    paramSet.destCIdx   = 1;
    paramSet.linkAddr   = 0xFFFFu;
    paramSet.srcAddr    = csBaseAddr;
    paramSet.srcBIdx    = 0;
    paramSet.srcCIdx    = 0;
    paramSet.opt = 0x00000000u;
    /* Src & Dest are in INCR modes */
    paramSet.opt &= 0xFFFFFFFCu;
    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= (( GPMC_EDMA_TCC_NUM << EDMA3CC_OPT_TCC_SHIFT)
                       & EDMA3CC_OPT_TCC);
    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);
    /* Now, write the PaRAM Set.                                */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM, &paramSet);
    /* Now enable the transfer                                  */
    return EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM,
                        EDMA3_TRIG_MODE_EVENT);
}

/******************************************************************************
*                                                                             *
* \brief  This function configs the DMA for write.\n                          *
*                                                                             *
* \param  csBaseAddr    Chipselect base address                               *
*                                                                             *
* \param  data          Data pointer for write data                           *
*                                                                             *
* \param  len           Byte count to write                                   *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static unsigned int GPMCNANDTxDmaConfig(unsigned int csBaseAddr,
                                unsigned char *data, unsigned int len)
{
    EDMA3CCPaRAMEntry paramSet;
    /* Fill the PaRAM Set with transfer specific information    */
    paramSet.aCnt       = GPMC_NAND_PREFETCH_FIFO_THRLD;
    paramSet.bCnt       = (len/GPMC_NAND_PREFETCH_FIFO_THRLD);
    paramSet.bCntReload = 0u;
    paramSet.cCnt       = 1u;
    //paramSet.destAddr   = nandInfo->nandCtrlInfo.dataRegAddr;
    paramSet.destAddr   = csBaseAddr;
    paramSet.destBIdx   = 0;
    paramSet.destCIdx   = 0;
    paramSet.linkAddr   = 0xFFFFu;
    paramSet.srcAddr    = (unsigned int )(data);
    paramSet.srcBIdx    = GPMC_NAND_PREFETCH_FIFO_THRLD;
    paramSet.srcCIdx    = 1;
    paramSet.opt = 0x00000000u;
    /* Src & Dest are in INCR modes */
    paramSet.opt &= 0xFFFFFFFCu;
    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= (( GPMC_EDMA_TCC_NUM << EDMA3CC_OPT_TCC_SHIFT)
                       & EDMA3CC_OPT_TCC);
    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);
    /* Now, write the PaRAM Set.                                */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM, &paramSet);
    /* Now enable the transfer                                  */
    return EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, GPMC_EDMA_CHANNEL_NUM,
                        EDMA3_TRIG_MODE_EVENT);
}

/******************************************************************************
*                                                                             *
* \brief  This function configures the INTC to receive EDMA3 interrupts.      *
*                                                                             *
******************************************************************************/
static void EDMA3INTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3CompletionIsr);

    /* Setting the priority for EDMA3CC0 completion interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3CCErrorIsr);

    /* Setting the priority for EDMA3CC0 Error interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 completion interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /* Enabling the EDMA3CC0 Error interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}


/******************************************************************************
**                       GLOBAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  This function setup the DMA for read/write                          *
*                                                                             *
* \param  nandInfo      Pointer to the strucure containing controller and     *
*                       device info.                                          *
*                                                                             *
* \param  data          Data pointer                                          *
*                                                                             *
* \param  dir           Direction which specifies the direction of transfer   *
*                       i.e read/write.                                       *
*                                                                             *
* \param  len           Byte count                                            *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void GPMCNANDXferSetup(NandInfo_t *nandInfo, unsigned char *data,
                       unsigned int len, NandDmaDir_t dir)
{
    unsigned int currCS = nandInfo->hNandCtrlInfo->currChipSelect;
    unsigned int csBase = nandInfo->hNandCtrlInfo->chipSelectBaseAddr[currCS];

    xferCompFlag  = 0;
    if(dir == NAND_DMA_DIR_READ)
    {
        GPMCNANDRxDmaConfig(csBase, data, len);
    }
    else
    {
        GPMCNANDTxDmaConfig(csBase, data, len);
    }
}

/******************************************************************************
*                                                                             *
* \brief  This function returns the status of the DMA transfer.               *
*                                                                             *
* \param  none.                                                               *
*                                                                             *
* \return DMA Xfer status.                                                    *
*                                                                             *
******************************************************************************/
unsigned int GPMCNANDXferStatusGet()
{
    return(xferCompFlag);
}

/******************************************************************************
*                                                                             *
* \brief  This function Initializs the DMA for read/write.                    *
*                                                                             *
* \note unused argument is added for future use.                              *
*                                                                             *
******************************************************************************/
void GPMCNANDEdmaInit(void *unused)
{
    /* Initialization of EDMA3 */
    EDMA3Init(SOC_EDMA30CC_0_REGS, GPMC_EDMA_EVENT_Q_NUM);
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();
    /* Configuring the INTC to receive EDMA3 interrupts. */
    EDMA3INTCConfigure();
    /* Request DMA Channel and TCC for NAND data movement. */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        GPMC_EDMA_CHANNEL_NUM, GPMC_EDMA_TCC_NUM,
                        GPMC_EDMA_EVENT_Q_NUM);
}

/******************************************************************************
**                              END OF FILE
*******************************************************************************/

