/**
 * \file   edmaTest.c
 *
 * \brief   This is a sample application file which invokes APIs from EDMA
 *          device abstraction layer to perform configuration, transmission
 *          and reception operations.
 *
 *          Application Configuration:
 *
 *              Modules Used:
 *                  DMA
 *                  UART0
 *
 *              Configurable Parameters:
 *                  None
 *
 *              Hard-coded configuration of other parameters:
 *                  Console interface - Uart interface is default configuration.
 *                                      For semihosting set ConsoleUtilsSetType
 *                                      to CONSOLE_DEBUGGER and set compile time
 *                                      macro CONSOLE as SEMIHOSTING.
 *
 *
 *          Application UseCase:
 *              The application demonstrates DMA transfer of data from a buffer
 *              populated with data to another buffer. DMA transfer can be done
 *              either through EDMA or QDMA based on the compile definition of
 *              macro CH_TYPE_DMA. After the transfer destination buffer is
 *              compared with source buffer and to determined pass/fail status.
 *
 *          Running example:
 *              On execution application will perform data transfer from source
 *              buffer to destination buffer either through EDMA or QDMA mode.
 *              After the transfer destination buffer is compared with source
 *              and status is displayed on console.
 *
 *          Limitations:
 *              None.
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

#include "interrupt.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "uartStdio.h"
#include "edma.h"
#include "consoleUtils.h"

/****************************************************************************/
/*                      MACRO DEFINITIONS                                   */
/****************************************************************************/
#define EDMAAPP_MAX_ACOUNT        (10u) /* MAX ACOUNT */
#define EDMAAPP_MAX_BCOUNT        (1u)  /* MAX BCOUNT */
#define EDMAAPP_MAX_CCOUNT        (1u)  /* MAX CCOUNT */
#define EDMAAPP_MAX_BUFFER_SIZE   (EDMAAPP_MAX_ACOUNT * EDMAAPP_MAX_BCOUNT * \
                                   EDMAAPP_MAX_CCOUNT)

#define EDMAAPP_IRQ_STATUS_XFER_INPROG     (0u)
#define EDMAAPP_IRQ_STATUS_XFER_COMP       (1u)
#define EDMAAPP_IRQ_STATUS_DMA_EVT_MISS    (-1)
#define EDMAAPP_IRQ_STATUS_QDMA_EVT_MISS   (-2)
#define EDMAAPP_EDMACC_BASE_ADDRESS        SOC_EDMA30CC_0_REGS

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES.                          */
/****************************************************************************/
static void _EDMAAppRegisterEdma3Interrupts(void);
static void _EDMAAppEdma3ccComplIsr(void);
static void _EDMAAppEdma3ccErrIsr(void);
void (*EDMAAppCallbackFxn[EDMA3_NUM_TCC])(unsigned int status);

#ifdef CH_TYPE_DMA
void EDMAAppEDMA3Test();
#else
void EDMAAppQDMA3Test();
#endif

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
volatile int IrqRaised;

#ifdef __TMS470__
#pragma DATA_ALIGN(SrcBuff, SOC_CACHELINE_SIZE_MAX);
#pragma DATA_ALIGN(DstBuff, SOC_CACHELINE_SIZE_MAX);
volatile char SrcBuff[EDMAAPP_MAX_BUFFER_SIZE];
volatile char DstBuff[EDMAAPP_MAX_BUFFER_SIZE];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = SOC_CACHELINE_SIZE_MAX
volatile char SrcBuff[EDMAAPP_MAX_BUFFER_SIZE];
volatile char DstBuff[EDMAAPP_MAX_BUFFER_SIZE];
#elif defined(gcc)
volatile char SrcBuff[EDMAAPP_MAX_BUFFER_SIZE]
                               __attribute__((aligned(SOC_CACHELINE_SIZE_MAX)));
volatile char DstBuff[EDMAAPP_MAX_BUFFER_SIZE]
                               __attribute__((aligned(SOC_CACHELINE_SIZE_MAX)));
#else
#error "Unsupported Compiler. \r\n"
#endif

/*
** Data parameters required by application
*/
#ifdef CH_TYPE_DMA
#define EDMAAPP_DMA_CH_TYPE     EDMA3_CHANNEL_TYPE_DMA
#define EDMAAPP_DMA_CH_NUM      (63u)
#define EDMAAPP_DMA_TCC_NUM     (63u)
#define EDMAAPP_DMA_SYNC_TYPE   EDMA3_SYNC_A
#define EDMAAPP_DMA_TRIG_MODE   EDMA3_TRIG_MODE_MANUAL
#define EDMAAPP_DMA_EVTQ        (0u)
#else
#define EDMAAPP_DMA_CH_TYPE     EDMA3_CHANNEL_TYPE_QDMA
#define EDMAAPP_DMA_CH_NUM      (0u)
#define EDMAAPP_DMA_TCC_NUM     (0u)
#define EDMAAPP_DMA_SYNC_TYPE   EDMA3_SYNC_A
#define EDMAAPP_DMA_TRIG_MODE   EDMA3_TRIG_MODE_QDMA
#define EDMAAPP_DMA_EVTQ        (0u)
#endif

/****************************************************************************/
/*                      LOCAL FUNCTION DEFINITIONS                          */
/****************************************************************************/

/* Callback function */
void EDMAAppCallback(unsigned int status)
{
    if(EDMA3_XFER_COMPLETE == status)
    {
        /* Transfer completed successfully */
        IrqRaised = EDMAAPP_IRQ_STATUS_XFER_COMP;
    }
    else if(EDMA3_CC_DMA_EVT_MISS == status)
    {
        /* Transfer resulted in DMA event miss error. */
        IrqRaised = EDMAAPP_IRQ_STATUS_DMA_EVT_MISS;
    }
    else if(EDMA3_CC_QDMA_EVT_MISS == status)
    {
        /* Transfer resulted in QDMA event miss error. */
        IrqRaised = EDMAAPP_IRQ_STATUS_QDMA_EVT_MISS;
    }
}

int main(void)
{
    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Configure EDMA module clock */
    EDMAModuleClkConfig();

    /* Initialize EDMA */
    EDMA3Init(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_EVTQ);

    _EDMAAppRegisterEdma3Interrupts();

#ifdef CH_TYPE_DMA
    EDMAAppEDMA3Test();
#else
    EDMAAppQDMA3Test();
#endif

    EDMA3Deinit(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_EVTQ);

    return 0;
}

#ifdef CH_TYPE_DMA
void EDMAAppEDMA3Test()
{
    volatile unsigned int index = 0u;
    volatile unsigned int count = 0u;
    EDMA3CCPaRAMEntry paramSet;
    unsigned char data = 0u;
    volatile unsigned int retVal = 0u;
    unsigned int isTestPassed = false;
    unsigned int numEnabled = 0u;
    unsigned int aCount = EDMAAPP_MAX_ACOUNT;
    unsigned int bCount = EDMAAPP_MAX_BCOUNT;
    unsigned int cCount = EDMAAPP_MAX_CCOUNT;

    /* Initalize source and destination buffers */
    for (count = 0u; count < (aCount * bCount * cCount); count++)
    {
        SrcBuff[count] = data++;
        /*
        ** No need to initialize the destination buffer as it is
        ** being invalidated.
        */
    }

    /* Request DMA channel and TCC */
    retVal = EDMA3RequestChannel(EDMAAPP_EDMACC_BASE_ADDRESS,
                                 EDMAAPP_DMA_CH_TYPE, EDMAAPP_DMA_CH_NUM,
                                 EDMAAPP_DMA_TCC_NUM, EDMAAPP_DMA_EVTQ);

    /* Registering Callback Function */
    EDMAAppCallbackFxn[EDMAAPP_DMA_TCC_NUM] = &EDMAAppCallback;

    if(TRUE == retVal)
    {
        /* Fill the PaRAM Set with transfer specific information */
        paramSet.srcAddr = (unsigned int)(SrcBuff);
        paramSet.destAddr = (unsigned int)(DstBuff);

        paramSet.aCnt = (unsigned short)aCount;
        paramSet.bCnt = (unsigned short)bCount;
        paramSet.cCnt = (unsigned short)cCount;

        /* Setting up the SRC/DES Index */
        paramSet.srcBIdx = (short)aCount;
        paramSet.destBIdx = (short)aCount;

        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            /* A Sync Transfer Mode */
            paramSet.srcCIdx = (short)aCount;
            paramSet.destCIdx = (short)aCount;
        }
        else
        {
            /* AB Sync Transfer Mode */
            paramSet.srcCIdx = ((short)aCount * (short)bCount);
            paramSet.destCIdx = ((short)aCount * (short)bCount);
        }

        /* Configure the paramset with NULL link */
        paramSet.linkAddr = (unsigned short)0xFFFFu;

        paramSet.bCntReload = (unsigned short)0u;
        paramSet.opt = 0u;

        /* Src & Dest are in INCR modes */
        paramSet.opt &= ~(EDMA3CC_OPT_SAM | EDMA3CC_OPT_DAM);

        /* Program the TCC */
        paramSet.opt |= ((EDMAAPP_DMA_TCC_NUM << EDMA3CC_OPT_TCC_SHIFT)
                         & EDMA3CC_OPT_TCC);

        /* Enable Intermediate & Final transfer completion interrupt */
        paramSet.opt |= (1u << EDMA3CC_OPT_ITCINTEN_SHIFT);
        paramSet.opt |= (1u << EDMA3CC_OPT_TCINTEN_SHIFT);

        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            paramSet.opt &= ~EDMA3CC_OPT_SYNCDIM;
        }
        else
        {
            /* AB Sync Transfer Mode */
            paramSet.opt |= (1u << EDMA3CC_OPT_SYNCDIM_SHIFT);
        }

        /* Now, write the PaRAM Set. */
        EDMA3SetPaRAM(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_CH_NUM,
                      &paramSet);

        EDMA3GetPaRAM(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_CH_NUM,
                      &paramSet);
    }

    /*
    ** Since the transfer is going to happen in Manual mode of EDMA3
    ** operation, we have to 'Enable the Transfer' multiple times.
    ** Number of times depends upon the Mode (A/AB Sync)
    ** and the different counts.
    */
    if(TRUE == retVal)
    {
        /* Need to activate next param */
        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            numEnabled = bCount * cCount;
        }
        else
        {
            /* AB Sync Transfer Mode */
            numEnabled = cCount;
        }

        for(index = 0u; index < numEnabled; index++)
        {
            IrqRaised = EDMAAPP_IRQ_STATUS_XFER_INPROG;

            /*
            ** Now enable the transfer as many times as calculated above.
            */
            retVal = EDMA3EnableTransfer(EDMAAPP_EDMACC_BASE_ADDRESS,
                                         EDMAAPP_DMA_CH_NUM,
                                         EDMAAPP_DMA_TRIG_MODE);

            /* Wait for the Completion ISR. */
            while(EDMAAPP_IRQ_STATUS_XFER_INPROG == IrqRaised)
            {
                /*
                ** Wait for the Completion ISR on Master Channel.
                ** You can insert your code here to do something
                ** meaningful.
                */
            }

            /* Check the status of the completed transfer */
            if(IrqRaised < (int)EDMAAPP_IRQ_STATUS_XFER_INPROG)
            {
                /* Some error occured, break from the FOR loop. */
                ConsoleUtilsPrintf("\r\nEDMA3Test: Event Miss Occured!!!\r\n");

                /* Clear the error bits first */
                EDMA3ClearErrorBits(EDMAAPP_EDMACC_BASE_ADDRESS,
                                    EDMAAPP_DMA_CH_NUM, EDMAAPP_DMA_EVTQ);
                break;
            }
        }
    }

    /* Match the Source and Destination Buffers. */
    if(TRUE == retVal)
    {
        for(index = 0u; index < (aCount * bCount * cCount); index++)
        {
            if(SrcBuff[index] != DstBuff[index])
            {
                isTestPassed = false;
                ConsoleUtilsPrintf("EDMA3Test: Data write-read matching FAILED.\r\n");
                ConsoleUtilsPrintf("The mismatch happened at index : %d\r\n",
                                   ((int)index + 1u));
                break;
            }
        }

        if(index == (aCount * bCount * cCount))
        {
            isTestPassed = true;
            ConsoleUtilsPrintf("EDMA3Test: Data write-read matching PASSED.\r\n");
        }

        /* Free the previously allocated channel. */
        retVal = EDMA3FreeChannel(EDMAAPP_EDMACC_BASE_ADDRESS,
                                  EDMAAPP_DMA_CH_TYPE, EDMAAPP_DMA_CH_NUM,
                                  EDMAAPP_DMA_TRIG_MODE, EDMAAPP_DMA_TCC_NUM,
                                  EDMAAPP_DMA_EVTQ);

        /* Unregister Callback Function */
        EDMAAppCallbackFxn[EDMAAPP_DMA_TCC_NUM] = NULL;

        if(TRUE != retVal)
        {
            ConsoleUtilsPrintf("EDMA3Test: EDMA3_DRV_freeChannel() FAILED.\r\n");
        }
    }

    if(true == isTestPassed)
    {
        ConsoleUtilsPrintf("EDMA3Test PASSED.\r\n");
    }
    else
    {
        ConsoleUtilsPrintf("EDMA3Test FAILED\r\n");
    }
}

#else

/*
** QDMA Application
*/
void EDMAAppQDMA3Test()
{
    volatile unsigned int index = 0u;
    volatile unsigned int count = 0u;
    EDMA3CCPaRAMEntry paramSet;
    unsigned char data = 0u;
    unsigned int retVal = 0u;
    unsigned int isTestPassed = false;
    unsigned int numEnabled = 0u;
    unsigned int aCount = EDMAAPP_MAX_ACOUNT;
    unsigned int bCount = EDMAAPP_MAX_BCOUNT;
    unsigned int cCount = EDMAAPP_MAX_CCOUNT;
    unsigned int DstBuffAddr = 0u;
    volatile unsigned int opt = 0u;
    unsigned int paramId = 32u;

    /* Initalize source and destination buffers */
    for(count = 0u; count < (aCount * bCount * cCount); count++)
    {
        SrcBuff[count] = data++;
        /*
        ** No need to initialize the destination buffer
        ** as it is being invalidated.
        */
    }

    /* Request DMA channel and TCC */
    retVal = EDMA3RequestChannel(EDMAAPP_EDMACC_BASE_ADDRESS,
                                 EDMAAPP_DMA_CH_TYPE, EDMAAPP_DMA_CH_NUM,
                                 EDMAAPP_DMA_TCC_NUM, EDMAAPP_DMA_EVTQ);

    EDMA3MapQdmaChToPaRAM(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_CH_NUM,
                          &paramId);

    EDMA3SetQdmaTrigWord(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_CH_NUM,
                         EDMA3CC_PARAM_ENTRY_DST);

    /* Registering Callback Function */
    EDMAAppCallbackFxn[EDMAAPP_DMA_TCC_NUM] = &EDMAAppCallback;

    if(TRUE == retVal)
    {
        /* Fill the PaRAM Set with transfer specific information */
        paramSet.srcAddr = (unsigned int)(SrcBuff);
        paramSet.destAddr = (unsigned int)(DstBuff);
        paramSet.aCnt = (unsigned short)aCount;
        paramSet.bCnt = (unsigned short)bCount;
        paramSet.cCnt = (unsigned short)cCount;

        /* Setting up the SRC/DES Index */
        paramSet.srcBIdx = (short)aCount;
        paramSet.destBIdx = (short)aCount;
        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            /* A Sync Transfer Mode */
            paramSet.srcCIdx = (short)aCount;
            paramSet.destCIdx = (short)aCount;
        }
        else
        {
            /* AB Sync Transfer Mode */
            paramSet.srcCIdx = ((short)aCount * (short)bCount);
            paramSet.destCIdx = ((short)aCount * (short)bCount);
        }

        /* Configure the paramset with NULL link */
        paramSet.linkAddr = (unsigned short)0xFFFFu;

        paramSet.bCntReload = (unsigned short)0u;
        paramSet.opt = 0u;

        /* Src & Dest are in INCR modes */
        paramSet.opt &= ~(EDMA3CC_OPT_SAM | EDMA3CC_OPT_DAM);

        /* Program the TCC */
        paramSet.opt |= ((EDMAAPP_DMA_TCC_NUM << EDMA3CC_OPT_TCC_SHIFT)
                         & EDMA3CC_OPT_TCC);

        /* Enable Intermediate & Final transfer completion interrupt */
        paramSet.opt |= (1u << EDMA3CC_OPT_ITCINTEN_SHIFT);
        paramSet.opt |= (1u << EDMA3CC_OPT_TCINTEN_SHIFT);

        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            paramSet.opt &= ~EDMA3CC_OPT_SYNCDIM;
        }
        else
        {
            /* AB Sync Transfer Mode */
            paramSet.opt |= (1u << EDMA3CC_OPT_SYNCDIM_SHIFT);
        }

        opt = paramSet.opt;

        /* Now, write the PaRAM Set. */
        EDMA3QdmaSetPaRAM(EDMAAPP_EDMACC_BASE_ADDRESS, EDMAAPP_DMA_CH_NUM,
                          paramId, &paramSet);
    }

    retVal = EDMA3EnableTransfer(EDMAAPP_EDMACC_BASE_ADDRESS,
                                 EDMAAPP_DMA_CH_NUM, EDMAAPP_DMA_TRIG_MODE);

    /*
    ** Since the transfer is going to happen in Manual mode of EDMA3
    ** operation, we have to 'Enable the Transfer' multiple times.
    ** Number of times depends upon the Mode (A/AB Sync)
    ** and the different counts.
    */
    if(TRUE == retVal)
    {
        /* Need to activate next param */
        if(EDMA3_SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            numEnabled = bCount * cCount;
        }
        else
        {
            /* AB Sync Transfer Mode */
            numEnabled = cCount;
        }

        for(index = 0u; index < numEnabled; index++)
        {
            IrqRaised = EDMAAPP_IRQ_STATUS_XFER_INPROG;

            if(index == (numEnabled - 1u))
            {
                /**
                ** Since OPT.STATIC field should be SET for isolated QDMA
                ** transfers or for the final transfer in a linked list of QDMA
                ** transfers, do the needful for the last request.
                */
                opt |= EDMA3CC_OPT_STATIC;
                EDMA3QdmaSetPaRAMEntry(EDMAAPP_EDMACC_BASE_ADDRESS, paramId,
                                       EDMA3CC_PARAM_ENTRY_OPT, opt);
            }

            opt |= EDMA3CC_OPT_FWID_8BIT;
            EDMA3QdmaSetPaRAMEntry(EDMAAPP_EDMACC_BASE_ADDRESS, paramId,
                                   EDMA3CC_PARAM_ENTRY_OPT, opt);

            /*
            ** Now trigger the QDMA channel by writing to the Trigger
            ** Word which is set as Destination Address.
            */
            DstBuffAddr = (unsigned int)EDMA3QdmaGetPaRAMEntry(
                 EDMAAPP_EDMACC_BASE_ADDRESS, paramId, EDMA3CC_PARAM_ENTRY_DST);

            EDMA3QdmaSetPaRAMEntry(EDMAAPP_EDMACC_BASE_ADDRESS, paramId,
                                   EDMA3CC_PARAM_ENTRY_DST, DstBuffAddr);

            /* Wait for the Completion ISR. */
            while(EDMAAPP_IRQ_STATUS_XFER_INPROG == IrqRaised)
            {
                /*
                ** Wait for the Completion ISR on Master Channel.
                ** You can insert your code here to do something
                ** meaningful.
                */
            }

            /* Check the status of the completed transfer */
            if(IrqRaised < (int)EDMAAPP_IRQ_STATUS_XFER_INPROG)
            {
                /* Some error occured, break from the FOR loop. */
                ConsoleUtilsPrintf("\r\nQDMA3Test: Event Miss Occured!!!\r\n");

                /* Clear the error bits first */
                EDMA3ClearErrorBits(EDMAAPP_EDMACC_BASE_ADDRESS,
                                    EDMAAPP_DMA_CH_NUM, EDMAAPP_DMA_EVTQ);
                break;
            }
        }
    }

    /* Match the Source and Destination Buffers. */
    if(TRUE == retVal)
    {
        for(index = 0u; index < (aCount * bCount * cCount); index++)
        {
            if(SrcBuff[index] != DstBuff[index])
            {
                isTestPassed = false;
                ConsoleUtilsPrintf("QDMA3Test: Data write-read matching FAILED.\r\n");
                ConsoleUtilsPrintf("The mismatch happened at index : %d\r\n",
                                   ((int)index + 1u));
                break;
            }
        }

        if(index == (aCount * bCount * cCount))
        {
            isTestPassed = true;
            ConsoleUtilsPrintf("QDMA3Test: Data write-read matching PASSED.\r\n");
        }

        /* Free the previously allocated channel. */
        retVal = EDMA3FreeChannel(EDMAAPP_EDMACC_BASE_ADDRESS,
                                  EDMAAPP_DMA_CH_TYPE, EDMAAPP_DMA_CH_NUM,
                                  EDMAAPP_DMA_TRIG_MODE, EDMAAPP_DMA_TCC_NUM,
                                  EDMAAPP_DMA_EVTQ);

        /* Unregister Callback Function */
        EDMAAppCallbackFxn[EDMAAPP_DMA_TCC_NUM] = NULL;

        if(TRUE != retVal)
        {
            ConsoleUtilsPrintf("QDMA3Test: EDMA3_DRV_freeChannel() FAILED.\r\n");
        }
    }

    if(true == isTestPassed)
    {
        ConsoleUtilsPrintf("QDMA3Test PASSED.\r\n");
    }
    else
    {
        ConsoleUtilsPrintf("QDMA3Test FAILED.\r\n");
    }
}
#endif

void _EDMAAppRegisterEdma3Interrupts()
{
    /* Enable IRQ in CPSR. */
    IntMasterIRQEnable();

    /* Intialize ARM interrupt controller */
    IntAINTCInit();

    /* Register Interrupts Here */

    /******************** Completion Interrupt ********************************/

    /* Registers Edma3ComplHandler0 Isr in Interrupt Vector Table of AINTC. */
    IntRegister(SYS_INT_EDMACOMPINT , _EDMAAppEdma3ccComplIsr);

    /* Set priority for system interrupt in AINTC */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0u, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the EDMA CC0 system interrupt in AINTC.*/
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /********************** CC Error Interrupt ********************************/

    /*
    ** Registers the EDMA3_0 Channel Controller 0 Error Interrupt Isr in the
    ** Interrupt Vector Table of AINTC.
    */
    IntRegister(SYS_INT_EDMAERRINT , _EDMAAppEdma3ccErrIsr);

    /* Set priority for system interrupt in AINTC */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0u, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the EDMA CCERR system interrupt AINTC.*/
    IntSystemEnable(SYS_INT_EDMAERRINT);
}

/*
** ISR for successful transfer completion.
**
** Note: This function first disables its own interrupt to make it non-entrant.
*/
void _EDMAAppEdma3ccComplIsr()
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIntrPending = 0u;
    volatile unsigned int isHighIntrPending = 0u;
    unsigned int index;
    unsigned int count = 0u;

    index = 1u;
    isIntrPending  = EDMA3GetIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
    isHighIntrPending = EDMA3IntrStatusHighGet(EDMAAPP_EDMACC_BASE_ADDRESS);

    if(isIntrPending | isHighIntrPending)
    {
        while ((count < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (index != 0u))
        {
            index = 0u;

            if(isIntrPending)
            {
                pendingIrqs = EDMA3GetIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
            }
            else
            {
                pendingIrqs =
                            EDMA3IntrStatusHighGet(EDMAAPP_EDMACC_BASE_ADDRESS);
            }
            while(pendingIrqs)
            {
                if(TRUE == (pendingIrqs & 1u))
                {
                    /*
                    **
                    ** If the user has not given any Callback function
                    ** while requesting the TCC, its TCC specific bit
                    ** in the IPR register will NOT be cleared.
                    */
                    if(isIntrPending)
                    {
                        /*
                        ** Here write to ICR to clear the corresponding
                        ** IPR bits
                        */
                        EDMA3ClrIntr(EDMAAPP_EDMACC_BASE_ADDRESS, index);
                        (*EDMAAppCallbackFxn[index])(EDMA3_XFER_COMPLETE);
                    }
                    else
                    {
                        /*
                        ** Here write to ICR to clear the corresponding
                        ** IPR bits
                        */
                        EDMA3ClrIntr(EDMAAPP_EDMACC_BASE_ADDRESS, index + 32u);
                        (*EDMAAppCallbackFxn[index + 32u])(EDMA3_XFER_COMPLETE);
                    }

                }
                ++index;
                pendingIrqs >>= 1u;
            }
            count++;
        }
    }
}

/*
** Interrupt ISR for Channel controller error.
**
** Note: This function first disables its own interrupt to make it non-entrant.
*/
void _EDMAAppEdma3ccErrIsr()
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int evtQueNum = 0u; /* Event Queue Num */
    volatile unsigned int isHighIntrPending = 0u;
    volatile unsigned int isIntrPending = 0u;
    volatile unsigned int count = 0u;
    volatile unsigned int index;

    pendingIrqs = 0u;
    index = 1u;

    isIntrPending  = EDMA3GetIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
    isHighIntrPending = EDMA3IntrStatusHighGet(EDMAAPP_EDMACC_BASE_ADDRESS);

    if((isIntrPending | isHighIntrPending ) ||
       (EDMA3QdmaGetErrIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS) != 0u)
        || (EDMA3GetCCErrStatus(EDMAAPP_EDMACC_BASE_ADDRESS) != 0u))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time,
        ** breaks when no pending interrupt is found
        */
        while ((count < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;

            if(isIntrPending)
            {
                pendingIrqs =
                             EDMA3GetErrIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
            }
            else
            {
                pendingIrqs =
                         EDMA3ErrIntrHighStatusGet(EDMAAPP_EDMACC_BASE_ADDRESS);
            }

            while(pendingIrqs)
            {
                /* Process all the pending interrupts */
                if(TRUE == (pendingIrqs & 1u))
                {
                    /*
                    ** Write to EMCR to clear the corresponding EMR bits.
                    ** Clear any SER
                    */
                    if(isIntrPending)
                    {
                        EDMA3ClrMissEvt(EDMAAPP_EDMACC_BASE_ADDRESS, index);
                    }
                    else
                    {
                        EDMA3ClrMissEvt(EDMAAPP_EDMACC_BASE_ADDRESS,
                                        index + 32u);
                    }
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs =
                         EDMA3QdmaGetErrIntrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
            while(pendingIrqs)
            {
                /* Process all the pending interrupts */
                if(TRUE == (pendingIrqs & 1u))
                {
                    /*
                    ** Here write to QEMCR to clear the corresponding QEMR bits
                    ** Clear any QSER
                    */
                    EDMA3QdmaClrMissEvt(EDMAAPP_EDMACC_BASE_ADDRESS, index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;

            pendingIrqs = EDMA3GetCCErrStatus(EDMAAPP_EDMACC_BASE_ADDRESS);
            if(pendingIrqs != 0u)
            {
                /*
                ** Process all the pending CC error interrupts.
                ** Queue threshold error for different event queues.
                */
                for(evtQueNum = 0u; evtQueNum < SOC_EDMA3_NUM_EVQUE;
                    evtQueNum++)
                {
                    if((pendingIrqs & (1u << evtQueNum)) != 0u)
                    {
                        /* Clear the error interrupt. */
                        EDMA3ClrCCErr(EDMAAPP_EDMACC_BASE_ADDRESS,
                                      (1u << evtQueNum));
                    }
                }

                /* Transfer completion code error. */
                if((pendingIrqs & (1u << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                {
                    EDMA3ClrCCErr(EDMAAPP_EDMACC_BASE_ADDRESS,
                                  (1u << EDMA3CC_CCERR_TCCERR_SHIFT));
                }
                ++index;
            }
            count++;
        }
    }
}
