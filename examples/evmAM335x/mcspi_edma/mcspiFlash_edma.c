/**
 * \file   mcspiFlash_edma.c 
 *
 * \brief  This is a sample application file which invokes some APIs from the
 *         McSPI device abstraction layer and EDMA device abstraction layer to
 *         perform configuration, transmission and reception of data between
 *         DDR SDRAM and McSPI Flash.
 *
 *         Application Configuration:
 *
 *             Modules Used:
 *                 McSPI0
 *                 UART0
 *                 EDMA0
 *
 *             Configurable parameters:
 *                 None.
 *
 *             Hard-coded configuration of other parameters:
 *                 1) Operating Speed - 24MHz
 *                 2) SPI Word length - 8 bits
 *                 3) Chip Select - 0
 *                 4) Chip Select Polarity - Low
 *                 5) Data pin configuration
 *                    a. Data Line 0 - MISO
 *                    b. Data Line 1 - MOSI
 *                 6) Master mode of operation
 *                 7) DMA mode of operation
 *
 *         Application Use Case:
 *             The application demonstrates writing and reading from a SPI
 *             enabled device, which in this case is a SPI enabled flash.
 *             In the example a data of 1 page size(i.e. 256 bytes) is
 *             written to the flash device. The data written on to flash
 *             is read back for verification.
 *             Part number of SPI Flash - Winbond W25Q64BV
 *
 *         Running the example:
 *             Upon execution of example the user is asked if the SPI flash
 *             needs to be erased before programming it. Depending on the
 *             input given the flash is either erased or not and then 256
 *             bytes of data is programmed on to flash. The programmed data
 *             is read back for verfication. Result is displayed on the
 *             console.
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

/* Include all the necessary header files */
#include "consoleUtils.h"
#include "edma_event.h"
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "hw_types.h"
#include "mcspi.h"
#include "error.h"
#include "edma.h"

/*****************************************************************************/
/*                      INTERNAL MACRO DEFINITIONS                           */
/*****************************************************************************/
#define MCSPI_TX_EVENT                  (EDMA3_CHA_MCSPI0_CH0_TX)
#define MCSPI_RX_EVENT                  (EDMA3_CHA_MCSPI0_CH0_RX)
#define MCSPI_TX0_REG                   (0x48030138u)
#define MCSPI_RX0_REG                   (0x4803013Cu)
#define MCSPI_IN_CLK                    (48000000u)
#define MCSPI_OUT_FREQ                  (24000000u)
#define FLASH_SECTOR_ADD_HIGH           (0x3Fu)
#define FLASH_SECTOR_ADD_LOW            (0x00u)
#define FLASH_SECTOR_ADD_MID            (0x00u)
#define FLASH_READ_STAT_REG1            (0x05u)
#define FLASH_WRITE_ENABLE              (0x06u)
#define FLASH_SECTOR_ERASE              (0x20u)
#define FLASH_PAGE_PROGRAM              (0x02u)
#define WRITE_EN_LATCHED                (0x02u)
#define FLASH_DATA_READ                 (0x03u)
#define IS_FLASH_BUSY                   (0x01u)
#define MCSPI_CH_NUM                    (0u)
#define EVT_QUEQUE_NUM                  (0u)
#define DUMMY_CH_NUM                    (5u)

/*****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                            */
/*****************************************************************************/
static void McSpiTxEdmaParamSet(unsigned int tccNum, unsigned int chNum,
                                volatile unsigned char *buffer, 
                                unsigned short buffLength);
static void McSpiRxEdmaParamSet(unsigned int tccNum, unsigned int chNum,
                                volatile unsigned char *buffer, 
                                unsigned short buffLength,
                                unsigned int destBidxFlag);
static void CallBack(unsigned int tccNum, unsigned int status);
static void McSPITransfer(unsigned short length);
static unsigned char FlashStatusRead(void);
static void TxDummyPaRAMConfEnable(void);
static void Edma3ErrorHandlerIsr(void);
static void Edma3ComplHandlerIsr(void);
static void RequestEDMA3Channels(void);
static void EDMA3AINTCConfigure(void);
static void FlashSectorErase(void);
static void FlashPageProgram(void);
static void EDMA3Initialize(void);
static void IsWriteEnabled(void);
static void ReadFromFlash(void);
static void WriteEnable(void);
static void IsFlashBusy(void);
static void McSPISetUp(void);
static void VerifyData(void);

/*****************************************************************************/
/*                         GLOBAL VARIABLES                                  */
/*****************************************************************************/
static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc,
                                      unsigned int status);
volatile unsigned char txBuffer[260];
volatile unsigned char rxBuffer[260];
volatile unsigned char vrfyData[256];
volatile unsigned char flagTx = 0;
volatile unsigned char flagRx = 0;

/*****************************************************************************/
/*                       LOCAL FUNCTION DEFINITIONS                          */
/*****************************************************************************/

/*
** Main Function.
*/
int main(void)
{
    unsigned char choice = 0;
    int retVal = E_FAIL;

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable the clocks for McSPI0 module.*/
    McSPI0ModuleClkConfig();

    /* Perform Pin-Muxing for SPI0 Instance.*/
    retVal = McSPIPinMuxSetup(0);

    if(S_PASS == retVal)
    {
        /* Perform Pin-Muxing for CS0 of SPI0 Instance.*/
        retVal = McSPI0CSPinMuxSetup(MCSPI_CH_NUM);

        if(S_PASS == retVal)
        {
            /* Enable IRQ in CPSR.*/
            IntMasterIRQEnable();

            ConsoleUtilsPrintf("Here the McSPI controller on the SOC");
            ConsoleUtilsPrintf(" communicates with the McSPI Flash.\r\n\r\n");

            /* Initialize the EDMA3 instance.*/
            EDMA3Initialize();

            /* Request EDMA3CC for Tx and Rx channels for SPI0. */
            RequestEDMA3Channels();

            /* Set up the McSPI instance.*/
            McSPISetUp();

            /* Enable the SPI Flash for writing to it. */
            WriteEnable();

            ConsoleUtilsPrintf("Do you want to erase a sector of the");
            ConsoleUtilsPrintf(" flash before writing to it ?.\r\n");
            ConsoleUtilsPrintf("Input y(Y)/n(N) to proceed.\r\n");

            ConsoleUtilsScanf("%c", &choice);

            if(('y' == choice) || ('Y' == choice))
            {
                /* Erasing the specified sector of SPI Flash. */
                FlashSectorErase();
            }

            /* Enable the SPI Flash for writing to it. */
            WriteEnable();

            /* Write data of 1 page size into a page of Flash.*/
            FlashPageProgram();

            /* Read data of 1 page size from a page of flash.*/
            ReadFromFlash();

            /* Verify the data written to and read from Flash are same or not.*/
            VerifyData();
        }
        else
        {
            PRINT_STATUS(E_INVALID_CHIP_SEL);
        }
    }
    else if(E_INVALID_PROFILE == retVal)
    {
        PRINT_STATUS(E_INVALID_PROFILE);
    }
    else
    {
        PRINT_STATUS(E_INST_NOT_SUPP);
    }

    while(1);
}

/*
**   This function will give the status of Flash status register.
*/
static unsigned char FlashStatusRead(void)
{
    unsigned short length = 0;

    txBuffer[0] = FLASH_READ_STAT_REG1;
    txBuffer[1] = 0xFF;

    length = 2;

    /* Configure the flash status read parameters of McSPI for Edma transmit.*/
    McSpiTxEdmaParamSet(MCSPI_TX_EVENT, MCSPI_TX_EVENT, txBuffer, 
                        length);

    /* Configure the flash status read parameters of McSPI for Edma receive.*/
    McSpiRxEdmaParamSet(MCSPI_RX_EVENT, MCSPI_RX_EVENT, rxBuffer, 
                        length, TRUE);

    /* Register the call-back function for Tx/Rx edma events of McSPI.*/
    cb_Fxn[MCSPI_TX_EVENT] = &CallBack;
    cb_Fxn[MCSPI_RX_EVENT] = &CallBack;

    McSPITransfer(length);

    return(rxBuffer[1]);
}

/*
** This function configures the power supply for EDMA3 Channel Controller 0
** and Transfer Controller 0, registers the EDMA interrupts in AINTC.
*/
static void EDMA3Initialize(void)
{
    /* Enable module clock for EDMA */
    EDMAModuleClkConfig();

    /* Initialization of EDMA3 */
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEQUE_NUM);

    /* Configuring the AINTC to receive EDMA3 Interrupts */
    EDMA3AINTCConfigure();
}

/*
**  This function will verify the data written to and read from flash and will
**  print the appropriate message.
*/ 
static void VerifyData(void)
{
    unsigned int index = 0;

    for(index = 4; index < 260; index++)
    {
        if(rxBuffer[index] != vrfyData[index - 4])
        {
            ConsoleUtilsPrintf("\r\n\r\n");
            ConsoleUtilsPrintf("VerifyData: Comparing the data written to");
            ConsoleUtilsPrintf(" and read from flash.\r\nThe two data blocks");
            ConsoleUtilsPrintf(" are unequal. Mismatch found at index ");
            ConsoleUtilsPrintf("%d",index + 1);

            ConsoleUtilsPrintf("\r\nThe data in the Flash and the one written ", -1);
            ConsoleUtilsPrintf("to it are not equal.\r\n", -1);

            break;
        }
    }
    
    if(260 == index)
    {
        ConsoleUtilsPrintf("\r\nThe data in the Flash and the one written ");
        ConsoleUtilsPrintf("to it are equal.\r\n");
    }
}

/*
**    This function will read data of 1 page size from a page of flash.
*/
static void ReadFromFlash(void)
{
    unsigned int index = 0;
    unsigned short length = 0;

    txBuffer[0] = FLASH_DATA_READ;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    for(index = 4; index < 260; index++)
    {
        txBuffer[index] = 0;
    }

    length = 260;

    /* Configure the read data parameters of McSPI for Edma transmit.*/
    McSpiTxEdmaParamSet(MCSPI_TX_EVENT, MCSPI_TX_EVENT, txBuffer, 
                        length);

    /* Configure the read data parameters of McSPI for Edma receive.*/
    McSpiRxEdmaParamSet(MCSPI_RX_EVENT, MCSPI_RX_EVENT, rxBuffer, 
                        length, TRUE);

    /* Register the call-back function for Tx/Rx edma events of McSPI.*/
    cb_Fxn[MCSPI_TX_EVENT] = &CallBack;
    cb_Fxn[MCSPI_RX_EVENT] = &CallBack;

    McSPITransfer(length);

    /* Check whether flash is busy in reading data from flash. */
    IsFlashBusy();
}

/*
**  This function will send the page program command to the flash device.
*/
static void FlashPageProgram(void)
{
    unsigned int index = 0;
    unsigned int dummy = 0;
    unsigned short length = 0;

    txBuffer[0] = FLASH_PAGE_PROGRAM;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    for(index = 0; index < 256; index++)
    {
        txBuffer[index + 4] = (unsigned char) index;
        vrfyData[index] = (unsigned char) index;
    }

    length = 260;

    /* Configure the Page-program parameters for Edma transmit.*/
    McSpiTxEdmaParamSet(MCSPI_TX_EVENT, MCSPI_TX_EVENT, txBuffer, 
                        length);

    /* Configure the Page-program parameters for Edma receive.*/
    McSpiRxEdmaParamSet(MCSPI_RX_EVENT, MCSPI_RX_EVENT, (unsigned char *)dummy, 
                        length, FALSE);

    /* Register the call-back function for McSPI Tx/Rx events.*/
    cb_Fxn[MCSPI_TX_EVENT] = &CallBack;
    cb_Fxn[MCSPI_RX_EVENT] = &CallBack;

    McSPITransfer(length);

    IsFlashBusy();
}

/*
**  This function will send a sector erase command and will erase a sector of 
**  Flash.
*/
static void FlashSectorErase(void)
{
    unsigned int dummy = 0;
    unsigned short length = 0;

    txBuffer[0] = FLASH_SECTOR_ERASE;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    length = 4;

    /* Configure the Sector erase parameters for Edma transmit.*/
    McSpiTxEdmaParamSet(MCSPI_TX_EVENT, MCSPI_TX_EVENT, txBuffer, 
                        length);

    /* Configure the sector erase parameters for Edma receive.*/
    McSpiRxEdmaParamSet(MCSPI_RX_EVENT, MCSPI_RX_EVENT, (unsigned char *)dummy, 
                        length, FALSE);

    /* Register the call-back function for McSPI Tx/Rx events.*/
    cb_Fxn[MCSPI_TX_EVENT] = &CallBack;
    cb_Fxn[MCSPI_RX_EVENT] = &CallBack;

    McSPITransfer(length);

    IsFlashBusy();

}

/*
**    This function will check whether flash is busy with any operation like
**    read, write, sector erase.
*/
static void IsFlashBusy(void)
{
    unsigned char temp = 0;
    
    do
    {
        temp = FlashStatusRead();
    }
    while(temp & IS_FLASH_BUSY);
}

/*
**  This function will send the Write Enable command to the Flash device.
*/
static void WriteEnable(void)
{
    unsigned int dummy = 0;
    unsigned short length = 0;

    txBuffer[0] = FLASH_WRITE_ENABLE;

    length = 1;

    /* Configure the write enable parameters for Edma transfer.*/
    McSpiTxEdmaParamSet(MCSPI_TX_EVENT, MCSPI_TX_EVENT, txBuffer,
                        length);

    /* Configure the write enable parameters for Edma receive.*/
    McSpiRxEdmaParamSet(MCSPI_RX_EVENT, MCSPI_RX_EVENT, (unsigned char *)dummy,
                        length, FALSE);

    /* Register the call-back function for Tx/Rx events of McSPI.*/
    cb_Fxn[MCSPI_TX_EVENT] = &CallBack;
    cb_Fxn[MCSPI_RX_EVENT] = &CallBack;

    McSPITransfer(length);

    /* Check whether write enable command is properly latched on to flash */
    IsWriteEnabled();
}

/*
**    This function will check whether write enable command is successfully
**    latched on to flash or not.
*/
static void IsWriteEnabled(void)
{
    unsigned char temp = 0;

    do
    {
        /* Read the status from flash */
        temp = FlashStatusRead();
    }
    while(!(temp & WRITE_EN_LATCHED)); 
}

/*
** This function will Assert the Chip select line before transmission, will 
** enable the Edma events for Tx/Rx of McSPI peripheral, will De-assert the 
** Chip select once communication is complete.
*/
static void McSPITransfer(unsigned short length)
{
    /* Set the word count field with the data length to be transferred.*/
    McSPIWordCountSet(SOC_SPI_0_REGS, length);
    
    /* Force the SPIEN to low state.*/
    McSPICSAssert(SOC_SPI_0_REGS, MCSPI_CH_NUM);

    /* Enable the Tx/Rx DMA events for McSPI. */
    McSPIDMAEnable(SOC_SPI_0_REGS, (MCSPI_DMA_RX_EVENT | MCSPI_DMA_TX_EVENT), 
                   MCSPI_CH_NUM);

    /* Enable the McSPI channel for communication.*/
    McSPIChannelEnable(SOC_SPI_0_REGS, MCSPI_CH_NUM);

    /* Wait for control to return from ISR.*/
    while((0 == flagTx) || (flagRx == 0));

    flagTx = 0;
    flagRx = 0;

    /* Force the SPIEN to high state.*/
    McSPICSDeAssert(SOC_SPI_0_REGS, MCSPI_CH_NUM);

    /* Disable the McSPI channel for communication.*/
    McSPIChannelDisable(SOC_SPI_0_REGS, MCSPI_CH_NUM);
}

/*
** EDMA3 completion Interrupt Service Routine(ISR).
*/
static void Edma3ComplHandlerIsr(void)
{
    unsigned int pendingIrqs;
    unsigned int isIPR = 0;
    unsigned int indexl;
    unsigned int Cnt = 0;

    indexl = 1;

    isIPR = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);

    if(isIPR)
    {
        while ((Cnt < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (indexl != 0))
        {
            indexl = 0;

            pendingIrqs = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);

            while (pendingIrqs)
            {
                if((pendingIrqs & 1) == TRUE)
                {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPR register will NOT be cleared.
                     */
                    /* Here write to ICR to clear the corresponding IPR bits. */
                    EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, indexl);

                    (*cb_Fxn[indexl])(indexl, EDMA3_XFER_COMPLETE);
                }

                ++indexl;

                pendingIrqs >>= 1;
            }
            Cnt++;
        }
    }

}

/* EDMA3 Error Handler */
static void Edma3ErrorHandlerIsr(void)
{
    volatile unsigned int pendingIrqs = 0;

    pendingIrqs = EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS);

    if((pendingIrqs & (0x01 << MCSPI_TX_EVENT)))
    {
        /* clear the pending error interrupt */
        EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, MCSPI_TX_EVENT);

        /* Disable McSPI Transmit event */
        McSPIDMADisable(SOC_SPI_0_REGS, MCSPI_DMA_TX_EVENT, MCSPI_CH_NUM);

        /* Disable Edma Transfer */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , MCSPI_TX_EVENT,
                             EDMA3_TRIG_MODE_EVENT);

        flagTx = 1;
    }

    else if ((pendingIrqs & (0x01 << MCSPI_RX_EVENT)))
    {
        /* clear the pending error interrupt */
        EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, MCSPI_RX_EVENT);

        /* Disable McSPI Receive event */
        McSPIDMADisable(SOC_SPI_0_REGS, MCSPI_DMA_RX_EVENT, MCSPI_CH_NUM);

        /* Disable Edma Transfer */
        EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS , MCSPI_RX_EVENT,
                             EDMA3_TRIG_MODE_EVENT);    

        flagRx = 1;
    }
}


/*
** Call back function. Here we disable the Tx/Rx DMA events of McSPI 
** peripheral.
*/
static void CallBack(unsigned int tccNum, unsigned int status)
{
    if(tccNum == MCSPI_TX_EVENT)
    {
        flagTx = 1;
    
        /* Disable McSPI Transmit event */ 
        McSPIDMADisable(SOC_SPI_0_REGS, MCSPI_DMA_TX_EVENT, MCSPI_CH_NUM);
                       
    }

    if(tccNum == MCSPI_RX_EVENT)
    {
        flagRx = 1;

        /* Disable McSPI Receive event */
        McSPIDMADisable(SOC_SPI_0_REGS, MCSPI_DMA_RX_EVENT, MCSPI_CH_NUM);
    }
}

/*
** This function is used to set the PaRAM entries of EDMA3 for the Receive
** event of channel 0 of McSPI0 instance. The corresponding EDMA3 channel 
** is also enabled for reception.
*/
static void McSpiRxEdmaParamSet(unsigned int tccNum, unsigned int chNum,
                                volatile unsigned char *buffer, 
                                unsigned short buffLength,
                                unsigned int destBidxFlag)
{
    EDMA3CCPaRAMEntry paramSet;

    unsigned char *p = (unsigned char *)&paramSet;
    unsigned int index = 0;

    /* Clean-up the contents of structure variable. */
    for (index = 0; index < sizeof(paramSet); index++)
    {
        p[index] = 0;
    }

    /* Fill the PaRAM Set with Receive specific information.*/

    /* srcAddr holds address of SPI Rx FIFO.*/
    paramSet.srcAddr = (unsigned int) (MCSPI_RX0_REG);

    /* destAddr is address of memory location named buffer.*/
    paramSet.destAddr = (unsigned int) buffer;

    /* aCnt holds the number of bytes in an array.*/
    paramSet.aCnt = 1;

    /* bCnt holds the number of such arrays to be transferred.*/
    paramSet.bCnt = buffLength;

    /* cCnt holds the number of frames of aCnt*bBcnt bytes to be transferred.*/
    paramSet.cCnt = 1;

    /* The srcBidx should not be incremented since it is a h/w register.*/
    paramSet.srcBIdx = 0;

    if(TRUE == destBidxFlag)
    {
        /* The destBidx should be incremented for every byte.*/
        paramSet.destBIdx = 1;
    }
    else
    {
        /* The destBidx should not be incremented.*/
        paramSet.destBIdx = 0;
    }

    /* A sync Transfer Mode. */
    /* srCIdx and destCIdx set to zero since ASYNC Mode is used.*/
    paramSet.srcCIdx = 0;
    paramSet.destCIdx = 0;

    /* Linking transfers in EDMA3 are not used.*/
    paramSet.linkAddr = 0xFFFF;

    paramSet.bCntReload = 0;

    paramSet.opt = 0x00000000;

    /* Set TCC field in OPT with the tccNum.*/
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* EDMA3 Interrupt is enabled and Intermediate Interrupt Disabled.*/
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRam Set to EDMA3.*/
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);

    /* EDMA3 Transfer is Enabled.*/
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, chNum, EDMA3_TRIG_MODE_EVENT);
}

/*
** This function is used to set the PaRAM entries of EDMA3 for the Transmit
** Channel 0 of SPI0 instance. The corresponding EDMA3 channel is also enabled
** for transmission.
*/
static void McSpiTxEdmaParamSet(unsigned int tccNum, unsigned int chNum,
                                volatile unsigned char *buffer, 
                                unsigned short buffLength)
{
    EDMA3CCPaRAMEntry paramSet;

    unsigned char *p = (unsigned char *)&paramSet;
    unsigned int index = 0;

    /* Clean-up the contents of structure variable. */
    for(index = 0; index < sizeof(paramSet); index++)
    {
        p[index] = 0;
    }

    /* Fill the PaRAM Set with transfer specific information. */

    /* srcAddr holds address of memory location buffer. */
    paramSet.srcAddr = (unsigned int) buffer;

    /* destAddr holds address of McSPI_TX register. */
    paramSet.destAddr = (unsigned int) (MCSPI_TX0_REG);

    /* aCnt holds the number of bytes in an array. */
    paramSet.aCnt = 1;

    /* bCnt holds the number of such arrays to be transferred. */
    paramSet.bCnt = buffLength;

    /* cCnt holds the number of frames of aCnt*bBcnt bytes to be transferred. */
    paramSet.cCnt = 1;

    /*
    ** The srcBidx should be incremented by aCnt number of bytes since the
    ** source used here is memory.
    */
    paramSet.srcBIdx = 1;
    paramSet.destBIdx = 0;

    /* Async Transfer Mode is set in OPT.*/
    /* srCIdx and destCIdx set to zero since ASYNC Mode is used. */
    paramSet.srcCIdx = 0;
    paramSet.destCIdx = 0;

    /* Linking transfers in EDMA3 are not used. */
    paramSet.linkAddr = (EDMA3CC_OPT(DUMMY_CH_NUM));
    paramSet.bCntReload = 0;

    paramSet.opt = 0x00000000;

    /* SAM and DAM fields both are set to 0 */

    /* Set TCC field in OPT with the tccNum. */
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* EDMA3 Interrupt is enabled and Intermediate Interrupt Disabled.*/
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRam Set to EDMA3.*/
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);

    /* Dummy param set is enabled */
    TxDummyPaRAMConfEnable();

    /* EDMA3 Transfer is Enabled. */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, chNum, EDMA3_TRIG_MODE_EVENT);
}


/*
** This function allocates EDMA3 channels to McSPI0 for trasmisssion and
** reception purposes.
*/
static void RequestEDMA3Channels(void)
{
    /* Request DMA Channel and TCC for SPI Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA, \
                        MCSPI_TX_EVENT, MCSPI_TX_EVENT, EVT_QUEQUE_NUM);

    /* Request DMA Channel and TCC for SPI Receive*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA, \
                        MCSPI_RX_EVENT, MCSPI_RX_EVENT, EVT_QUEQUE_NUM);
}

/*
** This function will call the necessary McSPI APIs which will configure the
** McSPI controller.
*/
static void McSPISetUp(void)
{
    /* Reset the McSPI instance.*/
    McSPIReset(SOC_SPI_0_REGS);

    /* Enable chip select pin.*/
    McSPICSEnable(SOC_SPI_0_REGS);

    /* Enable master mode of operation.*/
    McSPIMasterModeEnable(SOC_SPI_0_REGS);

    /* Perform the necessary configuration for master mode.*/
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH, MCSPI_TX_RX_MODE,
                          MCSPI_DATA_LINE_COMM_MODE_1, MCSPI_CH_NUM);

    /* Configure the McSPI output frequency. */ 
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, 
                   MCSPI_CH_NUM, MCSPI_CLK_MODE_0);

    /* Configure the word length.*/
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(8), MCSPI_CH_NUM);

    /* Set polarity of SPIEN to low.*/
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, MCSPI_CH_NUM);

    /* Enable the Tx FIFO of McSPI.*/
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_ENABLE, MCSPI_CH_NUM);

    /* Enable the Rx FIFO of McSPI.*/
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_ENABLE, MCSPI_CH_NUM);
}

/*
** This function configures the AINTC to receive EDMA3 interrupts.
*/
static void EDMA3AINTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3ComplHandlerIsr);

    /* Setting the priority for EDMA3CC0 completion interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 completion interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3ErrorHandlerIsr);

    /* Setting the priority for EDMA3CC0 Error interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 Error interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}

/*
** This configures the PaRAM set for the Dummy Transfer.
*/
static void TxDummyPaRAMConfEnable(void)
{
    EDMA3CCPaRAMEntry dummyPaRAMSet;

    EDMA3GetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);

    dummyPaRAMSet.aCnt = 1;
    dummyPaRAMSet.bCnt = 0;
    dummyPaRAMSet.cCnt = 0;
    dummyPaRAMSet.srcAddr = 0;
    dummyPaRAMSet.destAddr = 0;
    dummyPaRAMSet.srcBIdx = 0;
    dummyPaRAMSet.destBIdx = 0;
    dummyPaRAMSet.srcCIdx = 0;
    dummyPaRAMSet.destCIdx = 0;
    dummyPaRAMSet.linkAddr = 0xFFFFu;
    dummyPaRAMSet.bCntReload = 0;
    dummyPaRAMSet.opt = 0;

    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);
}

