/**
 * \file  spi.c
 *
 * \brief Spi Initialization functions.  And a funciton to copy data from Flash
 *        to the given address.
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

#include "mcspi.h"
#include "bl.h"
#include "bl_platform.h"
#include "bl_spi.h"
#include "uartStdio.h"

 
/******************************************************************************
**                     Macro Defination 
*******************************************************************************/

#define CHAR_LENGTH             0x8
#define MCSPI_IN_CLK            48000000
#define MCSPI_OUT_CLK           24000000

/* flash data read command */
#define SPI_FLASH_READ          0x03

/******************************************************************************
**                    Local  Declaration 
*******************************************************************************/
static void McSPITransfer(unsigned char *p_tx,
                         unsigned char *p_rx,
                         unsigned int len);

/*
* \brief - SPI Configures.
* \param - none.
*
* \return  none.
*/

void SPIConfigure(void)
{
    unsigned int retVal = 0;

    /* Reset the McSPI instance.*/
    McSPIReset(SPI_BASE);

    /* Enable chip select pin.*/
    McSPICSEnable(SPI_BASE);

    /* Enable master mode of operation.*/
    McSPIMasterModeEnable(SPI_BASE);

    /* Perform the necessary configuration for master mode.*/
    retVal = McSPIMasterModeConfig(SPI_BASE, MCSPI_SINGLE_CH, MCSPI_TX_RX_MODE,\
                                   MCSPI_DATA_LINE_COMM_MODE_1, SPI_CHAN);

    /* 
    ** If combination of trm and IS,DPE0 and DPE1 is not valid then retVal is 
    ** false.
    */
    if(!retVal)
    {
        UARTPuts("Invalid McSPI config \r\n", -1);
        UARTPuts("Aborting Boot\r\n", -1);
        BootAbort();
    }

    /* 
    ** Default granularity is used. Also as per my understanding clock mode 
    ** 0 is proper.
    */    
    McSPIClkConfig(SPI_BASE, MCSPI_IN_CLK, MCSPI_OUT_CLK, 
                   SPI_CHAN, MCSPI_CLK_MODE_0);                                        

    /* Configure the word length.*/    
    McSPIWordLengthSet(SPI_BASE, MCSPI_WORD_LENGTH(8), SPI_CHAN);                

    /* Set polarity of SPIEN to low.*/
    McSPICSPolarityConfig(SPI_BASE, MCSPI_CS_POL_LOW, SPI_CHAN);

    /* Enable the transmitter FIFO of McSPI peripheral.*/
    McSPITxFIFOConfig(SPI_BASE, MCSPI_TX_FIFO_ENABLE, SPI_CHAN);

    /* Enable the receiver FIFO of McSPI peripheral.*/
    McSPIRxFIFOConfig(SPI_BASE, MCSPI_RX_FIFO_ENABLE, SPI_CHAN);     
} 

/**
* \brief - Reads bytes from SPI Flash.
* \param - offset - SPI Flash address.\n.
* \param - size - Indicates the total size needs to be read from flash.\n.
* \param - dst - Destination address where data needs to be copy.\n.
*
* \return none
**/
void BlSPIReadFlash (unsigned int offset,
                     unsigned int size,
                     unsigned char *dst)
{
    unsigned char tx_data;
    unsigned char rx_data;
    unsigned char addr[3];
    unsigned int len;

    /* The process of reading the data from the flash involves asserting
     * proper chipselect line, asserting CSHOLD, selecting correct data format.
     * Then the flash needs a command to indicate start of read and then
     * any number of bytes can be read. After the required number of bytes
     * are read, the CS needs to be de-asserted to indicate the end of transfer
     */
    McSPICSAssert(SPI_BASE, SPI_CHAN);

    /* Enable the McSPI channel for communication.*/
    McSPIChannelEnable(SPI_BASE, SPI_CHAN);

    /* Send read command to the flash (one byte) */
    tx_data =  SPI_FLASH_READ;
    McSPITransfer(&tx_data, &rx_data, 1);

    /* Send the address to start reading from (3 bytes) */
    addr[0] = (unsigned char)(offset >> 16);
    addr[1] = (unsigned char)(offset >> 8);
    addr[2] = (unsigned char)offset;
    len = 0;

    while (len < sizeof(addr))
    {
        McSPITransfer(&addr[len], &rx_data, 1);
        len++;
    }

    /* Read all the bytes */
    len = 0;
    tx_data = 0;    

    while(len < size)
    {
        McSPITransfer(&tx_data, dst, 1);
        dst++;
        len++;
    }

    /* Force SPIEN line to the inactive state.*/
    McSPICSDeAssert(SPI_BASE, SPI_CHAN);

    /* Disable the McSPI channel.*/
    McSPIChannelDisable(SPI_BASE, SPI_CHAN);
}

/**
* \brief - Spi Write and Read.
* \param - p_tx - SPI transmit data address.\n.
* \param - p_rx - SPI data reception address.\n.
* \param - len - Indicates the total length the read and write has to do.\n.
*
* \return none
**/

static void McSPITransfer(unsigned char *p_tx, unsigned char *p_rx,
                          unsigned int len)
{
    while(len)
    {
	/* Wait till TX is empty. */
        while(MCSPI_INT_TX_EMPTY(SPI_CHAN) !=
              (McSPIIntStatusGet(SPI_BASE) & MCSPI_INT_TX_EMPTY(SPI_CHAN)));
        McSPITransmitData(SPI_BASE, *p_tx, SPI_CHAN);

        p_tx++;

	/* Wait till the DATA in RX. */       
        while(MCSPI_INT_RX_FULL(SPI_CHAN) !=
              (McSPIIntStatusGet(SPI_BASE) & MCSPI_INT_RX_FULL(SPI_CHAN)));
        *p_rx = McSPIReceiveData(SPI_BASE, SPI_CHAN);

        p_rx++;
        len--;
    }
}

/***************************** End Of File ***********************************/
