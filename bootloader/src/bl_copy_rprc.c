/**
 * \file  bl_copy_rprc.c
 *
 * \brief RPRC Image copy functionality for various boot types.
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


#include "bl.h"
#include "uartStdio.h"
#include "hw_types.h"
#if defined(SPI)
  #include "bl_spi.h"
#elif defined(MMCSD)
  #include "bl_mmcsd.h"
#elif defined(NAND)
  #include "bl_nand.h"
#endif
#include "bl_platform.h"
#include "bl_rprc.h"
#include "bl_copy.h"

/******************************************************************************
**                     External variable Declaration 
*******************************************************************************/

extern void BootAbort(void);


/******************************************************************************
**                     Local function Declaration 
*******************************************************************************/

#if defined(SPI)
static void SPI_readBytes(void *value, int *cursor, int size);
static unsigned int SPIBootCopy(void);
#elif defined(MMCSD)
static unsigned int MMCSDBootCopy(void);
extern int HSMMCSDInit(void);
extern unsigned int HSMMCSDImageCopy(void);
#elif defined(NAND)
static void NAND_readBytes(NandInfo_t *hNandInfo, void *value, int *cursor, int size);
static unsigned int NANDBootCopy(void);
extern NandInfo_t *BlNANDConfigure(void);
#endif


/******************************************************************************
**                       Global Function Definitions 
*******************************************************************************/

void ImageCopy(void)
{
#if defined(SPI)
    if (SPIBootCopy( ) != true)
        BootAbort();
#elif defined(MMCSD)
    if (MMCSDBootCopy() != true)
        BootAbort();
#elif defined(NAND)
    if (NANDBootCopy() != true)
        BootAbort();
#else
    #error Unsupported boot mode !!
#endif
}

/******************************************************************************
**                       Local Function Definitions 
*******************************************************************************/

/**
 * \brief  This function parses an RPRC application image from SPI flash
 *
 * \param  none
 *
 * \return unsigned int: Status (success or failure)
*/
#if defined(SPI)
static unsigned int SPIBootCopy(void)
{
    BL_SPI_Header spiBootHeader;
    rprcFileHeader rprcHeader;
    rprcSectionHeader section;
    int offset = IMAGE_OFFSET;
    int sectionCount;
    
    /* Spi Initialization */ 
    BlPlatformSPISetup();
    BlSPIConfigure();

    // check magic number and read image size from SPI header
    SPI_readBytes(&spiBootHeader, &offset, sizeof(spiBootHeader));
    if ((spiBootHeader.magicNum != MAGIC_NUM_SF) &&
        (spiBootHeader.magicNum != MAGIC_NUM_GF))
    {
        UARTPuts("Invalid magic number in boot image\r\n", -1);
        BootAbort();
    }

    // read application image header
    SPI_readBytes(&rprcHeader, &offset, sizeof(rprcFileHeader));

    // check magic number
    if (rprcHeader.magic != RPRC_MAGIC_NUMBER)
    {
        UARTPuts("Invalid magic number in boot image\r\n", -1);
        BootAbort();
    }
    else if ( 4 < rprcHeader.text_len)
    {
        UARTPuts("WARNING: RPRC Boot image header has larger text section than expected.\r\n", -1);
    }
    else if ( 4 > rprcHeader.text_len)
    {
      UARTPuts("ERROR: RPRC Boot image header is malformed.\r\n", -1);
      BootAbort();
    }

    /* Get loadable section count */
    SPI_readBytes(&sectionCount, &offset, 4);
          
    /* Skip over any remaining text header */
    offset += (rprcHeader.text_len - 4);

    /* Read entrypoint(s) and copy sections to memory */
    while (sectionCount> 0)
    {
        /* Read new section header */
        SPI_readBytes(&section, &offset, sizeof(rprcSectionHeader));

        if (section.type == RPRC_RESOURCE)
        {
            /* check that resource has BOOTADDR type (ignore other resources) */
            SPI_readBytes(&section.type, &offset, sizeof(int));
            if (section.type == RPRC_BOOTADDR)
            {
                if (entryPoint == 0)
                    entryPoint = section.addr;
                else if (DspEntryPoint == 0)
                    DspEntryPoint = section.addr;
                else
                    UARTPuts("Ignoring extra entrypoint in boot image\r\n", -1);
            }

            /* Skip the rest of the resource section */
            offset += section.size - sizeof(int);
        }
        else
        {
            /* Copy section to memory */
            SPI_readBytes((void *)section.addr, &offset, section.size);
            --sectionCount;
        }
    }

    return true;
}


/*
 * \brief  This function reads N bytes from SPI flash and advances a cursor
 *
 * \param  void *value:
 * \param  int *cursor:
 * \param  int size:
 *
 * \return none
*/
static void SPI_readBytes(void *value, int *cursor, int size)
{
    BlSPIReadFlash(*cursor, size, value);
    *cursor += size;
}

#endif

/*
 * \brief  This function parses an RPRC application image from NAND flash
 *
 * \param  none
 *
 * \return unsigned int: Status (success or failure)
*/
#if defined(NAND)
static unsigned int NANDBootCopy(void)
{
    NandInfo_t *hNandInfo;
    BL_NAND_Header nandBootHeader;
    rprcFileHeader rprcHeader;
    rprcSectionHeader section;
    int offset = IMAGE_OFFSET;
    int sectionCount = 0;
    
    /* NAND Initialization */
    BlPlatformNANDSetup();
    hNandInfo = BlNANDConfigure();
    
    /* Check magic number and read image size from NAND header */
    NAND_readBytes(hNandInfo, &nandBootHeader, &offset, sizeof(nandBootHeader));
    if ((nandBootHeader.magicNum != MAGIC_NUM_SF) &&
        (nandBootHeader.magicNum != MAGIC_NUM_GF))
    {
        UARTPuts("Invalid magic number in boot image\r\n", -1);
        BootAbort();
    }

    /* Read application image header */
    offset = nandBootHeader.block * hNandInfo->blkSize + nandBootHeader.page * hNandInfo->pageSize;
    NAND_readBytes(hNandInfo, &rprcHeader, &offset, sizeof(rprcFileHeader));

    /* Check RPRC header */
    if (rprcHeader.magic != RPRC_MAGIC_NUMBER)
    {
        UARTPuts("Invalid magic number in boot image\r\n", -1);
        BootAbort();
    }
    else if ( 4 < rprcHeader.text_len)
    {
        UARTPuts("WARNING: RPRC Boot image header has larger text section than expected.\r\n", -1);
    }
    else if ( 4 > rprcHeader.text_len)
    {
      UARTPuts("ERROR: RPRC Boot image header is malformed.\r\n", -1);
      BootAbort();
    }
    
    /* Get loadable section count */
    NAND_readBytes(hNandInfo, &sectionCount, &offset, 4);
          
    /* Skip over any remaining text header */
    offset += rprcHeader.text_len - 4;

    /* Read entrypoint(s) and copy sections to memory */
    while (sectionCount> 0)
    {
        /* Read new section header */
        NAND_readBytes(hNandInfo, &section, &offset, sizeof(rprcSectionHeader));

        if (section.type == RPRC_RESOURCE)
        {
            /* Check that resource has BOOTADDR type (ignore other resources) */
            NAND_readBytes(hNandInfo, &section.type, &offset, sizeof(int));
            if (section.type == RPRC_BOOTADDR)
            {
                if (entryPoint == 0)
                    entryPoint = section.addr;
                else if (DspEntryPoint == 0)
                    DspEntryPoint = section.addr;
                else
                    UARTPuts("Ignoring extra entrypoint in boot image\r\n", -1);
            }

            /* Skip the rest of the resource section */
            offset += section.size - sizeof(int);
        }
        else
        {
            /* Copy section to memory */
            NAND_readBytes(hNandInfo,(void *)section.addr, &offset, section.size);
            --sectionCount;
        }
    }

    return true;
}

/*
 * \brief  This function reads N bytes from NAND flash and advances a cursor
 *
 * \param  NandInfo_t *hNandInfo: Nand Info structure
 * \param  void *value: pointer to memory to which bytes will be read
 * \param  int *cursor: pointer to offset into memory we are reading from 
 * \param  int size: number of bytes to read
 *
 * \return none
*/
static void NAND_readBytes(NandInfo_t *hNandInfo, void *value, int *cursor, int size)
{
    BlNANDReadFlash(hNandInfo, *cursor, size, value);
    *cursor += size;
}
#endif

/*
 * \brief  This function parses an RPRC application image from MMCSD
 *
 * \param  none
 *
 * \return unsigned int: Status (success or failure)
*/

#if defined(MMCSD)
static unsigned int MMCSDBootCopy(void)
{
    HSMMCSDInit();
    HSMMCSDImageCopy();
    
    return true;
}
#endif


/******************************************************************************
**                              END OF FILE
*******************************************************************************/
