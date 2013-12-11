/**
 * \file  bl_copy.c
 *
 * \brief Initializes the Spi. Copies the application from SPI flash to DDR.
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


#include "uartStdio.h"
#include "bl.h"
#include "bl_platform.h"
#include "bl_copy.h"
#include "hw_types.h"
#if defined(SPI)
    #include "bl_spi.h"
#elif defined(MMCSD)
    #include "bl_mmcsd.h"
#elif defined(NAND)
    #include "bl_nand.h"
#endif


/******************************************************************************
**                          Extern Declarations 
*******************************************************************************/


/******************************************************************************
**                     Local Function Declarations 
*******************************************************************************/
#if defined(SPI)
    static unsigned int SPIBootCopy(void);
#elif defined(MMCSD)
    static unsigned int MMCSDBootCopy(void);
    extern int HSMMCSDInit(void);
    extern unsigned int HSMMCSDImageCopy(void);
#elif defined(NAND)
    static unsigned int NANDBootCopy(void);
    extern NandInfo_t *BlNANDConfigure(void);
#elif defined(UART)
    static unsigned int UARTBootCopy(void);
    extern int xmodemReceive(unsigned char *dest, int destsz);
#else
    #error Unsupported boot mode !!
#endif


/******************************************************************************
**                      Global Variable Declarations 
*******************************************************************************/


/******************************************************************************
**                       Global Function Definitions 
*******************************************************************************/
/*
 * \brief This function copies Image 
 *
 * \param  none
 *
 * \return none 
*/
void ImageCopy(void)
{
#if defined(SPI)
    if (SPIBootCopy( ) != true)
        BootAbort();
#elif defined(MMCSD)
        MMCSDBootCopy();
#elif defined(UART)
    if (UARTBootCopy() != true)
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

/*
 * \brief This function Initializes and configures SPI and copies 
 *        data from SPI FLASH.
 *
 * \param  none
 *
 * \return unsigned int: Status (Success or Failure) 
*/
#ifdef SPI
static unsigned int SPIBootCopy(void)
{
    unsigned int retVal;
    /* SPI Initialization */
    BlPlatformSPISetup();

    UARTPuts("Copying application image from SPI to RAM\r\n", -1);

    retVal = BlPlatformSPIImageCopy();

    return retVal;
}
#endif

/*
 * \brief This function Initializes and configures MMCSD and copies 
 *        data from MMCSD. 
 *
 * \param  none
 *
 * \return unsigned int: Status (Success or Failure) 
*/
#ifdef MMCSD
static unsigned int MMCSDBootCopy(void)
{
    unsigned int retVal;

    BlPlatformMMCSDSetup();

    retVal = BlPlatformMMCSDImageCopy();

    return retVal;
}
#endif

/*
 * \brief This function Initializes and configures NAND and copies
 *        data from NAND
 *
 * \param  none
 *
 * \return unsigned int: Status (Success or Failure)
*/
#ifdef NAND
static unsigned int NANDBootCopy(void)
{
    NandInfo_t *nandInfo;
    unsigned int retVal;

    /* Platform/Device specific NAND setup */
    BlPlatformNANDSetup();

    /* Open NAND device (uses platform specific functions) */
    nandInfo = BlNANDConfigure();

    UARTPuts("Copying application image from NAND to RAM\r\n", -1);

    retVal = BlPlatformNANDImageCopy(nandInfo);

    return retVal;
}
#endif

/**
*
* \brief   : This function receives the file from UART using XMODEM protocol. 
* \param   : None
* \return  : Status (Success or Failure)

**/

#ifdef UART
static unsigned int UARTBootCopy(void)
{
    unsigned int retVal = true;

    UARTPuts("\nPlease transfer file:\n", -1);

    if( 0 > xmodemReceive((unsigned char *)DDR_START_ADDR,
                          BL_UART_MAX_IMAGE_SIZE))
    {
        UARTPuts("\nXmodem receive error\n", -1);
        retVal = FALSE;
    }

    UARTPuts("\nCopying application image from UART to RAM is  done\n", -1);

    entryPoint  = DDR_START_ADDR;

    /*
    ** Dummy return.
    */
    return retVal;
}
#endif
/******************************************************************************
**                              END OF FILE
*******************************************************************************/

