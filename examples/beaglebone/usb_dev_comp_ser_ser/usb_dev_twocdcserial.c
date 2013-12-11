//****************************************************************************
//
// usb_dev_chidcdc.c - Main routines for the composite two CDC serial example.
//
//****************************************************************************
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

#include "raster.h"
#include "ustdlib.h"
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "hw_types.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usb.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "hw_usb.h"
#include "delay.h"
#include "usbdcomp.h"
#include "usb_structs.h"
#include "debug.h"
#include "mmu.h"

/****************************************************************************
*
* USB composite Two CDC serial Device (usb_dev_twocdcserial)
*
* This example application turns the evaluation board into a composite USB
* two CDC serial devices. The serial port application supports the
* USB Communication Device Class, Abstract Control Model to redirect CDC serial
* device traffic to and from the other CDC serial device. File usb_dev_serial.inf
* may be used to install the example as a virtual COM port on host side.
*
****************************************************************************/


/****************************************************************************
*
* Holds command bits used to signal the main loop to perform various tasks.
*
*****************************************************************************/

volatile unsigned int g_ulFlags;

/*****************************************************************************
*
*  The memory allocated to hold the composite descriptor that is created by
*  the call to USBDCompositeInit().
*
****************************************************************************/

unsigned char g_pucDescriptorData[DESCRIPTOR_DATA_SIZE];

/******************************************************************************
*
*  page tables start must be aligned in 16K boundary
*
*******************************************************************************/

#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif


/*****************************************************************************
*
*  External CDC serial definitions used by the composite main routine.
*
*******************************************************************************/

extern const tUSBDCDCDevice g_sCDCDevice1;
extern const tUSBDCDCDevice g_sCDCDevice2;

/*****************************************************************************
*
*  External composite device definitions used by the composite main routine.
*
******************************************************************************/

extern tUSBDCompositeDevice g_sCompDevice;
extern tCompositeEntry g_psCompDevices[];

/*****************************************************************************
*
*  External USB buffer pointer used by the composite main routine.
*
******************************************************************************/

extern const tUSBBuffer g_sTxBuffer1;
extern const tUSBBuffer g_sRxBuffer1;
extern const tUSBBuffer g_sTxBuffer2;
extern const tUSBBuffer g_sRxBuffer2;

/*****************************************************************************
*
*  External status variable used by the composite main routine.
*
******************************************************************************/

extern char *g_pcStatus;

/******************************************************************************
*                                                                             *
* \brief  This function Configures the AINTC controller for USB.\n            *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void USB0AINTCConfigure(void)
{

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_USB0);
}

/******************************************************************************
*                                                                             *
* \brief  This function Enables the interrupts for USB.\n                     *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void USBInterruptEnable(void)
{

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Configuring AINTC to receive USB0 interrupts. */
    USB0AINTCConfigure();

}

/******************************************************************************
*                                                                             *
* \brief  Function to setup MMU. This function Maps three regions (1. DDR     *
*         2. OCMC and 3. Device memory) and enables MMU.                      *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                     MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                      MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

/******************************************************************************
*                                                                             *
* \brief  Generic event handler for the composite device.\n                   *
*                                                                             *
* \param pvCBData    : Is the event callback pointer provided during          *
*                       USBDCompositeInit                                     *
*                                                                             *
* \param ulEvent     : Identifies the event we are being called back for.     *
*                                                                             *
* \param ulMsgValue  : Is an event-specific value.                            *
*                                                                             *
* \param pvMsgData   : Is an event-specific pointer.                          *
*                                                                             *
* \return  The return value is event-specific.                                *
*                                                                             *
******************************************************************************/
unsigned int EventHandler(void *pvCBData, unsigned int ulEvent,
                          unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ulIntsOff;

    /*  Which event are we being asked to process?   */
    switch(ulEvent)
    {
        /* We are connected to a host and communication is now possible. */

        case USB_EVENT_CONNECTED:

            /* Flush our buffers. */

            USBBufferFlush(&g_sTxBuffer1);
            USBBufferFlush(&g_sRxBuffer1);
            USBBufferFlush(&g_sTxBuffer2);
            USBBufferFlush(&g_sRxBuffer2);
            ulIntsOff = IntDisable();
            g_pcStatus = "Host connected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;


        /* The host has disconnected. */
        case USB_EVENT_DISCONNECTED:
            ulIntsOff = IntDisable();
            g_pcStatus = "Host disconnected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;
        default:
            break;
    }
    return(0);
}

/*****************************************************************************
*
*  This is the main loop that runs the application.
*
*****************************************************************************/
int main(void)
{

    MMUConfigAndEnable();

    /* Enable USB module clock */

    USB0ModuleClkConfig();

    /* configures arm interrupt controller to generate raster interrupt  */

    USBInterruptEnable();


    /* Delay timer setup */

    DelayTimerSetup();

    /* Pass the USB library our device information, initialize the USB
       controller and connect the device to the bus.
    */

    g_psCompDevices[0].pvInstance =
        USBDCDCCompositeInit(0, (tUSBDCDCDevice *)&g_sCDCDevice1);
    g_psCompDevices[1].pvInstance =
        USBDCDCCompositeInit(0, (tUSBDCDCDevice *)&g_sCDCDevice2);

    /*
      Pass the device information to the USB library and place the device
      on the bus.
    */
    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_DATA_SIZE,
                      g_pucDescriptorData);


    /* Initialize the serial devices. */
    SerialInit();


    /* Drop into the main loop. */
    while(1)
    {

        /* Allow the main serial routine to run. */
        SerialMain();
    }
}
