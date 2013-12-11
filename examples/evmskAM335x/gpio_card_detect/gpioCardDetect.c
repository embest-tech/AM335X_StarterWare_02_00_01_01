/**
 *  \file   gpioCardDetect.c
 *
 *  \brief  This is a sample application which demonstrates the configuration 
 *          and usage of GPIO. The application indicates whether the MMC/SD card
 *          is inserted by blinking an LED.
 *
 *          Application Configuration:
 *
 *              Modules Used:
 *                  GPIO0
 *                  GPIO1
 *                  UART0
 *                  Interrupt Controller
 *
 *              Configurable Parameters(Runtime)
 *                   None
 *
 *              Hard coded configurations (compile time)
 *                  1) Debouncing Feature - Enabled
 *                  2) Debouncing Time - 1.519ms 
 *                  3) Interrupt Request Lines - Line 1
 *                  4) Interrupt Event Type - Rising & Falling edge
 *
 *          Application Use Cases:
 *              The card detection and controlling the LED happens in the following
 *              manner:
 *              1) A Pin (GPIO0[6]) is used as an Input Pin. This detects the
 *                 insertion and ejection of a MicroSD Card from its slot on
 *                 the board.
 *              2) Another GPIO pin (GPIO1[6]) is used as an Output Pin. This
 *                 pin drives a LED in the manner mentioned above.
 *                      
 *
 *          Running the application:
 *              Insert/Eject a MicroSD card into/from its slot to observe the
 *              following actions:
 *              1) An appropriate message will be seen on serial console.
 *              2) A LED will be turned ON when the card is inserted and will
 *                 remain ON until the card is ejected. Similarly the LED will
 *                 be turned OFF when the card is ejected and will remain OFF
 *                 until it is inserted again.
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

#include "hw_control_AM335x.h"
#include "consoleUtils.h"
#include "evmskAM335x.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "hw_types.h"
#include "pin_mux.h"
#include "gpio_v2.h"
#include "cache.h"
#include "mmu.h"

/*****************************************************************************
**                     INTERNAL MACRO DEFINITIONS
*****************************************************************************/

/* Wrapper Definitions related to GPIO Card Detect Pin. */
#define GPIO_INST_BASE_ADD_CD         (SOC_GPIO_0_REGS)
#define GPIO_CD_PIN_NUM               (6u)
#define GPIO_INST_SYS_INT_NUM         (SYS_INT_GPIOINT0A)

/* Wrapper Definitions related to GPIO Pin controlling the LED. */
#define GPIO_INST_BASE_ADD_LED        (SOC_GPIO_1_REGS)
#define GPIO_LED_PIN_NUM              (6u)

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR                (0x80000000)
#define START_ADDR_DEV                (0x44000000)
#define START_ADDR_OCMC               (0x40300000)
#define NUM_SECTIONS_DDR              (512)
#define NUM_SECTIONS_DEV              (960)
#define NUM_SECTIONS_OCMC             (1)

/*****************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
*****************************************************************************/
static void GPIOINTCConfigure(void);
static void CheckCardStatus(void);
static void GPIOIsr(void);

/*****************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*****************************************************************************/
static volatile unsigned int gpioIsrFlag = 0;

/* Page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];

#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY]
 __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif

/*****************************************************************************
**                     FUNCTION DEFINITIONS
*****************************************************************************/

/*
** This function will setup the MMU. The function maps three regions -
** 1. DDR
** 2. OCMC RAM
** 3. Device memory
** The function also enables the MMU.
*/
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
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
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

/*
** Main function.
*/
int main(void)
{
    /* Configure and enable the MMU. */
    MMUConfigAndEnable();

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);

    /* Initialize the UART console */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Enable IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* This function enables the functional clocks for the GPIO0 instance. */
    GPIO0ModuleClkConfig();

    /* This function enables the functional clocks for the GPIO1 instance. */
    GPIO1ModuleClkConfig();

    /* Perform Pin Multiplexing for the pin GPIO0[6]. */
    GPIO_PMUX_OFFADDR_VALUE(0, 6, PAD_FS_RXE_PU_PUPDE(7));

    /* Perform Pin Multiplexing for the pin GPIO1[6]. */
    GPIO_PMUX_OFFADDR_VALUE(1, 6, PAD_FS_RXD_PD_PUPDE(7));

    /**        Configure the GPIO pin driving the LED.                    **/

    /* Enable the GPIO Module. */
    GPIOModuleEnable(GPIO_INST_BASE_ADD_LED);

    /* Perform a reset of the GPIO Module. */
    GPIOModuleReset(GPIO_INST_BASE_ADD_LED);

    /* Configure the GPIO Pin that controls the LED as an Output Pin. */
    GPIODirModeSet(GPIO_INST_BASE_ADD_LED,
                   GPIO_LED_PIN_NUM,
                   GPIO_DIR_OUTPUT);

    /**        Configure the GPIO Pin used for Card Detection.            **/

    /* Enable the GPIO Module. */
    GPIOModuleEnable(GPIO_INST_BASE_ADD_CD);

    /* Perform a reset of the GPIO Module. */
    GPIOModuleReset(GPIO_INST_BASE_ADD_CD);

    /* Configure the Card Detection Pin as an Input Pin. */
    GPIODirModeSet(GPIO_INST_BASE_ADD_CD,
                   GPIO_CD_PIN_NUM,
                   GPIO_DIR_INPUT);

    /* Enable Debouncing feature for the Intput GPIO Pin. */
    GPIODebounceFuncControl(GPIO_INST_BASE_ADD_CD,
                            GPIO_CD_PIN_NUM,
                            GPIO_DEBOUNCE_FUNC_ENABLE);

    /*
    ** Configure the Debouncing Time for all the input pins of
    ** the seleceted GPIO instance.
    */
    GPIODebounceTimeConfig(GPIO_INST_BASE_ADD_CD, 48);

    ConsoleUtilsPrintf("StarterWare AM335x GPIO Application.\r\n\r\n");
    ConsoleUtilsPrintf("Insert/Eject a MicroSD card onto/from the");
    ConsoleUtilsPrintf(" board.\r\n\r\n");

    /* Configure the INTC to receive GPIO Interrupts. */
    GPIOINTCConfigure();

    /*
    ** Disable interrupt generation on detection of a logic HIGH or
    ** LOW levels.
    */
    GPIOIntTypeSet(GPIO_INST_BASE_ADD_CD,
                   GPIO_CD_PIN_NUM,
                   GPIO_INT_TYPE_NO_LEVEL);

    /*
    ** Enable interrupt generation on detection of a rising or a
    ** falling edge.
    */
    GPIOIntTypeSet(GPIO_INST_BASE_ADD_CD,
                   GPIO_CD_PIN_NUM,
                   GPIO_INT_TYPE_BOTH_EDGE);

    /* Enable interrupt for the specified GPIO Input Pin. */
    GPIOPinIntEnable(GPIO_INST_BASE_ADD_CD,
                     GPIO_INT_LINE_1,
                     GPIO_CD_PIN_NUM);

    while(1)
    {
        if(1 == gpioIsrFlag)
        {
            CheckCardStatus();
        }
    }
}

/*
** GPIO Interrupt Service Routine.
*/
static void GPIOIsr(void)
{
    /* Check the Interrupt Status of the GPIO Card Detect pin. */
    if(GPIOPinIntStatus(GPIO_INST_BASE_ADD_CD,
                        GPIO_INT_LINE_1,
                        GPIO_CD_PIN_NUM) & (1 << GPIO_CD_PIN_NUM))
    {
        /* Clear the Interrupt Status of the GPIO Card Detect pin. */
        GPIOPinIntClear(GPIO_INST_BASE_ADD_CD,
                        GPIO_INT_LINE_1,
                        GPIO_CD_PIN_NUM);
    }

    gpioIsrFlag = 1;
}

/*
** This function checks the insertion and ejection status of the MicroSD card
** and does appropriate operations.
*/
static void CheckCardStatus(void)
{
    /* Read the data on the GPIO Card Detect Pin. */
    if(GPIOPinRead(GPIO_INST_BASE_ADD_CD, GPIO_CD_PIN_NUM))
    {
        ConsoleUtilsPrintf("MicroSD card was removed.\n\r");

        /* Drive a logic LOW on the GPIO Pin controlling the LED. */
        GPIOPinWrite(GPIO_INST_BASE_ADD_LED,
                     GPIO_LED_PIN_NUM,
                     GPIO_PIN_LOW);
    }
    else
    {
        ConsoleUtilsPrintf("MicroSD card was inserted.\n\r");

        /* Drive a logic HIGH on the GPIO Pin controlling the LED. */
        GPIOPinWrite(GPIO_INST_BASE_ADD_LED,
                     GPIO_LED_PIN_NUM,
                     GPIO_PIN_HIGH);
    }
 
    gpioIsrFlag = 0;
}

/*
** This function configures the Interrupt Controller (INTC) to receive
** GPIO Interrupt.
*/
static void GPIOINTCConfigure(void)
{
    /* Initialize the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Register the Interrupt Service Routine(ISR). */
    IntRegister(GPIO_INST_SYS_INT_NUM, GPIOIsr);

    /* Set the priority for the GPIO0 system interrupt in INTC. */
    IntPrioritySet(GPIO_INST_SYS_INT_NUM, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the GPIO0 system interrupt in INTC. */
    IntSystemEnable(GPIO_INST_SYS_INT_NUM);
}

/***************************** End of file ***********************************/
