/*
 * enet_lwip.c
 *
 * \brief   This is a sample application file demonstrating httpserver.
 *
 *          Application Configuration:
 *
 *              Modules Used:
 *                  Ethernet
 *                  Interrupt
 *                  UART0
 *                  Timer7
 *
 *              Configurable Parameters:
 *                  Phy Configuration - Forced or Autonegotiation
 *
 *              Hard-coded configuration of other parameters:
 *                  IP Address - DHCP is default configuration. For manual
 *                               configuration use STATIC_IP_ADDRESS_PORT1
 *                               or STATIC_IP_ADDRESS_PORT2 in lwipopts.h.
 *                  Operating Modes - DUAL MAC mode is default configuration.
 *                                    Comment CPSW_DUAL_MAC_MODE definition
 *                                    in lwipopts.h for Switch mode.
 *                  Console interface - Uart interface is default configuration.
 *                                      For semihosting set ConsoleUtilsSetType
 *                                      to CONSOLE_DEBUGGER and set compile time
 *                                      macro CONSOLE as SEMIHOSTING.
 *
 *          Application UseCase:
 *              Use httppserver, application of LwIP, to host web page.
 *
 *          Running the example:
 *              1. Connect board to Network, load example application and run.
 *              2. Application performs phy configuration and IP acquisition.
 *              3. Usee IP displayed on console to access web page from host PC.
 *              4. UART console provides interface to configure the parameters
 *                 listed under configurable parameters
 *
 *          Limitations:
 *              None
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

#include "locator.h"
#include "httpd.h"
#include "interrupt.h"
#include "lwiplib.h"
#include "lwipopts.h"
#include "consoleUtils.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "beaglebone.h"
#include "cache.h"
#include "delay.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

#define LEN_IP_ADDR                        (4u)
#define ASCII_NUM_IDX                      (48u) 

#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)

#define CONFIG_SWITCH_EXIT_CMD             (2)

#define DEF_SLV_PORT                       (1)
#define DEF_AUTONEG                        (1)
#define DEF_PHY_CONFIG                     (0x3f)
#define DEF_SPEED                          (1)
#define DEF_DUPLEX                         (1)

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

static CPSW_PHY_PARAM_IF cpswPhyParam;
static CPSW_CONF_IF cpswConfig;

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void CPSWCore0RxIsr(void);
static void CPSWCore0TxIsr(void);
static void AintcCPSWIntrSetUp(void);
static void IpAddrDisplay(unsigned int ipAddr);
static void MMUConfigAndEnable(void);

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/*
** Function to setup MMU. This function Maps three regions (1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
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
** This function takes user input for value and checks given values is the
** range of min and max values. If not default value is assiagned.
*/
static unsigned int UserValueInfoGet(unsigned int min, unsigned int max,
                                     unsigned int def, unsigned int hex,
                                     unsigned char *help)
{
    unsigned int value = 0;

    ConsoleUtilsPrintf("%s", help);

    if(FALSE == hex)
    {
        ConsoleUtilsScanf("%d", &value);
    }
    else
    {
        ConsoleUtilsScanf("%x", &value);
    }

    if((min > value) || (value > max))
    {
        ConsoleUtilsPrintf("\n\rSetting Default Value: 0x%x\r\n", def);
        return def;
    }

    return value;
}

/*
** This is the CPSW Switch Configuration interface to
**   1. Take User Input
**   2. Configure the CPSW
**   3. Print the status onto user console
**
** Available PHY Configuration
**  1 - Configure Phy of a port
**
*/
static unsigned int EnetLWIPSwitchConfiguration(unsigned int switchConfig)
{
    unsigned char slavePortInputHelp[] =
     "\nSlave Port (0x1-0x2):0x";
    unsigned char autonegInputHelp[] =
     "\nSelect Configuration (0x0 - Manual, 0x1 - Autonegotiation) : 0x";
    unsigned char configSpeedDuplexInputHelp1[] =
     "\nSelect Speed-Duplex Modes\n BIT0 - 10 Base Half Duplex\n BIT1 - 10 Bas";
    unsigned char configSpeedDuplexInputHelp2[] =
     "e FULL Duplex\n BIT2 - 100 Base Half Duplex\n BIT3 - 100 Base Full Duple";
    unsigned char configSpeedDuplexInputHelp3[] =
     "x\n BIT4 - 1000 Base Half Duplex\n BIT5 - 1000 Base Full Duplex\n : 0x";
    unsigned char speedInputHelp[] =
     "\nSpeed (0x0 - 10 Base, 0x1 - 100 Base, 0x2 - 1000 Base) : 0x";
    unsigned char duplexInputHelp[] =
     "\nDuplex (0x0 - Half Duplex, 0x1 - Full Duplex) : 0x";

    cpswConfig.cmd = CONFIG_SWITCH_INVALID;
    cpswConfig.cpsw_inst = 0;
    cpswConfig.ret = ERR_INVAL;
    cpswConfig.phy_param->slv_port_num = 0;
    cpswConfig.phy_param->autoneg = 0;
    cpswConfig.phy_param->config = 0;
    cpswConfig.phy_param->speed = 0;
    cpswConfig.phy_param->duplex = 0;

    switch(switchConfig)
    {
        case CONFIG_SWITCH_SET_PORT_CONFIG:
        {
            cpswConfig.cmd = CONFIG_SWITCH_SET_PORT_CONFIG;
            cpswPhyParam.slv_port_num = UserValueInfoGet(MIN_SLV_PORT,
                                                         MAX_SLV_PORT,
                                                         DEF_SLV_PORT, TRUE,
                                                         slavePortInputHelp);
            cpswPhyParam.autoneg = UserValueInfoGet(MIN_AUTONEG,
                                                    MAX_AUTONEG,
                                                    DEF_AUTONEG, TRUE,
                                                    autonegInputHelp);
            if(TRUE == cpswPhyParam.autoneg)
            {
                ConsoleUtilsPrintf("%s", configSpeedDuplexInputHelp1);
                ConsoleUtilsPrintf("%s", configSpeedDuplexInputHelp2);
                cpswPhyParam.config = UserValueInfoGet(MIN_PHY_CONFIG,
                                                       MAX_PHY_CONFIG,
                                                       DEF_PHY_CONFIG, TRUE,
                                                   configSpeedDuplexInputHelp3);
            }
            else
            {
                cpswPhyParam.speed = UserValueInfoGet(MIN_SPEED,
                                                      MAX_SPEED,
                                                      DEF_SPEED, TRUE,
                                                      speedInputHelp);
                cpswPhyParam.duplex = UserValueInfoGet(MIN_DUPLEX,
                                                       MAX_DUPLEX,
                                                       DEF_DUPLEX, TRUE,
                                                       duplexInputHelp);
            }
            cpsw_switch_configuration(&cpswConfig);
            return TRUE;
        }

        default:
            break;
    }

    return FALSE;
}

/*
** The main function
*/
int main(void)
{
    unsigned int ipAddr;
    unsigned int initFlg = 1;
    LWIP_IF lwipIfPort1, lwipIfPort2;

    MMUConfigAndEnable();

#ifdef LWIP_CACHE_ENABLED
    CacheEnable(CACHE_ALL);
#endif

    CPSWPinMuxSetup();
    CPSWClkEnable();

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();

    /* Select the console type based on compile time check */
    ConsoleUtilsSetType(CONSOLE_UART);

    /* Chip configuration RGMII selection */
    EVMPortMIIModeSelect();

    /* Get the MAC address */
    EVMMACAddrGet(0, lwipIfPort1.macArray); 
    EVMMACAddrGet(1, lwipIfPort2.macArray); 

    AintcCPSWIntrSetUp();
    DelayTimerSetup();

    ConsoleUtilsPrintf("\n\rStarterWare Ethernet Application. Access the"
             " embedded web page using http://<ip address assigned>/index.html"
             " via a web browser. \n\r\n\r");
   
    ConsoleUtilsPrintf("Acquiring IP Address for Port 1... \n\r" );

#if STATIC_IP_ADDRESS_PORT1

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1; 
    lwipIfPort1.ipAddr = STATIC_IP_ADDRESS_PORT1; 
    lwipIfPort1.netMask = 0; 
    lwipIfPort1.gwAddr = 0; 
    lwipIfPort1.ipMode = IPADDR_USE_STATIC; 

    ipAddr = lwIPInit(&lwipIfPort1);

#else

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1; 
    lwipIfPort1.ipAddr = 0; 
    lwipIfPort1.netMask = 0; 
    lwipIfPort1.gwAddr = 0; 
    lwipIfPort1.ipMode = IPADDR_USE_DHCP; 

    ipAddr = lwIPInit(&lwipIfPort1);

#endif
    if(ipAddr)
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Assigned: ");
        IpAddrDisplay(ipAddr);
    }
    else
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Acquisition Failed.");
    }

    /* Initialize the sample httpd server. */
    httpd_init();
   
    cpswConfig.phy_param = &cpswPhyParam;

    /* Loop forever.  For Switch Condigraton and interrupt handlers. */
    while(1)
    {
        unsigned int switchConfig = 0;
        unsigned char switchConfigInputHelp[] = "):  ";

        if(initFlg)
        {
            ConsoleUtilsPrintf("\n\r\n\r === CPSW Configurations === ");
            ConsoleUtilsPrintf("\n\r\n\r === Available PHY Configurations ===");
            ConsoleUtilsPrintf("\n\r\n\r1 - Configure Phy of a Port ");
            ConsoleUtilsPrintf("\n\r\n\r2 - Exit ");
            initFlg = 0;
        }

        if (!initFlg)
        {
            ConsoleUtilsPrintf("\n\r\n\r Select Switch Configuration (1 to %d",
                               CONFIG_SWITCH_EXIT_CMD);

            switchConfig = UserValueInfoGet(1, CONFIG_SWITCH_EXIT_CMD, 0, FALSE,
                                            switchConfigInputHelp);

            ConsoleUtilsPrintf("\n\r\n\rSwitch Configuration selected: %d\r\n",
                               switchConfig);

            if(EnetLWIPSwitchConfiguration(switchConfig))
                initFlg = 1;
        }

        if(switchConfig == CONFIG_SWITCH_EXIT_CMD)
            break;
    }

    /* Loop forever.  All the work is done in interrupt handlers. */
    while(1)
    {
        ; /* Perform nothing */
    }
}

/*
** Interrupt Handler for Core 0 Receive interrupt
*/
static void CPSWCore0RxIsr(void)
{
    lwIPRxIntHandler(0);
}

/*
** Interrupt Handler for Core 0 Transmit interrupt
*/
static void CPSWCore0TxIsr(void)
{
    lwIPTxIntHandler(0);
}

/*
** Displays the IP addrss on the Console
*/
static void IpAddrDisplay(unsigned int ipAddr) 
{
    ConsoleUtilsPrintf("%d.%d.%d.%d", (ipAddr & 0xFF), ((ipAddr >> 8) & 0xFF),
                       ((ipAddr >> 16) & 0xFF), ((ipAddr >> 24) & 0xFF));
}

/*
** Set up the ARM Interrupt Controller for generating timer interrupt
*/
static void AintcCPSWIntrSetUp(void)
{
    /* Enable IRQ for ARM (in CPSR)*/
    IntMasterIRQEnable();

    IntAINTCInit();
    
    /* Register the Receive ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWRXINT0, CPSWCore0RxIsr);
  
    /* Register the Transmit ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWTXINT0, CPSWCore0TxIsr);
    
    /* Set the priority */
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
}

/***************************** End Of File ***********************************/
