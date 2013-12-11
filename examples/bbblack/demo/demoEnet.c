/**
 * \file    demoEnet.c
 *
 * \brief   This file contains Ethernet related functions.
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

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "interrupt.h"
#include "cpsw.h"
#include "mdio.h"
#include "demoEnet.h"
#include "demoCfg.h"
#include "lwiplib.h"
#include "httpd.h"
#include "demoMain.h"
#include "lwipopts.h"
#include "cpsw.h"
#include "consoleUtils.h"

#include <string.h>

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LEN_IP_ADDR                          (4u)
#define ASCII_NUM_IDX                        (48u)
#define NUM_CONFIG_CGI_URIS                  (1)
#define CPSW_STAT_TX_COL                     (0x48)
#define CPSW_STAT_RX_CRC_ERR                 (0x10)
#define CPSW_STAT_RX_ALIGN_CODE              (0x14)

/******************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void CPSWCore0RxIsr(void);
static void CPSWCore0TxIsr(void);
static const char* ControlCGIHandler(int iIndex, int iNumParams, char *pcParam[],
                                     char *pcValue[]);
/*******************************************************************************
**                     INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static const tCGI g_psConfigCGIURIs[] =
{
    { "/io_control.cgi", ControlCGIHandler }      
};

unsigned int ipAddr;
unsigned int  enetInitFlag = FALSE;

CPSWCONTEXT cpswContext;
MDIOCONTEXT mdioContext;

/*******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/* 
** Registers ethernet ISRs
*/
void EnetIntRegister(void)
{
    /* Register the Receive ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWRXINT0, CPSWCore0RxIsr);

    /* Register the Transmit ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWTXINT0, CPSWCore0TxIsr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
}

unsigned int EnetIfIsUp(void)
{
   return (lwIPNetIfStatusGet(0, 1));
}

unsigned int EnetLinkIsUp(void)
{

   return (lwIPLinkStatusGet(0, 1));
}

/*
** initializes the httpserver and prints the IP address on the UART console
*/
void EnetHttpServerInit(void)
{
    LWIP_IF lwipIfPort1;

    /* Chip configuration RGMII selection */
    EVMPortMIIModeSelect();

    /* Get the MAC address */
    EVMMACAddrGet(0, lwipIfPort1.macArray);

    ConsoleUtilsPrintf("\n\rAcquiring IP Address for Port 1... \n\r" );

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
        IpAddrDisplay();
    }
    else
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Acquisition Failed.");
    }

    http_set_cgi_handlers(g_psConfigCGIURIs, NUM_CONFIG_CGI_URIS);

    /* Initialize the sample httpd server. */
    httpd_init();

    enetInitFlag = TRUE;
}



/*
** CGI handler 
*/
static const char* ControlCGIHandler(int iIndex, int iNumParams, char *pcParam[],
                                     char *pcValue[])
{
    if(!(strcmp(pcValue[0],"TIMER")))
    {
        clickIdx = MENU_IDX_TIMER;
    }

    else if(!(strcmp(pcValue[0],"LED")))
    {
        clickIdx = MENU_IDX_LED;
    }

    else if(!(strcmp(pcValue[0],"RTC")))
    {
        clickIdx = MENU_IDX_RTC;
    }

    else if(!(strcmp(pcValue[0],"MMCSD")))
    {
        clickIdx = MENU_IDX_SD;
    }
    else if(!(strcmp(pcValue[0],"PM")))
    {
        clickIdx = MENU_IDX_PM;
    }
    else if(!(strcmp(pcValue[0],"WWW")))
    {
        clickIdx = MENU_IDX_WWW;
    }
    else
    {
        clickIdx = 0;
    }
 
    return "/io_cgi.ssi";
}


/*
** Displays the IP addrss on the UART Console
*/
void IpAddrDisplay(void)
{
    ConsoleUtilsPrintf("%d.%d.%d.%d", (ipAddr & 0xFF), ((ipAddr >> 8) & 0xFF),
                       ((ipAddr >> 16) & 0xFF), ((ipAddr >> 24) & 0xFF));
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
** Returns the Error Statistics
*/
void EnetErrStatsticsGet(unsigned int *tx, unsigned int *rx)
{
    /* Transmit collitions */
    *tx = CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_TX_COL);

    /* Receive CRC errors and alignment/code errors */
    *rx = (CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_RX_CRC_ERR)
           + CPSWStatisticsGet(SOC_CPSW_STAT_REGS, CPSW_STAT_RX_ALIGN_CODE));
}

/****************************** End of file **********************************/


/*
** This function restarts DHCP
*/
void EnetDHCPRestart(void)
{
    if(enetInitFlag == TRUE)
    {
        if(lwIPLinkStatusGet(0, 1))
        {
            ConsoleUtilsPrintf("\n\r Acquiring IP Address...");

            ipAddr = lwIPDHCPStart(0, 1);

            if(ipAddr)
            {
                ConsoleUtilsPrintf("\n\r EVM IP Address Assigned: ");

                IpAddrDisplay();
            }

            else
            {
                ConsoleUtilsPrintf("\n\rIP Address not assigned. ");
                ConsoleUtilsPrintf("Check connection/DHCP.\n\r");
            }
        }

        else
        {
            ConsoleUtilsPrintf("\n\rEthernet Link is down. ");
            ConsoleUtilsPrintf("Unable to acquire IP address.");
        }
    }
}

/*
** Saves the CPSW and MDIO register context. This can be used while going
** to power down mode where CPSW power will be cut down.
*/
void EnetContextSave(void)
{
    MDIOContextSave(CPSW_MDIO_BASE_ADDR, &mdioContext);
    cpswContext.aleBase = CPSW_ALE_BASE_ADDR;
    cpswContext.ssBase = CPSW_SS_BASE_ADDR;
    cpswContext.port1Base = CPSW_PORT1_BASE_ADDR;
    cpswContext.port2Base = CPSW_PORT2_BASE_ADDR;
    cpswContext.cpdmaBase = CPSW_CPDMA_BASE_ADDR;
    cpswContext.wrBase = CPSW_WR_BASE_ADDR;
    cpswContext.sl1Base = CPSW_SLIVER1_BASE_ADDR;
    cpswContext.sl2Base = CPSW_SLIVER2_BASE_ADDR;
    cpswContext.cppiRamBase = CPSW_CPPI_RAM_BASE_ADDR;
    CPSWContextSave(&cpswContext);
}

/*
** Resets all modules of CPSW subsystem
*/
void EnetReset(void)
{
    CPSWSSReset(CPSW_SS_BASE_ADDR);
    CPSWSlReset(CPSW_SLIVER1_BASE_ADDR);
    CPSWSlReset(CPSW_SLIVER2_BASE_ADDR);
    CPSWWrReset(CPSW_WR_BASE_ADDR);
    CPSWCPDMAReset(CPSW_CPDMA_BASE_ADDR);
}
