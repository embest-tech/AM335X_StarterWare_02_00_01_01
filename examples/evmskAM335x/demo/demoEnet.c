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
#include "evmskAM335x.h"
#include "interrupt.h"
#include "cpsw.h"
#include "mdio.h"
#include "demoEnet.h"
#include "demoCfg.h"
#include "lwiplib.h"
#include "httpd.h"
#include "demoMain.h"
#include "lwipopts.h"
#include "consoleUtils.h"

#include <string.h>

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define LEN_IP_ADDR                          (4u)
#define ASCII_NUM_IDX                        (48u)
#define NUM_CONFIG_CGI_URIS                  (1)

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

unsigned int ipAddr1;
unsigned int ipAddr2;
static unsigned int  enetPort1InitFlag = FALSE;
static unsigned int  enetPort2InitFlag = FALSE;

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

/*
**  This function checks if the interface is up for Port 1 
*/
unsigned int EnetPort1IfIsUp(void)
{
   return ((lwIPNetIfStatusGet(0, 1)));
}

/*
**  This function checks if the interface is up for Port 2 
*/
unsigned int EnetPort2IfIsUp(void)
{
   return ((lwIPNetIfStatusGet(0, 2)));
}

/*
**  This function checks if the Link is active for Port 1 
*/
unsigned int EnetPort1LinkIsUp(void)
{
   return ((lwIPLinkStatusGet(0, 1)));
}

/*
**  This function checks if the Link is active for Port 2 
*/
unsigned int EnetPort2LinkIsUp(void)
{
   return ((lwIPLinkStatusGet(0, 2)));
}

/*
**  Intialize ethernet.
*/
void EthernetInit(void)
{
    EVMPortRGMIIModeSelect();
}

/*
** initializes the httpserver for Port 1
*/
void EnetPort1HttpServerInit(void)
{
    LWIP_IF lwipIfPort1;

    /* Get the MAC address */
    EVMMACAddrGet(0, lwipIfPort1.macArray);

    ConsoleUtilsPrintf("\n\rAcquiring IP Address for Port 1... \n\r");

#if STATIC_IP_ADDRESS_PORT1

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1;
    lwipIfPort1.ipAddr = STATIC_IP_ADDRESS_PORT1;
    lwipIfPort1.netMask = 0;
    lwipIfPort1.gwAddr = 0;
    lwipIfPort1.ipMode = IPADDR_USE_STATIC;

    ipAddr1 = lwIPInit(&lwipIfPort1);

#else

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1;
    lwipIfPort1.ipAddr = 0;
    lwipIfPort1.netMask = 0;
    lwipIfPort1.gwAddr = 0;
    lwipIfPort1.ipMode = IPADDR_USE_DHCP;

    ipAddr1 = lwIPInit(&lwipIfPort1);

#endif
    if(ipAddr1)
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Assigned: ");
        IpAddrDisplay(ipAddr1);
    }
    else
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Acquisition Failed.");
    }
}

/*
** initializes the CPSW Port 2
*/
void EnetPort2HttpServerInit(void)
{    
    LWIP_IF lwipIfPort2;

    EVMMACAddrGet(1, lwipIfPort2.macArray);

    ConsoleUtilsPrintf("\n\r\n\rAcquiring IP Address for Port 2... \n\r");

#if STATIC_IP_ADDRESS_PORT2

    lwipIfPort2.instNum = 0;
    lwipIfPort2.slvPortNum = 2;
    lwipIfPort2.ipAddr = STATIC_IP_ADDRESS_PORT2;
    lwipIfPort2.netMask = 0;
    lwipIfPort2.gwAddr = 0;
    lwipIfPort2.ipMode = IPADDR_USE_STATIC;

    ipAddr2 = lwIPInit(&lwipIfPort2);

#else

    lwipIfPort2.instNum = 0;
    lwipIfPort2.slvPortNum = 2;
    lwipIfPort2.ipAddr = 0;
    lwipIfPort2.netMask = 0;
    lwipIfPort2.gwAddr = 0;
    lwipIfPort2.ipMode = IPADDR_USE_DHCP;

    ipAddr2 = lwIPInit(&lwipIfPort2);

#endif
    if(ipAddr2)
    {
        enetPort2InitFlag = TRUE;
        ConsoleUtilsPrintf("\n\r\n\rPort 2 IP Address Assigned: ");
        IpAddrDisplay(ipAddr2);
    }
    else
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 2 IP Address Acquisition Failed.");
    }

    /* Initialize the sample httpd server. */
    httpd_init();

    http_set_cgi_handlers(g_psConfigCGIURIs, NUM_CONFIG_CGI_URIS);
}

/*
** This function restarts DHCP 
*/
void EnetDHCPRestart(void)
{
    if(enetPort1InitFlag == TRUE)
    { 
        if(lwIPLinkStatusGet(0, 1))
        {
            ipAddr1 = lwIPDHCPStart(0, 1);

            if(ipAddr1)
            {
                ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Assigned: ");
                IpAddrDisplay(ipAddr1);
            }
  
            else
            {         
                ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Acquisition Failed.");
            }
        }
        else
        {
            ConsoleUtilsPrintf("\n\rLink is down on Port 1. ");
            ConsoleUtilsPrintf("Unable to acquire IP address.");
        }
    }

    if(enetPort2InitFlag == TRUE)
    {
        if(lwIPLinkStatusGet(0, 2))
        {
            ipAddr2 = lwIPDHCPStart(0, 2);

            if(ipAddr2)
            {
                ConsoleUtilsPrintf("\n\r\n\rPort 2 IP Address Assigned: ");
                IpAddrDisplay(ipAddr2);
            }
  
            else
            {         
                ConsoleUtilsPrintf("\n\r\n\rPort 2 IP Address Acquisition Failed.");
            }
        }
        else
        {
            ConsoleUtilsPrintf("\n\rLink is down on Port 2. ");
            ConsoleUtilsPrintf("Unable to acquire IP address.");
        }
    }
}

/*
** CGI handler 
*/
static const char* ControlCGIHandler(int iIndex, int iNumParams, char *pcParam[],
                                     char *pcValue[])
{
    if(!(strcmp(pcValue[0],"INTRO")))
    {
        clickIdx = CLICK_IDX_INTRO;    
    }
    else if(!(strcmp(pcValue[0],"WWW")))
    {
        clickIdx = CLICK_IDX_CHOICE;    
    }
    else if(!(strcmp(pcValue[0],"MCASP")))
    {
        clickIdx = CLICK_IDX_MCASP;    
    }
    else if(!(strcmp(pcValue[0],"MMCSD")))
    {
        clickIdx = CLICK_IDX_MMCSD;    
    }
    else if(!(strcmp(pcValue[0],"UART")))
    {
        clickIdx = CLICK_IDX_UART;    
    }
    else if(!(strcmp(pcValue[0],"RTC")))
    {
        clickIdx = CLICK_IDX_RTC;    
    }
    else if(!(strcmp(pcValue[0],"TIMER")))
    {
        clickIdx = CLICK_IDX_TIMER;    
    }
    else if(!(strcmp(pcValue[0],"ETHERNET")))
    {
        clickIdx = CLICK_IDX_ETHERNET;    
    }
    else if(!(strcmp(pcValue[0],"ECAP")))
    {
        clickIdx = CLICK_IDX_ECAP;    
    }	
    else if(!(strcmp(pcValue[0],"GPIO")))
    {
        clickIdx = CLICK_IDX_GPIO;    
    }
    else if(!(strcmp(pcValue[0],"I2C")))
    {
        clickIdx = CLICK_IDX_I2C;    
    }
    else if(!(strcmp(pcValue[0],"PM")))
    {
        clickIdx = CLICK_IDX_PM;    
    }
    else if(!(strcmp(pcValue[0],"DVFS")))
    {
        clickIdx = CLICK_IDX_DVFS;    
    }
    else
    {
        clickIdx = 0;
    }
 
    return "/io_cgi.ssi";
}


/*
** Displays the IP addrss on the Console
*/
void IpAddrDisplay(unsigned int ipAddr)
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
