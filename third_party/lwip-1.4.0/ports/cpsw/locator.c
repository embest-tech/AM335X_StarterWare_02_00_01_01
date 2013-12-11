/*
 * locator.c
 *
 * Device Locator server using UDP
 *
*/
/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
*/

/* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */

#include "locator.h"
#include "lwiplib.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TAG_CMD                      (0xff)
#define TAG_STATUS                   (0xfe)
#define CMD_DISCOVER_TARGET          (0x02)

/******************************************************************************
**                    INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
/*
** An array that contains the device locator response data.  The format of the
** data is as follows:
**
**     Byte        Description
**     --------    ------------------------
**      0          TAG_STATUS
**      1          packet length
**      2          CMD_DISCOVER_TARGET
**      3          board type
**      4          board ID
**      5..8       client IP address
**      9..14      MAC address
**     15..18      firmware version
**     19..82      application title
**     83          checksum
*/
static unsigned char locatorData[84];

/******************************************************************************
**                    INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void LocatorReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                           struct ip_addr *addr, u16_t port);

/******************************************************************************
**                        FUNCTION DEFINITIONS
*******************************************************************************/
/*
** This function is called by the lwIP TCP/IP stack when it receives a UDP
** packet from the discovery port.  It produces the response packet, which is
** sent back to the querying client.
*/
static void LocatorReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                           struct ip_addr *addr, u16_t port)
{
    unsigned char *pucData;
    unsigned long ulIdx;

    /* Validate the contents of the datagram.*/
    pucData = p->payload;
    if((p->len != 4) || (pucData[0] != TAG_CMD) || (pucData[1] != 4) ||
       (pucData[2] != CMD_DISCOVER_TARGET) ||
       (pucData[3] != ((0 - TAG_CMD - 4 - CMD_DISCOVER_TARGET) & 0xff)))
    {
        pbuf_free(p);
        return;
    }

    pbuf_free(p);

    p = pbuf_alloc(PBUF_TRANSPORT, sizeof(locatorData), PBUF_RAM);
    if(p == NULL)
    {
        return;
    }

    /* Calcuate and fill in the checksum on the response packet.*/
    for(ulIdx = 0, locatorData[sizeof(locatorData) - 1] = 0;
        ulIdx < (sizeof(locatorData) - 1); ulIdx++)
    {
        locatorData[sizeof(locatorData) - 1] -=
            locatorData[ulIdx];
    }

    /* Copy the response packet data into the pbuf. */
    pucData = p->payload;
    for(ulIdx = 0; ulIdx < sizeof(locatorData); ulIdx++)
    {
        pucData[ulIdx] = locatorData[ulIdx];
    }

    /* Send the response.*/
    udp_sendto(pcb, p, addr, port);

    pbuf_free(p);
}

/*
** Initializes the locator service. Prepares the locator service to
** handle device discovery requests.  .
*/
void LocatorConfig(unsigned char *macArray, const char *appTitle)
{
    unsigned int idx;
    void *pcb;

    /* Clear out the response data.*/
    for(idx = 0; idx < 84; idx++)
    {
        locatorData[idx] = 0;
    }

    /* Fill in the header for the response data.*/
    locatorData[0] = TAG_STATUS;
    locatorData[1] = sizeof(locatorData);
    locatorData[2] = CMD_DISCOVER_TARGET;

    /* Fill in the MAC address for the response data. */
    locatorData[9] =  macArray[0];
    locatorData[10] = macArray[1];
    locatorData[11] = macArray[2];
    locatorData[12] = macArray[3];
    locatorData[13] = macArray[4];
    locatorData[14] = macArray[5];

    /* Create a new UDP port for listening to device locator requests.*/
    pcb = udp_new();
    udp_recv(pcb, LocatorReceive, NULL);
    udp_bind(pcb, IP_ADDR_ANY, 23);
    udp_connect(pcb, IP_ADDR_ANY, 23);

    /* Copy the application title string into the response data. */
    for(idx = 0; (idx < 64) && *appTitle; idx++)
    {
        locatorData[idx + 19] = *appTitle++;
    }

    /* Zero-fill the remainder of the space in the response data (if any).*/
    for(; idx < 64; idx++)
    {
        locatorData[idx + 19] = 0;
    }

}
/***************************** End Of File ***********************************/

