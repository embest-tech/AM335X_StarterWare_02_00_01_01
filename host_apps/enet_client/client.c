/**
*  \file client.c
*    
*  \brief This is a client program which connects to echoserver to
*        start communication with echoserver.
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

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#define MAX_PKT_SIZE 4*1024
#define MAX_NUM_PKTS 50


int main(int argc, char*argv[])
{
    int sock;
    unsigned int i;  
    int snd_err = 0;
    int rcv_err = 0;
    struct sockaddr_in server_addr;
    unsigned char *send_data, *recv_data;

    if(argc!=2)
    {
         printf("server ip needs to be passed as command line argument\n");
         return -1;
    }

    printf("\nNumber of bytes  to be transferred %d\n" , MAX_PKT_SIZE);

    /* Allocate send and receive buffers */
    send_data = calloc(MAX_PKT_SIZE, sizeof(char));
    recv_data = calloc(MAX_PKT_SIZE, sizeof(char));

    if (!send_data || !recv_data)
    {
        printf("Error ! No mem for buffers \n");
    }

    for (i = 0; i < MAX_PKT_SIZE; i++)
    {
	send_data[i] = i % 255 ;	
    }

    memset(recv_data, 0, MAX_PKT_SIZE);
  
    /* create a socket */ 
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
    {
            perror("Socket");
            exit(1);
    }

    server_addr.sin_family = AF_INET;     
    server_addr.sin_port = htons(2000);   
    server_addr.sin_addr.s_addr = inet_addr(argv[1]);
    bzero(&(server_addr.sin_zero),8); 

    printf("connecting .......\n");
    /* connect to the server */
    if (connect(sock, (struct sockaddr *)&server_addr,
                       sizeof(struct sockaddr)) == -1) 
    {
            perror("Connect");
            exit(1);
    }
   
    for(i = 0; i < MAX_NUM_PKTS ; i++)
    {
        unsigned int j;

        /* send data to server */    
        printf("\n(%d) sending data..........",i);

        snd_err = send(sock, send_data, MAX_PKT_SIZE, 0);

	if (snd_err < 0)
	    printf("\n send error : %s\n", strerror(snd_err));

        printf("\n(%d)receiving data..........\n",i);
    
        /* receive the data from server */
        rcv_err = recv(sock, recv_data, MAX_PKT_SIZE, MSG_WAITALL);

	if (rcv_err < 0)
	    printf("\n recv error %s\n", strerror(rcv_err));
 
 	printf("number of bytes received = %d\n" , rcv_err);

	for (j = 0; (j < MAX_PKT_SIZE) && (snd_err >= 0 && rcv_err >= 0); j++)
	{
	    if ( recv_data[j] != send_data[j])
	    {
		printf("Data error at %d for %d transfer \n", j, i);
		printf("send %d ; recv : %d \n", send_data[j], recv_data[j]);
		break;
	    }
        }
	
	if (j == MAX_PKT_SIZE)
        {
	    printf("Data passed for %d  transfer \n", i);
        }

        memset(recv_data, 0, MAX_PKT_SIZE);
    }

    return 0;
}
