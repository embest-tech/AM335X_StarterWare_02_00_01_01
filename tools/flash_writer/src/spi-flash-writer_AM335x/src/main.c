/*
 * Copyright (C) 2008 Texas Instruments, Inc <www.ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "util.h"

#include "../inc/device.h"
#include "../inc/omap3_spi.h"
#include "board_identification.h"

static Uint8 tx[512];
static Uint8 rx[512];
struct spi_slave *slave;
struct spi_flash *flash;

static int getPlatform(void); 

#ifdef CONFIG_TI816X_EVM
#define MAX_DSPUBL_SIZE	(0)
#define MAX_ARMUBL_SIZE	(0)
#else
#define MAX_DSPUBL_SIZE	(8*1024)
#define MAX_ARMUBL_SIZE	(24*1024)
#endif

#define JTAG_ID_BASE    (CFG_MOD_BASE + 0x600)	
#define CHIPREV_ID_BASE	0x01c14024
#define AM335X_PART_NUM	0xB944

#ifndef max
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

typedef struct _SPIBOOT_HEADER_
{
	Uint32		magicNum;
	Uint32		entryPoint;	
	Uint32		appSize;
	Uint32		ldAddress;	/* Starting RAM address where image is to copied - XIP Mode */
} SPIBOOT_HeaderObj;

#ifndef CONFIG_TI816X_EVM
/*
 * Get Device Part No. from JTAG ID register
 */
static Uint16 am335x_get_part_no(void)
{
        Uint32 dev_id, part_no;

        dev_id = *(unsigned int*) JTAG_ID_BASE;

        part_no = ((dev_id >> 12) & 0xffff);

        return part_no;
}
#endif

static pin_muxing_t PAD_Conf_SPI0Flash[] = {
	{ SPI0_SCLK_OFF, MODE(0) | RXACTIVE},
	{ SPI0_D0_OFF, MODE(0) | PULL_UP_EN | RXACTIVE},
	{ SPI0_D1_OFF, MODE(0) | RXACTIVE},
	{ SPI0_CS0_OFF, MODE(0) | PULL_UP_EN | RXACTIVE},
	{ 0xFFFFFFFF},
};

static pin_muxing_t PAD_Conf_SPI1Flash[] = {
	{ SPI1_SCLK_OFF, MODE(3)},
	{ SPI1_D0_OFF, MODE(3) | RXACTIVE},
	{ SPI1_D1_OFF, MODE(3) },
	{ SPI1_CS0_OFF, MODE(3)},
	{ 0xFFFFFFFF},
};

static pin_muxing_t PAD_Conf_I2C0 [] = {
	{I2C0_SD0_OFF, MODE(0) | RXACTIVE | PULL_UP_EN },
	{I2C0_SCL_OFF, MODE(0) | PULL_UP_EN},
	{0xFFFFFFFF},
};

void PAD_ConfigMux(pin_muxing_t	*pin_mux)
{
	Uint8 i;

	for (i = 0; i < 50; i++) {
		if(pin_mux[i].offset == 0xffffffff)
			break;
		*(volatile Uint32 *)(CFG_MOD_BASE + pin_mux[i].offset) = pin_mux[i].val;
	}

}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  main( )                                               		            *
 *      write to SPI flash and then verify the contents 					*
 *                                                                          *
 * ------------------------------------------------------------------------ */
Int32 main( )
{
	Int16   i;
	Uint8*  p8;
	Uint16  page_size;
	FILE	*fPtr;
   	Int32	fileSize = 0;
	Int32   no_of_pages;  
	Int32   no_of_sectors;
	Int32	offset = -1;
	Int8	fileName[256];
	Uint8 	idcode[3];
	Uint32	ret;
	Uint32  sector_size;
	Uint8   board;

	board = getPlatform();
	AM335X_UTIL_setCurrMemPtr(0);
	/* read the chip type you are on. */
	if (am335x_get_part_no() == AM335X_PART_NUM) {	//FIXME PARTNNUM
		printf("AM335X part detected. ");
	}

	/* Initialize the SPI interface */
	if(board == GP_BOARD) {
		*(unsigned int*)(CM_PER_SPI0_CLKCTRL) = PRCM_MOD_EN;		//TODO
		PAD_ConfigMux(PAD_Conf_SPI0Flash);
		slave = (struct spi_slave *)spi_setup_slave(0, 0, (24*1000*1000), 0);  //REVISIT
	} else if(board == GP_BOARD) {
		*(unsigned int*)(CM_PER_SPI1_CLKCTRL) = PRCM_MOD_EN;
		PAD_ConfigMux(PAD_Conf_SPI1Flash);
		slave = (struct spi_slave *)spi_setup_slave(1, 0, (24*1000*1000), 0);  //REVISIT
	}

	spi_claim_bus(slave);

	/* Read the ID codes */
	ret = spi_flash_cmd(slave, CMD_READ_ID, idcode, sizeof(idcode));
	if (ret) {
		printf ( "SF: Error in reading the idcode\n");
		exit (1);
	}

	printf("SF: Got idcode %02x %02x %02x\n", idcode[0],
			idcode[1], idcode[2]);

	printf("Checking if Winbond flash writer can be used..\n");
	flash = (struct spi_flash *)spi_flash_probe_winbond (slave, idcode);
	if (flash == NULL) {
		printf ("No known Serial Flash found\n");
		exit (2);
	}

	if (strstr(flash->name, "W25")) {
		struct winbond_spi_flash *sf = to_winbond_spi_flash(flash);
		page_size = sf->params->page_size;
		sector_size = sf->params->page_size *
					sf->params->pages_per_sector;
	} else {
		printf("Unsupported flash type: %s\n", flash->name);
		exit (2);
	}

	printf("Flash page size: %d bytes\n", page_size);
	printf("Flash sector size: %d bytes\n", sector_size);

	printf( "Starting SPIWriter.\r\n");

	/* user need to privde offset for ti816x */
	offset = -1;


  	// Read the file from host
	printf("Enter the File Name\n");
	scanf("%s", fileName);
	fflush(stdin);

	// Read the offset from user
	if(offset == -1) {		
		printf("Enter the Offset in bytes (in HEX)\n");
		scanf("%x", &offset);
		fflush(stdin);
	}

	// Open an File from the hard drive
	fPtr = fopen(fileName, "rb");
	if(fPtr == NULL)
	{
		printf("File %s Open failed\n", fileName);
		exit (2);
	}

	// Initialize the pointer
	fileSize = 0;

	// Read file size
	fseek(fPtr,0,SEEK_END);
	fileSize = ftell(fPtr);

	if(fileSize == 0)
	{
		printf("File read failed.. Closing APP\n");
		fclose (fPtr);
		exit (2);
	}
	fseek(fPtr,0,SEEK_SET);

	no_of_pages = fileSize/page_size + ((fileSize % page_size) ? 1 : 0);
	no_of_sectors = (fileSize/sector_size + ((fileSize % sector_size) ? 1 : 0));

	/* Erase the Serial Flash */
	printf("Erasing flash at byte offset: %d, byte length: %d\n",
			(offset/sector_size)*sector_size, no_of_sectors*sector_size);
	flash->erase (flash, (offset/sector_size)*sector_size,
			no_of_sectors*sector_size);

    /* Write 1 page at a time */
	printf("Writing flash at page offset: %d, number of pages: %d\n", (offset/page_size), no_of_pages);
	
	i = (offset/page_size);

	while(!feof(fPtr)) {
		p8 = (Uint8*) tx;
	   	if(!feof(fPtr)) {
	   		fread(p8, 1, page_size, fPtr);
		}
		flash->write (flash, i * page_size, page_size, ( Uint8 *)tx);
		i++;
	}

	fseek(fPtr,0,SEEK_SET);

	printf("Verifying... ");
	fflush(stdout);

	i = (offset/page_size);

	while(!feof(fPtr)) {
		unsigned int nbytes;

	   	if(!feof(fPtr)) {
			nbytes = fread(tx, 1, page_size, fPtr);
		}		
		tx[nbytes] = '\0';

		flash->read (flash, i * page_size, page_size, (Uint8 *)rx);
		rx[nbytes] = '\0'; 
		if(strcmp((const char *)rx,(const char *)tx)!= 0) {
			printf("Did not match @ %d\n", ftell(fPtr));
			goto finish;
		}
		i++;
	}

	printf("Success. \n");

finish:
	fclose(fPtr);
	return 0;
}

static int getPlatform(void) {
	int board = -1,profile = -1;

	printf("Welcome to CCS SPI Flash Utility \n");
	PAD_ConfigMux(PAD_Conf_I2C0);
	board   = i2c_daughter_card_detection();
	if( (board == GP_BOARD) || (board == IA_BOARD))
		profile = 1 << profile_identification();
	else{
		printf("Closing CCS SPI Flash Utility \n");
		printf("SPI NOR FLASH device is not supported.\n");
		printf("board <%d>.\n",board);
		exit(1);
	}

	switch(board){
		case GP_BOARD:
			if(profile == PROFILE_2)
				break;
			else{
				printf("Closing CCS SPI Flash Utility \n");
				printf("SPI FLASH device is not supported.\n");
				printf("board <%d>, Profile <%d>.\n",
						board,profile);
				exit(1);
			}
		case IA_BOARD:
			break;
		default:
		case BASE_BOARD:
		case IP_BOARD:
			printf("Closing CCS SPI Flash Utility \n");
			printf("SPI FLASH device is not supported.\n");
			printf("board <%d>, Profile <%d>.\n",board,profile);
			exit(1);
	}
	return board;
}
