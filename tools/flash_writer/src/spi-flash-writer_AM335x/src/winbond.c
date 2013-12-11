/*
 * (C) Copyright 2000-2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Copyright 2008, Texas Instrumemnts Inc.
 * Modified to suppport Winbond flash.
 *
 * Copyright 2008, Network Appliance Inc.
 * Jason McMullan <mcmullan@netapp.com>
 *
 * Copyright (C) 2004-2007 Freescale Semiconductor, Inc.
 * TsiChung Liew (Tsi-Chung.Liew@freescale.com)
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <stdio.h>
#include <stdlib.h>
#include "util.h"
#include "omap3_spi.h"

/* W25 specific commands. Derived from section 12.2 of 
 * http://www.winbond-usa.com/products/Nexflash/pdfs/datasheets/W25X16_32_64h.pdf 
 */
#define CMD_W25_WREN		0x06	/* Write Enable */
#define CMD_W25_WRDI		0x04	/* Write Disable */
#define CMD_W25_RDSR		0x05	/* Read Status Register */
#define CMD_W25_WRSR		0x01	/* Write Status Register */
#define CMD_W25_READ		0x03	/* Read Data Bytes */
#define CMD_W25_FAST_READ	0x0b	/* Read Data Bytes at Higher Speed */
#define CMD_W25_PP		0x02	/* Page Program */
#define CMD_W25_SE		0x20	/* Sector Erase */
#define CMD_W25_BE		0xd8	/* Bulk Erase */
#define CMD_W25_DP		0xb9	/* Deep Power-down */
#define CMD_W25_RES		0xab	/* Release from DP, and Read Signature */

#define WINBOND_ID_W25X16	      0x3015
#define WINBOND_ID_W25X32	      0x3016
#define WINBOND_ID_W25X64	      0x3017
#define WINBOND_ID_W25Q64	      0x4017

#define WINBOND_SR_WIP		(1 << 0)	/* Write-in-Progress */

/* W25 specific flash parameters. Derived from chapter 1 of
 * http://www.winbond-usa.com/products/Nexflash/pdfs/datasheets/W25X16_32_64h.pdf 
 */
static const struct winbond_spi_flash_params winbond_spi_flash_table[] = {
	{
		WINBOND_ID_W25X32,
		256,
		16,
		1024,
		"W25X32",
	},
	{
		WINBOND_ID_W25X16,
		256,
		16,
		512,
		"W25X16",
	},
	{
		WINBOND_ID_W25X64,
		256,
		16,
		2048,
		"W25X64",
	},
	{
		WINBOND_ID_W25Q64,  //REVISIT as per spec
		256,
		16,
		2048,
		"W25Q64",
	},
	
};

static int winbond_wait_ready(struct spi_flash *flash, unsigned long timeout) {
	struct spi_slave *spi = flash->spi;
	int ret;
	u8 status;
	u8 cmd[4] = { CMD_W25_RDSR, 0xff, 0xff, 0xff };

	ret = spi_xfer(spi, 32, &cmd[0], NULL, SPI_XFER_BEGIN);
	if (ret) {
		printf("SF: Failed to send command %02x: %d\n", cmd, ret);
		return ret;
	}

	do {
		ret = spi_xfer(spi, 8, NULL, &status, 0);
		if (ret)
			return -1;

		if ((status & WINBOND_SR_WIP) == 0)
			break;

		timeout--;
		if (!timeout)
			break;

	} while (1);

	spi_xfer(spi, 0, NULL, NULL, SPI_XFER_END);

	if ((status & WINBOND_SR_WIP) == 0)
		return 0;

    printf("SF: Timed out on command %02x: %d\n", cmd, ret);
	/* Timed out */
	return -1;
}

static int winbond_read_fast(struct spi_flash *flash,
			     u32 offset, size_t len, void *buf) {
	struct winbond_spi_flash *win = to_winbond_spi_flash(flash);
	u8 cmd[5];
	int i, j;

	cmd[0] = CMD_READ_ARRAY_FAST;
	for (i = 0, j = 1; i < 24; i += 8)
		cmd[j++] = ((offset >> (16 - i)) & 0xFF);
	cmd[4] = 0x00;

	return spi_flash_read_common(flash, cmd, sizeof(cmd), buf, len);
}

static int winbond_write(struct spi_flash *flash,
			 u32 offset, size_t len, unsigned char *buf) {
	struct winbond_spi_flash *win = to_winbond_spi_flash(flash);
	unsigned long page_addr;
	unsigned long byte_addr;
	unsigned long page_size;
	size_t chunk_len;
	size_t actual;
	int ret, i, j;
	u8 cmd[4];

	page_size = win->params->page_size;
	page_addr = offset / page_size;
	byte_addr = offset % page_size;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < len; actual += chunk_len) {
		   chunk_len = ((len - actual) < (page_size - byte_addr) ? 
		   				(len - actual) : (page_size - byte_addr));

		cmd[0] = CMD_W25_PP;
		for (i = 0, j = 1; i < 24; i += 8)
		    cmd[j++] = ((offset >> (16 - i)) & 0xFF);

		printf
		    ("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %d\n",
		     buf + actual, cmd[0], cmd[1], cmd[2], cmd[3], chunk_len);

		ret = spi_flash_cmd(flash->spi, CMD_W25_WREN, NULL, 0);
		if (ret < 0) {
			printf("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4,
					  buf + actual, chunk_len);
		if (ret < 0) {
			printf("SF: Winbond Page Program failed\n");
			break;
		}

		ret = winbond_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			printf("SF: Winbond page programming timed out\n");
			break;
		}

		page_addr++;
		offset += chunk_len;
		byte_addr = 0;
	}

	printf("SF: Winbond: Successfully programmed %u bytes @ 0x%x\n",
	      len, offset);

	spi_release_bus(flash->spi);
	return ret;
}

int winbond_erase(struct spi_flash *flash, u32 offset, size_t len) {
	struct winbond_spi_flash *win = to_winbond_spi_flash(flash);
	unsigned long sector_size;
	size_t actual;
	int ret, i , j;
	u8 cmd[4];

	/*
	 * This function currently uses sector erase only.
	 * probably speed things up by using bulk erase
	 * when possible.
	 */

	sector_size = win->params->page_size * win->params->pages_per_sector;

	if (offset % sector_size || len % sector_size) {
		printf("SF: Erase offset/length not multiple of sector size\n");
		return -1;
	}

	len /= sector_size;
	cmd[0] = CMD_W25_SE;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < len; actual++) {

		for (i = 0, j = 1; i < 24; i += 8)
		    cmd[j++] = ((offset >> (16 - i)) & 0xFF);

		printf
		    ("SE: cmd = { 0x%02x 0x%02x%02x%02x }\n",
		     cmd[0], cmd[1], cmd[2], cmd[3]);

		ret = spi_flash_cmd(flash->spi, CMD_W25_WREN, NULL, 0);
		if (ret < 0) {
			printf("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret < 0) {
			printf("SF: Winbond page erase failed\n");
			break;
		}

		/* Up to 2 seconds */
		ret = winbond_wait_ready(flash, 100 * CFG_HZ);
		if (ret < 0) {
			printf("SF: Winbond page erase timed out\n");
			break;
		}

		offset += sector_size;
	}

	printf("SF: Winbond: Successfully erased %u bytes @ 0x%x\n",
	      len * sector_size, offset);

	spi_release_bus(flash->spi);
	return ret;
}

struct spi_flash *spi_flash_probe_winbond(struct spi_slave *spi, u8 * idcode) {
	const struct winbond_spi_flash_params *params;
	struct winbond_spi_flash *win;
	unsigned int i;
	int ret;
	u8 id[3];
	u16 idmatch = ((idcode[1] << 8) | idcode[2]);

	ret = spi_flash_cmd(spi, CMD_READ_ID, id, sizeof(id));
	if (ret)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(winbond_spi_flash_table); i++) {
		params = &winbond_spi_flash_table[i];
		if (params->idcode == idmatch)
			break;
	}

	if (i == ARRAY_SIZE(winbond_spi_flash_table)) {
		printf("SF: Unsupported Winbond ID %02x\n", idmatch);
		return NULL;
	}

	win = (struct winbond_spi_flash *)AM335X_UTIL_allocMem(sizeof(struct winbond_spi_flash));
	if (!win) {
		printf("SF: Failed to allocate memory\n");
		return NULL;
	}

	win->params = params;
	win->flash.spi = spi;
	win->flash.name = params->name;

	win->flash.write = winbond_write;
	win->flash.erase = winbond_erase;
	win->flash.read = winbond_read_fast;
	win->flash.size = params->page_size * params->pages_per_sector
	    * params->nr_sectors;

	printf("SF: Detected %s with page size %u, total %u bytes\n",
	      params->name, params->page_size, win->flash.size);

	return &win->flash;
}
