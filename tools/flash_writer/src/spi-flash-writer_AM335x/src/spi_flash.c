/*
 * SPI flash interface
 *
 * Copyright (C) 2008 Atmel Corporation
 * 
 * This file uses source obtained from uboot which was licensed under
 * GPL v2 at the time this source was obtained, using the following
 * commit id:
 * http://git.denx.de/?p=u-boot.git;a=blob;f=drivers/mtd/spi/spi_flash.c;hb=7b7a869a8ba3bd6d9bffb748c91232141330f514
 */
#include <stdio.h>
#include <stdlib.h>
#include "../inc/omap3_spi.h"

int spi_flash_cmd(struct spi_slave *spi, Uint8 cmd, void *response, Uint32 len) {
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, 8, &cmd, NULL, flags);
	if (ret) {
		printf("SF: Failed to send command %02x: %d\n", cmd, ret);
		return ret;
	}

	if (len) {
		ret = spi_xfer(spi, len * 8, NULL, response, SPI_XFER_END);
		if (ret)
			printf("SF: Failed to read response (%zu bytes): %d\n",
					len, ret);
	}

	return ret;
}

int spi_flash_cmd_read(struct spi_slave *spi, const Uint8 *cmd,
		Uint32 cmd_len, void *data, Uint32 data_len) {
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
	if (ret) {
		printf("SF: Failed to send read command (%zu bytes): %d\n",
				cmd_len, ret);
	} else if (data_len != 0) {
		ret = spi_xfer(spi, data_len * 8, NULL, data, SPI_XFER_END);
		if (ret)
			printf("SF: Failed to read %zu bytes of data: %d\n",
					data_len, ret);
	}

	return ret;
}

int spi_flash_cmd_write(struct spi_slave *spi, const Uint8 *cmd, Uint32 cmd_len,
		const void *data, Uint32 data_len) {
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, cmd_len * 8, cmd, NULL, flags);
	if (ret) {
		printf("SF: Failed to send read command (%zu bytes): %d\n",
				cmd_len, ret);
	} else if (data_len != 0) {
		ret = spi_xfer(spi, data_len * 8, data, NULL, SPI_XFER_END);
		if (ret)
			printf("SF: Failed to read %zu bytes of data: %d\n",
					data_len, ret);
	}

	return ret;
}


int spi_flash_read_common(struct spi_flash *flash, const Uint8 *cmd,
		Uint32 cmd_len, void *data, Uint32 data_len) {
	struct spi_slave *spi = flash->spi;
	int ret;

	spi_claim_bus(spi);
	ret = spi_flash_cmd_read(spi, cmd, cmd_len, data, data_len);
	spi_release_bus(spi);

	return ret;
}

struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode) {
	struct spi_slave *spi;
	struct spi_flash *flash;
	int ret;
	Uint8 idcode[3];

	spi = (struct spi_slave *)spi_setup_slave(bus, cs, max_hz, spi_mode);
	if (!spi) {
		printf("SF: Failed to set up slave\n");
		return NULL;
	}

	ret = spi_claim_bus(spi);
	if (ret) {
		printf("SF: Failed to claim SPI bus: %d\n", ret);
		goto err_claim_bus;
	}

	/* Read the ID codes */
	ret = spi_flash_cmd(spi, CMD_READ_ID, &idcode, sizeof(idcode));
	if (ret)
		goto err_read_id;

	printf("SF: Got idcode %02x %02x %02x\n", idcode[0],
			idcode[1], idcode[2]);

	switch (idcode[0]) {
#ifdef CONFIG_SPI_FLASH_SPANSION
	case 0x01:
		flash = spi_flash_probe_spansion(spi, idcode);
		break;
#endif
#ifdef CONFIG_SPI_FLASH_ATMEL
	case 0x1F:
		flash = spi_flash_probe_atmel(spi, idcode);
		break;
#endif
#ifdef CONFIG_SPI_FLASH_WINBOND
	case 0xef:
		flash = spi_flash_probe_winbond(spi, idcode);
		break;
#endif
#ifdef CONFIG_SPI_FLASH_STMICRO
	case 0x20:
		flash = spi_flash_probe_stmicro(spi, idcode);
		break;
#endif
	default:
		printf("SF: Unsupported manufacturer %02X\n", idcode[0]);
		flash = NULL;
		break;
	}

	if (!flash)
		goto err_manufacturer_probe;

	spi_release_bus(spi);

	return flash;

err_manufacturer_probe:
err_read_id:
	spi_release_bus(spi);
err_claim_bus:
	spi_free_slave(spi);
	return NULL;
}

void spi_flash_free(struct spi_flash *flash) {
	spi_free_slave(flash->spi);
	free(flash);
}
