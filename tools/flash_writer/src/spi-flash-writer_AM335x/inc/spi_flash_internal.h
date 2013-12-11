/*
 * SPI flash internal definitions
 *
 * Copyright (C) 2008 Atmel Corporation
 * 
 * This file uses source obtained from uboot which was licensed under
 * GPL v2 at the time this source was obtained, using the following
 * commit id:
 * http://git.denx.de/?p=u-boot.git;a=blob;f=drivers/mtd/spi/spi_flash_internal.h;hb=6d0f6bcf337c5261c08fabe12982178c2c489d76
 *
 */
/* ------------------------------------------------------------------------ *
 *  Variable types                                                          *
 * ------------------------------------------------------------------------ */
#define Uint32  unsigned int
#define Uint16  unsigned short
#define Uint8   unsigned char
#define Int32   int
#define Int16   short
#define Int8    char

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

/* Common parameters */
#define	CFG_HZ				1000
#define SPI_FLASH_PROG_TIMEOUT		((100000 * CFG_HZ) / 1000)
#define SPI_FLASH_PAGE_ERASE_TIMEOUT	((500000 * CFG_HZ) / 1000)
#define SPI_FLASH_SECTOR_ERASE_TIMEOUT	(100000 * CFG_HZ)


/* SPI mode flags */
#define SPI_CPHA         0x01            /* clock phase */
#define SPI_CPOL         0x02            /* clock polarity */
#define SPI_MODE_0       (0|0)           /* (original MicroWire) */
#define SPI_MODE_1       (0|SPI_CPHA)
#define SPI_MODE_2       (SPI_CPOL|0)
#define SPI_MODE_3       (SPI_CPOL|SPI_CPHA)
#define SPI_CS_HIGH      0x04            /* CS active high */
#define SPI_LSB_FIRST    0x08            /* per-word bits-on-wire */
#define SPI_3WIRE        0x10            /* SI/SO signals shared */
#define SPI_LOOP         0x20            /* loopback mode */

/* Common commands */
#define CMD_READ_ID		0x9f

#define CMD_READ_ARRAY_SLOW	0x03
#define CMD_READ_ARRAY_FAST	0x0b
#define CMD_READ_ARRAY_LEGACY	0xe8

