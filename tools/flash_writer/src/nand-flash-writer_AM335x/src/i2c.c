/*
 * Basic I2C functions
 *
 * Copyright (c) 2004 Texas Instruments
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Jian Zhang jzhang@ti.com, Texas Instruments
 *
 * Copyright (c) 2003 Wolfgang Denk, wd@denx.de
 * Rewritten to fit into the current U-Boot framework
 *
 * Adapted for OMAP2420 I2C, r-woodruff2@ti.com
 *
 */
#define udelay UTIL_waitLoop

#include "i2c.h"
#include "tistdtypes.h"
#include "stdio.h"
#include "util.h"
#include "AM335x_device.h"

static void wait_for_bb (void);
static Int16 wait_for_pin (void);
static void flush_fifo(void);



void I2C_writew(  Uint32 val, Uint32 offset) {
    *(volatile Uint32 *)( offset ) = val;
}

void I2C_writeb(  Uint8 val, Uint32 offset) {
    *((volatile Uint8 *)( offset )) = val;
}

Uint32 I2C_readw(  Uint32 offset) {
    return *(volatile Uint32 *)( offset );
}
#define I2C_IP_CLK                48000000
#define I2C_INTERNAL_SAMPLING_CLK 12000000 
void i2c_init (int speed, int slaveadd) {
	int psc, fsscll, fssclh;
	int hsscll = 0, hssclh = 0;
	Int32 scll, sclh;
	Int32 reg;


	*(volatile Uint32 *)CM_PER_I2C0_CLKCTRL = PRCM_MOD_EN;
	while (*(volatile Uint32 *) CM_PER_I2C0_CLKCTRL != PRCM_MOD_EN);

	/* Only handle standard, fast and high speeds */
	if ((speed != OMAP_I2C_STANDARD) &&
	    (speed != OMAP_I2C_FAST_MODE) &&
	    (speed != OMAP_I2C_HIGH_SPEED)) {
		printf("Error : I2C unsupported speed %d\n", speed);
		return;
	}

	I2C_writew(I2C_readw(I2C_CON) &~I2C_CON_EN, I2C_CON);
	I2C_writew( I2C_SYST_RESET, I2C_SYSC);
	I2C_writew(I2C_CON_EN, I2C_CON);
	while(!(I2C_readw(I2C_SYSS) & I2C_SYST_RESETDONE))
	       udelay(1000);
	reg = I2C_readw( I2C_SYSC);
	reg &= ~I2C_SYST_AUTOIDLE;
	reg |= I2C_SYST_NOIDLE;
	I2C_writew( reg , I2C_SYSC);

	psc = I2C_IP_CLK / I2C_INTERNAL_SAMPLING_CLK;
	psc -= 1;
	if (psc < I2C_PSC_MIN) {
		printf("Error : I2C unsupported prescalar %d\n", psc);
		return;
	}

	if (speed == OMAP_I2C_HIGH_SPEED) {
		/* High speed */

		/* For first phase of HS mode */
		fsscll = fssclh = I2C_INTERNAL_SAMPLING_CLK /
			(2 * OMAP_I2C_FAST_MODE);

		fsscll -= I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM;
		fssclh -= I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			printf("Error : I2C initializing first phase clock\n");
			return;
		}

		/* For second phase of HS mode */
		hsscll = hssclh = I2C_INTERNAL_SAMPLING_CLK / (2 * speed);

		hsscll -= I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM;
		hssclh -= I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			printf("Error : I2C initializing second phase clock\n");
			return;
		}

		scll = (unsigned int)hsscll << 8 | (unsigned int)fsscll;
		sclh = (unsigned int)hssclh << 8 | (unsigned int)fssclh;

	} else {
		/* Standard and fast speed */
		fsscll = fssclh = I2C_INTERNAL_SAMPLING_CLK / (2 * speed);

		fsscll -= I2C_FASTSPEED_SCLL_TRIM;
		fssclh -= I2C_FASTSPEED_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			printf("Error : I2C initializing clock\n");
			return;
		}

		scll = (unsigned int)fsscll;
		sclh = (unsigned int)fssclh;
	}

	if (I2C_readw (I2C_CON) & I2C_CON_EN) {
		I2C_writew (0, I2C_CON);
		udelay (50000);
	}

	I2C_writew(psc, I2C_PSC);
	I2C_writew(scll, I2C_SCLL);
	I2C_writew(sclh,I2C_SCLH);

	printf("Psc %d scll %d sclh %d\n",psc,scll,sclh);
	/* own address */
	I2C_writew (slaveadd, I2C_OA);
	I2C_writew (I2C_CON_EN, I2C_CON);

	/* have to enable intrrupts or OMAP i2c module doesn't work */
	I2C_writew (I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | I2C_IE_ARDY_IE |
		I2C_IE_NACK_IE | I2C_IE_AL_IE, I2C_IE);
	udelay (1000);
	flush_fifo();
	I2C_writew (0xFFFF, I2C_STAT);
	I2C_writew (0, I2C_CNT);
}

static void flush_fifo(void) {
	Int16 stat;

	/* note: if you try and read data when its not there or ready
	 * you get a bus error
	 */
	while(1){
		stat = I2C_readw(I2C_STAT);
		if(stat == I2C_STAT_RRDY){
			I2C_readw(I2C_DATA);
			I2C_writew(I2C_STAT_RRDY,I2C_STAT);
			udelay(1000);
		}else
			break;
	}
}

int i2c_probe (int chip) {
	int res = 1; /* default = fail */

	if (chip == I2C_readw (I2C_OA)) {
		return res;
	}

	/* wait until bus not busy */
	wait_for_bb ();

	/* try to read one byte */
	I2C_writew (1, I2C_CNT);
	/* set slave address */
	I2C_writew (chip, I2C_SA);
	/* stop bit needed here */
	I2C_writew (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP, I2C_CON);
	/* enough delay for the NACK bit set */
	udelay (50000);

	if (!(I2C_readw (I2C_STAT) & I2C_STAT_NACK)) {
		res = 0;      /* success case */
		flush_fifo();
		I2C_writew(0xFFFF, I2C_STAT);
		printf("EEPROM detected %x\n",chip);
	} else {
		I2C_writew(0xFFFF, I2C_STAT);	 /* failue, clear sources*/
		I2C_writew (I2C_readw (I2C_CON) | I2C_CON_STP, I2C_CON); /* finish up xfer */
		udelay(20000);
		wait_for_bb ();
	}
	flush_fifo();
	I2C_writew (0, I2C_CNT); /* don't allow any more data in...we don't want it.*/
	I2C_writew(0xFFFF, I2C_STAT);
	return res;
}

static void wait_for_bb (void) {
	int timeout = 10;
	Int16 stat;

	I2C_writew(0xFFFF, I2C_STAT);	 /* clear current interruts...*/
	while ((stat = I2C_readw (I2C_STAT) & I2C_STAT_BB) && timeout--) {
		//I2C_writew (0xfff, I2C_STAT);
		udelay (50000);
	}

	if (timeout <= 0) {
		printf ("timed out in wait_for_bb: I2C_STAT=%x\n",
			I2C_readw (I2C_STAT));
	}
	I2C_writew(0xFFFF, I2C_STAT);	 /* clear delayed stuff*/
}

static Int16 wait_for_pin (void) {
	Int16 status;
	int timeout = 10;

	do {
		udelay (1000);
		status = I2C_readw (I2C_STAT);
       } while (  !(status &
                   (I2C_STAT_ROVR | I2C_STAT_XUDF | I2C_STAT_XRDY |
                    I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
                    I2C_STAT_AL)) && timeout--);

	if (timeout <= 0) {
		printf ("timed out in wait_for_pin: I2C_STAT=%x\n",
			I2C_readw (I2C_STAT));
			I2C_writew(0xFFFF, I2C_STAT);
	}
	return status;
}
Int32 wait_status_mask(Int32 mask)
{
	Int32 count = 1000,status =0;

	for(; count; count--){
		udelay(1000);
		status = I2C_readw(I2C_STAT);

		if(status & mask){
			return status;
		}
	}

	return status | 1<<31;
}


Int32 i2c_read_byte (Int8 devaddr, Uint16 regoffset, Uint8 alen, Uint8 * buffer, Uint8 len) {
	int i2c_error = 0;
	Int16 status,i;

	/* wait until bus not busy */
	wait_for_bb ();

	/* one byte only */
	I2C_writew (alen, I2C_CNT);
	/* set slave address */
	I2C_writew (devaddr, I2C_SA);
	I2C_writew((I2C_readw(I2C_BUF) | (I2C_TXFIFO_CLEAR |I2C_RXFIFO_CLEAR)), I2C_BUF);
	/* no stop bit needed here */
	I2C_writew (I2C_CON_EN | I2C_CON_MST | I2C_CON_TRX | I2C_CON_STT, I2C_CON);
	status = wait_status_mask(I2C_STAT_XRDY | I2C_STAT_NACK);

	for(i = 0; i < alen; i++){
		if (status & I2C_STAT_XRDY) {
			/* Important: have to use byte access */
			if (alen > 1)
				I2C_writeb ((regoffset >> (i?0:8) & 0xff), I2C_DATA);
			else
				I2C_writeb ((regoffset & 0xff), I2C_DATA);
			udelay (20000);
			status = wait_status_mask(I2C_STAT_XRDY);

			if (status & (1 << 31) ){
				I2C_writew(0, I2C_CON);
				printf("[Connection timeout]\n");
			}

			if ( status & I2C_STAT_NACK) {
				i2c_error = 1;
			}
		} else {
			i2c_error = 1;
		}
	}

	if (!i2c_error) {
		status = wait_status_mask(I2C_STAT_ARDY | I2C_STAT_NACK);

		if (status & (1 << 31) ){
			I2C_writew(0, I2C_CON);
			printf("[Connection timeout]\n");
		}
		

		/* set slave address */
		I2C_writew (devaddr, I2C_SA);
		/* read one byte from slave */
		I2C_writew ((len & 0xffff) , I2C_CNT);
		I2C_writew((I2C_readw(I2C_BUF) | (I2C_TXFIFO_CLEAR |I2C_RXFIFO_CLEAR)), I2C_BUF);
		/* need stop bit here */
		I2C_writew (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP,
				I2C_CON);

		status = wait_for_pin ();
		for(i=0;i<len&0xffff;i++) {

			Uint8 data;
			status = wait_status_mask(I2C_STAT_RRDY);

			if (status & (1 << 31) ){
				I2C_writew(0, I2C_CON);
				printf("[Connection timeout]\n");
			}

			if (status & I2C_STAT_RRDY) {
				data = I2C_readw (I2C_DATA);
				printf("%0x-",data);
				buffer[i] = data;
			} else {
				printf("read_error\n");
				i2c_error = 1;
			}

			udelay (20000);
		}
		printf("\n");

		if (!i2c_error) {
			I2C_writew (I2C_CON_EN, I2C_CON);
			while (I2C_readw (I2C_STAT)
			       || (I2C_readw (I2C_CON) & I2C_CON_MST)) {
				udelay (10000);
				I2C_writew (0xFFFF, I2C_STAT);
			}
		}
	}
	flush_fifo();
	I2C_writew (0xFFFF, I2C_STAT);
	I2C_writew (0, I2C_CNT);
	return i2c_error;
}

#if 0
static int i2c_write_byte (Int8 devaddr, Int8 regoffset, Int8 value) {
	int i2c_error = 0;
	Int16 status, stat;

	/* wait until bus not busy */
	wait_for_bb ();

	/* two bytes */
	I2C_writew (2, I2C_CNT);
	/* set slave address */
	I2C_writew (devaddr, I2C_SA);
	/* stop bit needed here */
	I2C_writew (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX |
		I2C_CON_STP, I2C_CON);

	/* wait until state change */
	status = wait_for_pin ();

	if (status & I2C_STAT_XRDY) {

		/* send out two bytes */
		I2C_writew ((value << 8) + regoffset, I2C_DATA);

		/* must have enough delay to allow BB bit to go low */
		udelay (50000);
		if (I2C_readw (I2C_STAT) & I2C_STAT_NACK) {
			i2c_error = 1;
		}
	} else {
		i2c_error = 1;
	}

	if (!i2c_error) {
		int eout = 200;

		I2C_writew (I2C_CON_EN, I2C_CON);
		while ((stat = I2C_readw (I2C_STAT)) || (I2C_readw (I2C_CON) & I2C_CON_MST)) {
			udelay (1000);
			/* have to read to clear intrrupt */
			I2C_writew (0xFFFF, I2C_STAT);
			if(--eout == 0) /* better leave with error than hang */
				break;
		}
	}
	flush_fifo();
	I2C_writew (0xFFFF, I2C_STAT);
	I2C_writew (0, I2C_CNT);
	return i2c_error;
}

int i2c_read (Int8 chip, Int addr, int alen, Int8 * buffer, int len) {
	int i;

	if (alen > 1) {
		printf ("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > 256) {
		printf ("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_read_byte (chip, addr + i, &buffer[i])) {
			printf ("I2C read: I/O error\n");
			i2c_init (OMAP_I2C_STANDARD, OMAP_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

int i2c_write (Int8 chip, Int addr, int alen, Int8 * buffer, int len) {
	int i;

	if (alen > 1) {
		printf ("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > 256) {
		printf ("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_write_byte (chip, addr + i, buffer[i])) {
			printf ("I2C read: I/O error\n");
			i2c_init (OMAP_I2C_STANDARD, OMAP_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

int i2c_set_bus_num(unsigned int bus) {
	if ((bus < 0) || (bus >= I2C_BUS_MAX)) {
		printf("Bad bus: %d\n", bus);
		return -1;
	}

#if I2C_BUS_MAX==3
	if (bus == 2)
		i2c_base = (struct i2c *)I2C_BASE3;
	else
#endif
	if (bus == 1)
		i2c_base = (struct i2c *)I2C_BASE2;
	else
		i2c_base = (struct i2c *)I2C_BASE1;

	current_bus = bus;

	if(!bus_initialized[current_bus])
		i2c_init(4000, i2c_init);

	return 0;
}
#endif
