/*
 * TI Booting and Flashing Utilities
 *
 * Copyright 2007 by Spectrum Digital Incorporated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 *  Linker command file
 *
 */
/* -l rtsv5_A_le_eabi.lib */
/*-l rts32e.lib */

-stack          0x00004000      /* Stack Size */
-heap           0x00001000      /* Heap Size */

MEMORY
{
    L3OCM0:     o = 0x402F8000  l = 0x00017fff  /*  256 KBytes SRAM */
    L3OCM1:     o = 0x40400000  l = 0x0003FFFF  /*  256 KBytes SRAM */
}

SECTIONS
{
    .bss        >   L3OCM0
    .cinit      >   L3OCM0
    .cio        >   L3OCM0
    .const      >   L3OCM0
    .stack      >   L3OCM0
    .sysmem     >   L3OCM0
    .text       >   L3OCM0
    .DRAM       >   L3OCM0
	.far		>	L3OCM0
	.switch		>	L3OCM0
	.data		>   L3OCM0
}
