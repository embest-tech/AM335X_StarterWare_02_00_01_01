;*******************************************************************************
;*+--------------------------------------------------------------------------+**
;*|                            ****                                          |**
;*|                            ****                                          |**
;*|                            ******o***                                    |**
;*|                      ********_///_****                                   |**
;*|                      ***** /_//_/ ****                                   |**
;*|                       ** ** (__/ ****                                    |**
;*|                           *********                                      |**
;*|                            ****                                          |**
;*|                            ***                                           |**
;*|                                                                          |**
;*|         Copyright (c) 2009-2010 Texas Instruments Incorporated           |**
;*|                        ALL RIGHTS RESERVED                               |**
;*|                                                                          |**
;*| Permission is hereby granted to licensees of Texas Instruments           |**
;*| Incorporated (TI) products to use this computer program for the sole     |**
;*| purpose of implementing a licensee product based on TI products.         |**
;*| No other rights to reproduce, use, or disseminate this computer          |**
;*| program, whether in part or in whole, are granted.                       |**
;*|                                                                          |**
;*| TI makes no representation or warranties with respect to the             |**
;*| performance of this computer program, and specifically disclaims         |**
;*| any responsibility for any damages, special or consequential,            |**
;*| connected with the use of this program.                                  |**
;*|                                                                          |**
;*+--------------------------------------------------------------------------+**
;******************************************************************************/

;**
; \file super_startup.asm
;
; \brief Firmware startup for supervisor mode
;
; @author mansoor.ahamed@ti.com
;
;/



	.sect	".text"
	.state32
	.global	mystartup
	.global _c_int00

mystartup:
	ldr pc, RTS_JUMP ;Jump to rtslib _c_int00 but skip 4 instructions 
					 ;which puts cpu in user mode 
	 

RTS_JUMP		.long	(_c_int00 + 0x10)
	
	.end
