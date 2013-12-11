#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
#
# 
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#
#    Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the   
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
#
# Locating the root directory
#
ROOT=../../../../../

# Device definition (EVM not necessary)
#
DEVICE=am335x

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Source files for given target
#
SOURCE=${DRIVERS_SRC}/dmtimer.c	 \
       ${DRIVERS_SRC}/uart_irda_cir.c   \
       ${DRIVERS_SRC}/ecap.c	    \
       ${DRIVERS_SRC}/ehrpwm.c	  \
       ${DRIVERS_SRC}/mcspi.c	   \
       ${DRIVERS_SRC}/mcasp.c	   \
       ${DRIVERS_SRC}/mdio.c	    \
       ${DRIVERS_SRC}/phy.c	     \
       ${DRIVERS_SRC}/cpsw.c	    \
       ${DRIVERS_SRC}/hsi2c.c	   \
       ${DRIVERS_SRC}/gpio_v2.c	 \
       ${DRIVERS_SRC}/watchdog.c	\
       ${DRIVERS_SRC}/edma.c	    \
       ${DRIVERS_SRC}/rtc.c	     \
       ${DRIVERS_SRC}/raster.c	  \
       ${DRIVERS_SRC}/gpmc.c	    \
       ${DRIVERS_SRC}/elm.c	     \
       ${DRIVERS_SRC}/tsc_adc.c	 \
       ${DRIVERS_SRC}/hs_mmcsd.c	\
       ${DRIVERS_SRC}/mailbox.c	 \
       ${DRIVERS_SRC}/usb.c	     \
       ${DRIVERS_SRC}/cppi41dma.c       \
       ${DRIVERS_SRC}/dcan.c	    \
       ${DRIVERS_SRC}/usbphyGS70.c	


#
# Rules for  building the source files
#
all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libdrivers.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${DRIVERS_BIN}/Debug
	@cp Debug/libdrivers.a ${DRIVERS_BIN}/Debug/libdrivers.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libdrivers.a *.o*
	@mv *.o* Release/
	@mkdir -p ${DRIVERS_BIN}/Release
	@cp Release/libdrivers.a ${DRIVERS_BIN}/Release/libdrivers.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${DRIVERS_BIN}/Debug ${DRIVERS_BIN}/Release


