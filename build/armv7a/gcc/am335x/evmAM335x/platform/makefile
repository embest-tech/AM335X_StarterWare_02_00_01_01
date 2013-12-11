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
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am335x
EVM=evmAM335x

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Source files
#
SOURCE=${PLATFORM_SRC}/uart.c	\
       ${PLATFORM_SRC}/hsi2c.c       \
       ${PLATFORM_SRC}/gpio.c	\
       ${PLATFORM_SRC}/mcasp.c       \
       ${PLATFORM_SRC}/rtc.c	 \
       ${PLATFORM_SRC}/board.c   \
       ${PLATFORM_SRC}/lcd.c	 \
       ${PLATFORM_SRC}/uartConsole.c \
       ${PLATFORM_SRC}/edma.c	\
       ${PLATFORM_SRC}/sysdelay.c    \
       ${PLATFORM_SRC}/sysperf.c    \
       ${PLATFORM_SRC}/timertick.c   \
       ${PLATFORM_SRC}/usb.c	 \
       ${PLATFORM_SRC}/cpld.c	\
       ${PLATFORM_SRC}/hs_mmcsd.c    \
       ${PLATFORM_SRC}/pwmss.c       \
       ${PLATFORM_SRC}/eeprom.c      \
       ${PLATFORM_SRC}/dcan.c	\
       ${PLATFORM_SRC}/tscAdc.c      \
       ${PLATFORM_SRC}/mcspi.c       \
       ${PLATFORM_SRC}/dmtimer.c     \
       ${PLATFORM_SRC}/nand.c	\
       ${PLATFORM_SRC}/cpsw.c	\
       ${PLATFORM_SRC}/ctlregcontext.c	\
       ${PLATFORM_SRC}/watchdog.c

#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libplatform.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${PLATFORM_BIN}/Debug
	@cp Debug/libplatform.a ${PLATFORM_BIN}/Debug/libplatform.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libplatform.a *.o*
	@mv *.o* Release/
	@mkdir -p ${PLATFORM_BIN}/Release
	@cp Release/libplatform.a ${PLATFORM_BIN}/Release/libplatform.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release

clean+:
	@rm -rf Debug Release ${PLATFORM_BIN}/Debug ${PLATFORM_BIN}/Release


