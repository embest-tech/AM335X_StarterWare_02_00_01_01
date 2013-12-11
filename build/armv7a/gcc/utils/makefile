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
ROOT=../../../../

#
# Include makefile definitions.
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Source files for given target
#

SOURCE=${UTILITY_SRC}/ascii.c   \
       ${UTILITY_SRC}/cmdline.c   \
       ${UTILITY_SRC}/delay.c     \
       ${UTILITY_SRC}/perf.c     \
       ${UTILITY_SRC}/systick.c   \
       ${UTILITY_SRC}/misc.c \
       ${UTILITY_SRC}/uartStdio.c \
       ${UTILITY_SRC}/board.c \
       ${UTILITY_SRC}/consoleUtils.c

#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libutils.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${UTILITY_BIN}/Debug
	@cp Debug/libutils.a ${UTILITY_BIN}/Debug/libutils.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libutils.a *.o*
	@mv *.o* Release/
	@mkdir -p ${UTILITY_BIN}/Release
	@cp Release/libutils.a ${UTILITY_BIN}/Release/libutils.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${UTILITY_BIN}/Debug ${UTILITY_BIN}/Release

