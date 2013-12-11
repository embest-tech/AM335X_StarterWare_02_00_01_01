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

#
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
SOURCE= ${SYSCONFIG_SRC}/mmu.c                 \
	${SYSCONFIG_SRC}/cache.c               \
	${SYSCONFIG_SRC}/${COMPILER}/cpu.c     \
	${SYSCONFIG_SRC}/${COMPILER}/cp15.S    \
	${SYSCONFIG_SRC}/${COMPILER}/init.S    \
	${SYSCONFIG_SRC}/${DEVICE}/interrupt.c \
	${SYSCONFIG_SRC}/${DEVICE}/startup.c   \
	${SYSCONFIG_SRC}/${DEVICE}/clock.c     \
    ${SYSCONFIG_SRC}/${DEVICE}/device.c    \
	${SYSCONFIG_SRC}/${DEVICE}/${COMPILER}/exceptionhandler.S

#
# Add compiler options to compile NEON SIMD instructions
#
CFLAGS+= -mfpu=neon -mfloat-abi=softfp

#
# Rules for building the source files
#

all: debug release

debug:
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libsystem_config.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${SYSCONFIG_BIN}/Debug
	@cp Debug/libsystem_config.a ${SYSCONFIG_BIN}/Debug/libsystem_config.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libsystem_config.a *.o*
	@mv *.o* Release/
	@mkdir -p ${SYSCONFIG_BIN}/Release
	@cp Release/libsystem_config.a ${SYSCONFIG_BIN}/Release/libsystem_config.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${SYSCONFIG_BIN}/Debug ${SYSCONFIG_BIN}/Release


