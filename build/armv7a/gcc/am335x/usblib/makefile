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
# Device definition (required for interrupt API prototypes)
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
SOURCE= ${ROOT}usblib/usbbuffer.c                 \
        ${ROOT}usblib/usbdesc.c                   \
        ${ROOT}usblib/usbringbuf.c                \
        ${ROOT}usblib/usbtick.c                   \
        ${ROOT}usblib/usbdata.c                   \
        ${ROOT}usblib/device/usbdcdc.c            \
        ${ROOT}usblib/device/usbdhid.c            \
        ${ROOT}usblib/device/usbdhidmouse.c       \
        ${ROOT}usblib/device/usbdmsc.c            \
        ${ROOT}usblib/device/usbdbulk.c           \
        ${ROOT}usblib/device/usbdcdesc.c          \
        ${ROOT}usblib/device/usbdcomp.c           \
        ${ROOT}usblib/device/usbdconfig.c         \
        ${ROOT}usblib/device/usbdenum.c           \
        ${ROOT}usblib/device/usbdhandler.c        \
        ${ROOT}usblib/host/usbhostenum.c          \
        ${ROOT}usblib/host/usbhhid.c              \
        ${ROOT}usblib/host/usbhmsc.c              \
        ${ROOT}usblib/host/usbhscsi.c             \
        ${ROOT}usblib/host/usbhhidmouse.c

#
# Rules for building the source files
#
all: debug release

debug:
	mkdir -p Debug 
	$(CC) $(CFLAGS) -g -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libusblib.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${USBLIB_BIN}/Debug
	@cp Debug/libusblib.a ${USBLIB_BIN}/Debug/libusblib.a

release:
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Release/libusblib.a *.o*
	@mv *.o* Release/
	@mkdir -p ${USBLIB_BIN}/Release
	@cp Release/libusblib.a ${USBLIB_BIN}/Release/libusblib.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${USBLIB_BIN}/Debug ${USBLIB_BIN}/Release



