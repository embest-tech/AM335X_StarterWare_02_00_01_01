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
#******************************************************************************

#
# Locating the root directory
#
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am335x
EVM=beaglebone

#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${UTILITY_BLD}

#
# The application directory and name
#
APPDIR=bootloader
APPNAME=boot

#
# Where the bootloader will be loaded to by the ROM code
# starting 1Kbyte cannot be used due to hardware limitation
START_ADDR=0x402F0400


#
# Application Location
#
APP=${ROOT}bootloader/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/$(APPDIR)

CFLAGS+=-D $(BOOT) -D $(DEVICE)  -I$(APP)include -I$(APP)/include/${TARGET}/${DEVICE}

#
# Application source files
#
COMMON=$(APP)src/bl_main.c\
       $(APP)/src/bl_copy.c\
       $(APP)src/$(TARGET)/gcc/*.S\
       ${ROOT}/system_config/armv7a/cache.c\
       ${ROOT}/system_config/armv7a/mmu.c\
       ${ROOT}/system_config/armv7a/gcc/cp15.S\
       ${ROOT}/system_config/armv7a/$(DEVICE)/device.c\
       $(APP)src/$(TARGET)/$(DEVICE)/bl_platform.c

ifeq ($(BOOT), MMCSD)
SOURCE=$(APP)/src/bl_hsmmcsd.c \
       $(FATFS_SRC)/src/ff.c       \
       $(FATFS_SRC)/port/fat_mmcsd.c
DIRS+=${MMCSDLIB_BLD}
endif

ifeq ($(BOOT), UART)
SOURCE=$(APP)/src/bl_uart.c \
       ${ROOT}/third_party/xmodem/xmodem.c \
       ${ROOT}/third_party/xmodem/crc16.c
endif

APP_SRC=$(SOURCE) $(COMMON)

#
# Required library files
#
APP_LIB=-ldrivers  \
	-lutils    \
	-lplatform 
ifeq ($(BOOT), MMCSD)
APP_LIB+=-lmmcsd
endif

#
# Rules for building the application and library
#
all: debug release

debug:
	make TARGET_MODE=debug lib
	make TARGET_MODE=Debug bin

release:
	make TARGET_MODE=release lib
	make TARGET_MODE=Release bin

lib:
	@for i in ${DIRS};				\
	do						\
		if [ -f $${i}/makefile ] ;		    \
		then					  \
			make $(TARGET_MODE) -C $${i} || exit $$?; \
		fi;					   \
	done;


bin:
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)_$(BOOT)/
	@mv *.o* $(TARGET_MODE)_$(BOOT)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)_$(BOOT)/$(APPNAME).out -Map $(TARGET_MODE)_$(BOOT)/$(APPNAME).map \
	      $(TARGET_MODE)_$(BOOT)/*.o*  --defsym BOOT_START_ADDR=$(START_ADDR) -T $(APPNAME).lds $(APP_LIB) -lc -lgcc $(APP_LIB) -lc -lgcc
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)_$(BOOT)
	@cp $(TARGET_MODE)_$(BOOT)/$(APPNAME).out $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).out
	$(BIN) $(BINFLAGS) $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).out \
               $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).bin
	cd $(ROOT)/tools/ti_image/; gcc tiimage.c -o a.out; cd - 
	       $(ROOT)/tools/ti_image/a.out $(START_ADDR) $(BOOT) \
               $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).bin \
               $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME)_ti.bin; rm -rf $(ROOT)/tools/ti_image/a.out;


#
# Rules for cleaning
#
clean:
	@rm -rf Debug_MMCSD Debug_UART \
                Release_MMCSD Release_UART \
                $(APP_BIN)/Debug_MMCSD $(APP_BIN)/Debug_UART \
                $(APP_BIN)/Release_MMCSD $(APP_BIN)/Release_UART

clean+: clean
	@make TARGET_MODE=clean lib


