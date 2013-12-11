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
EVM=evmAM335x

#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ${ROOT}/build/armv7a/gcc/makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD}

#
# The application directory and name
#
APPDIR=cache_mmu
APPNAME=uartEdma_Cache

#
# Where the application will be loaded to. This is required to generate
# image with Header (Load Address and Size)
#
IMG_LOAD_ADDR = 0x80000000

#
# Application Location
#
APP=${ROOT}examples/${EVM}/$(APPDIR)/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/$(APPDIR)

#
# Application source files
#
APP_SRC=$(APP)/*.c

#
# Required library files
#
APP_LIB=-ldrivers  \
	-lutils    \
	-lplatform \
	-lsystem_config

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
# Below commands generate .out and .bin without cache cleaning 
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/$(APPNAME)_Cache.out -Map $(TARGET_MODE)/$(APPNAME)_Cache.map \
	      $(TARGET_MODE)/*.o* -T $(APPNAME).lds $(APP_LIB) -lc -lgcc $(APP_LIB) -lc -lgcc
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)_Cache
	@cp $(TARGET_MODE)/$(APPNAME)_Cache.out $(APP_BIN)/$(TARGET_MODE)_Cache/$(APPNAME)_Cache.out
	$(BIN) $(BINFLAGS) $(APP_BIN)/$(TARGET_MODE)_Cache/$(APPNAME)_Cache.out \
	       $(APP_BIN)/$(TARGET_MODE)_Cache/$(APPNAME)_Cache.bin
	cd $(ROOT)/tools/ti_image/; gcc tiimage.c -o a.out; cd -
	       $(ROOT)/tools/ti_image/a.out $(IMG_LOAD_ADDR) NONE \
	       $(APP_BIN)/$(TARGET_MODE)_Cache/$(APPNAME)_Cache.bin \
	       $(APP_BIN)/$(TARGET_MODE)_Cache/$(APPNAME)_Cache_ti.bin; rm -rf $(ROOT)/tools/ti_image/a.out;

# Below commands generate .out and .bin with cache cleaning
	rm -rf *.o Debug Release
	$(CC)  $(CFLAGS) -D CACHE_FLUSH $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/$(APPNAME)_CacheFlush.out -Map $(TARGET_MODE)/$(APPNAME)_CacheFlush.map \
	      $(TARGET_MODE)/*.o* -T $(APPNAME).lds $(APP_LIB) -lc -lgcc $(APP_LIB) -lc -lgcc
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)_CacheFlush
	@cp $(TARGET_MODE)/$(APPNAME)_CacheFlush.out $(APP_BIN)/$(TARGET_MODE)_CacheFlush/$(APPNAME)_CacheFlush.out
	$(BIN) $(BINFLAGS) $(APP_BIN)/$(TARGET_MODE)_CacheFlush/$(APPNAME)_CacheFlush.out \
	       $(APP_BIN)/$(TARGET_MODE)_CacheFlush/$(APPNAME)_CacheFlush.bin
	cd $(ROOT)/tools/ti_image/; gcc tiimage.c -o a.out; cd -
	       $(ROOT)/tools/ti_image/a.out $(IMG_LOAD_ADDR) NONE \
	       $(APP_BIN)/$(TARGET_MODE)_CacheFlush/$(APPNAME)_CacheFlush.bin \
	       $(APP_BIN)/$(TARGET_MODE)_CacheFlush/$(APPNAME)_CacheFlush_ti.bin; rm -rf $(ROOT)/tools/ti_image/a.out;


#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib


