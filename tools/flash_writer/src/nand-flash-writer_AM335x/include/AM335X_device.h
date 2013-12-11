/*
 * device.h
 */

/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 */
/* 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* --------------------------------------------------------------------------
 * FILE        : device.h
 * PROJECT     : AM335X NAND Flash Writer.
 * AUTHOR      : Mansoor Ahamed (mansoor.ahamed@ti.com)
 * 		 Vinay Agrawal (vinayagw@ti.com)
 * DESC        : Provides device differentiation for the project files. This
 * 			file MUST be modified to match the device specifics.
 * ---------------------------------------------------------------------------
 */

#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "tistdtypes.h"
#include "AM335X_nand.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/
#define AM335X_DEVICE_ASYNC_MEM_INTERFACE_CNT  (1)
#define AM335X_DEVICE_ASYNC_MEM0_REGION_CNT    (4)
#define AM335X_DEVICE_ASYNC_MEM_NANDBOOT_INTERFACE   (0)
#define AM335X_DEVICE_ASYNC_MEM_NANDBOOT_REGION      (1)

#define GPMC_MAX_CS                           (7)
#define CONTROL_MODULE                        (0x44e10000)
#define GPMC_AD0_OFF                          (0x800)
#define GPMC_AD1_OFF                          (0x804)
#define GPMC_AD2_OFF                          (0x808)
#define GPMC_AD3_OFF                          (0x80c)
#define GPMC_AD4_OFF                          (0x810)
#define GPMC_AD5_OFF                          (0x814)
#define GPMC_AD6_OFF                          (0x818)
#define GPMC_AD7_OFF                          (0x81C)
#define GPMC_WAIT0_OFF                        (0x870)
#define GPMC_WPN_OFF                          (0x874)
#define GPMC_CSN0_OFF                         (0x87c)
#define GPMC_ADVN_ALE_OFF                     (0x890)
#define GPMC_OEN_REN_OFF                      (0x894)
#define GPMC_WEN_OFF                          (0x898)
#define GPMC_BE0N_CLE_OFF                     (0x89c)

#define I2C0_SD0_OFF                          (0x988)
#define I2C0_SCL_OFF                          (0x98C)

#define MODE(val)                             (val << 0)
#define SLEW_CONTROL_SLOW                     (1 << 6)
#define RXACTIVE                              (1 << 5)
#define PULLUP_EN                             (1 << 4)
#define PULLUPDOWN_DISABLE                    (1 << 3)

// GPMC register
#define GPMC_REVISION                         (0x50000000u)
#define GPMC_SYSCONFIG                       (0x50000010u)
#define GPMC_SYSSTATUS                       (0x50000014u)
#define GPMC_IRQSTATUS                       (0x50000018u)
#define GPMC_IRQENABLE                       (0x5000001Cu)
#define GPMC_TIMEOUT_CONTRO                  (0x50000040u)
#define GPMC_ERR_ADDRESS                     (0x50000044u)
#define GPMC_ERR_TYPE                        (0x50000048u)
#define GPMC_CONFIG                          (0x50000050u)
#define GPMC_STATUS                          (0x50000054u)

//GPMC CS base address
#define GPMC_base_CS_0                        (0x50000000u)
#define GPMC_base_CS_1                        (0x50000030u)
#define GPMC_base_CS_2                        (0x50000060u)
#define GPMC_base_CS_3                        (0x50000090u)
#define GPMC_base_CS_4                        (0x500000c0u)
#define GPMC_base_CS_5                        (0x500000e0u)
#define GPMC_base_CS_6                        (0x50000120u)
#define GPMC_base_CS_7                        (0x50000150u)


#define	GPMC_CONFIG1                          (0x00000060u)
#define	GPMC_CONFIG2                          (0x00000064u)
#define	GPMC_CONFIG3                          (0x00000068u)
#define	GPMC_CONFIG4                          (0x0000006Cu)
#define	GPMC_CONFIG5                          (0x00000070u)
#define	GPMC_CONFIG6                          (0x00000074u)
#define	GPMC_CONFIG7                          (0x00000078u)
#define	GPMC_NAND_COMMAND                     (0x0000007Cu)
#define	GPMC_NAND_ADDRESS                     (0x00000080u)
#define	GPMC_NAND_DATA                        (0x00000084u)

#define	GPMC_ECC_CONFIG                       (0x500001F4u)
#define	GPMC_ECC_CONTROL                      (0x500001F8u)
#define	GPMC_ECC_SIZE_CONFIG                  (0x500001FCu)
#define GPMC_ECC1_RESULT                      (0x50000200u)
#define	GPMC_TESTMODE_CTRL                    (0x50000230u)

#define	GPMC_BCH_RESULT_0                     (0x50000240u)
#define	GPMC_BCH_RESULT_1                     (0x50000244u)
#define	GPMC_BCH_RESULT_2                     (0x50000248u)
#define	GPMC_BCH_RESULT_3                     (0x5000024Cu)
#define	GPMC_BCH_SWDATA                       (0x500002D0u)
#define	GPMC_BCH_RESULT_4                     (0x50000300u)
#define	GPMC_BCH_RESULT_5                     (0x50000304u)
#define	GPMC_BCH_RESULT_6                     (0x50000308u)

#define AM335X_DEVICE_NAND_DATA_OFFSET        GPMC_NAND_DATA
#define AM335X_DEVICE_NAND_ALE_OFFSET         GPMC_NAND_ADDRESS
#define AM335X_DEVICE_NAND_CLE_OFFSET         GPMC_NAND_COMMAND

#define AM335X_DEVICE_NAND_TIMEOUT            (10240)

#define AM335X_DEVICE_NAND_MAX_BYTES_PER_OP       (512)  
#define AM335X_DEVICE_NAND_MAX_SPAREBYTES_PER_OP  (16)   
#define AM335X_DEVICE_NAND_MIN_SPAREBYTES_PER_OP  (10)

// Defines which NAND blocks the RBL will search in for a U-BOOT image
#define AM335X_DEVICE_NAND_RBL_SEARCH_START_BLOCK     (0)
#define AM335X_DEVICE_NAND_RBL_SEARCH_END_BLOCK       (3)

// Defines which NAND blocks are valid for writing the APP data
#define AM335X_DEVICE_NAND_UBOOT_SEARCH_END_BLOCK       (15)

#define AM335X_DEVICE_DDR2_START_ADDR    (0x80000000u)
#define AM335X_DEVICE_DDR2_END_ADDR      ((AM335X_DEVICE_DDR2_START_ADDR + AM335X_DEVICE_DDR2_RAM_SIZE))
#define AM335X_DEVICE_DDR2_RAM_SIZE      (0x10000000u)

#define PRCM_BASE             (0x44e00000) 
#define CM_PER_GPMC_CLKCTRL   (PRCM_BASE + 0x030)
#define CM_PER_ELM_CLKCTRL   (PRCM_BASE + 0x040)
#define CM_PER_I2C0_CLKCTRL   (PRCM_BASE + 0x4B8)
#define PRCM_MOD_EN           (0x2)

/******************************************************
* Global Typedef declarations                         *
******************************************************/

typedef struct GPMC_CONFIG_STRUCT {
	Uint32 SysConfig;
	Uint32 IrqStatus;
	Uint32 IrqEnable;
	Uint32 TimeOutControl;
	Uint32 Config;
	Uint32 ChipSelectConfig[GPMC_MAX_CS];
} GPMC_Config_t, *GPMC_Config_handle;

typedef struct pin_muxing {
	int offset;
	int val;
}pin_muxing_t, *pin_muxing_handle;


/***********************************************************
* Global Variable Declarations                             *
***********************************************************/
extern __FAR__ AM335X_ASYNC_MEM_InfoHandle AM335X_ASYNC_MEM_Open(AM335X_ASYNC_MEM_Type memType,
		Uint32 baseAddress, Uint8 busWidth);
/***********************************************************
* Global Function Declarations                             *
***********************************************************/
// System Initialization prototypes
extern __FAR__ Uint32 AM335X_DEVICE_init(void);
//extern Uint32 AM335X_DEVICE_setGPMC(GPMC_Config_t *cfg, Uint32 base_CS);
extern AM335X_NAND_InfoHandle AM335X_NAND_open(Uint32 baseCSAddr, Uint8 busWidth);
extern Uint32 AM335X_Start(void); // AM335X_Start
extern void GPMC_Write(Uint32 base, Uint32 offset, Uint32 val);
extern void PAD_ConfigMux (pin_muxing_t * pin_mux);
/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_H_
