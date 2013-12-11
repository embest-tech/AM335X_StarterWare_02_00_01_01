/*
 * \file	mmcsd_proto.h
 *
 * \brief	MMC/SD definitions
 *
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

#ifndef __MMCSD_PROTO_H__
#define __MMCSD_PROTO_H__
#ifdef __cplusplus
extern "C" {
#endif

#define BIT(x) (1 << x)

/**
 * SD Card information structure
 */
#define MMCSD_CARD_SD		       (0u)
#define MMCSD_CARD_MMC		       (1u)

struct _mmcsdCtrlInfo;

/* Structure for SD Card information */
typedef struct _mmcsdCardInfo {
    struct _mmcsdCtrlInfo *ctrl;
	unsigned int cardType;
	unsigned int rca;
	unsigned int raw_scr[2];
	unsigned int raw_csd[4];
	unsigned int raw_cid[4];
	unsigned int ocr;
	unsigned char sd_ver;
	unsigned char busWidth;
	unsigned char tranSpeed;
	unsigned char highCap;
	unsigned int blkLen;
	unsigned int nBlks;
	unsigned int size;

}mmcsdCardInfo;

/* Structure for command */
typedef struct _mmcsdCmd {
	unsigned int idx;
	unsigned int flags;
	unsigned int arg;
	signed char *data;
	unsigned int nblks;
	unsigned int rsp[4];
}mmcsdCmd;

/* Structure for controller information */
typedef struct _mmcsdCtrlInfo {
	unsigned int memBase;
	unsigned int ipClk;
	unsigned int opClk;
	unsigned int (*ctrlInit) (struct _mmcsdCtrlInfo *ctrl);
	unsigned int (*cmdSend) (struct _mmcsdCtrlInfo *ctrl, mmcsdCmd *c);
    void (*busWidthConfig) (struct _mmcsdCtrlInfo *ctrl, unsigned int busWidth);
    int (*busFreqConfig) (struct _mmcsdCtrlInfo *ctrl, unsigned int busFreq);
	unsigned int (*cmdStatusGet) (struct _mmcsdCtrlInfo *ctrl);
	unsigned int (*xferStatusGet) (struct _mmcsdCtrlInfo *ctrl);
	void (*xferSetup) (struct _mmcsdCtrlInfo *ctrl, unsigned char rwFlag,
					       void *ptr, unsigned int blkSize, unsigned int nBlks);
    unsigned int (*cardPresent) (struct _mmcsdCtrlInfo *ctrl);
    void (*intrEnable) (struct _mmcsdCtrlInfo *ctrl);
    unsigned int intrMask;
	unsigned int dmaEnable;
	unsigned int busWidth;
	unsigned int highspeed;
	unsigned int ocr;
        unsigned int cdPinNum;
        unsigned int wpPinNum;
	mmcsdCardInfo *card;
}mmcsdCtrlInfo;

/* SD Commands enumeration */
#define SD_CMD(x)   (x)

/* Command/Response flags for notifying some information to controller */
#define SD_CMDRSP_NONE			BIT(0)
#define SD_CMDRSP_STOP			BIT(1)
#define SD_CMDRSP_FS			BIT(2)
#define SD_CMDRSP_ABORT			BIT(3)
#define SD_CMDRSP_BUSY			BIT(4)
#define SD_CMDRSP_136BITS		BIT(5)
#define SD_CMDRSP_DATA			BIT(6)
#define SD_CMDRSP_READ			BIT(7)
#define SD_CMDRSP_WRITE			BIT(8)



/* SD voltage enumeration as per VHS field of the interface command */
#define SD_VOLT_2P7_3P6                 (0x000100u)

/* SD OCR register definitions */
/* High capacity */
#define SD_OCR_HIGH_CAPACITY    	BIT(30)
/* Voltage */
#define SD_OCR_VDD_2P7_2P8		BIT(15)
#define SD_OCR_VDD_2P8_2P9		BIT(16)
#define SD_OCR_VDD_2P9_3P0		BIT(17)
#define SD_OCR_VDD_3P0_3P1		BIT(18)
#define SD_OCR_VDD_3P1_3P2		BIT(19)
#define SD_OCR_VDD_3P2_3P3		BIT(20)
#define SD_OCR_VDD_3P3_3P4		BIT(21)
#define SD_OCR_VDD_3P4_3P5		BIT(22)
#define SD_OCR_VDD_3P5_3P6		BIT(23)
/* This is for convenience only. Sets all the VDD fields */
#define SD_OCR_VDD_WILDCARD		(0x1FF << 15)

/* SD CSD register definitions */
#define SD_TRANSPEED_25MBPS		(0x32u)
#define SD_TRANSPEED_50MBPS		(0x5Au)

#define SD_CARD_CSD_VERSION(crd) (((crd)->raw_csd[3] & 0xC0000000) >> 30)

#define SD_CSD0_DEV_SIZE(csd3, csd2, csd1, csd0) (((csd2 & 0x000003FF) << 2) | ((csd1 & 0xC0000000) >> 30))
#define SD_CSD0_MULT(csd3, csd2, csd1, csd0) ((csd1 & 0x00038000) >> 15)
#define SD_CSD0_RDBLKLEN(csd3, csd2, csd1, csd0) ((csd2 & 0x000F0000) >> 16)
#define SD_CSD0_TRANSPEED(csd3, csd2, csd1, csd0) ((csd3 & 0x000000FF) >> 0)

#define SD_CARD0_DEV_SIZE(crd) SD_CSD0_DEV_SIZE((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD0_MULT(crd) SD_CSD0_MULT((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD0_RDBLKLEN(crd) SD_CSD0_RDBLKLEN((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD0_TRANSPEED(crd) SD_CSD0_TRANSPEED((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD0_NUMBLK(crd) ((SD_CARD0_DEV_SIZE((crd)) + 1) * (1 << (SD_CARD0_MULT((crd)) + 2)))
#define SD_CARD0_SIZE(crd) ((SD_CARD0_NUMBLK((crd))) * (1 << (SD_CARD0_RDBLKLEN(crd))))

#define SD_CSD1_DEV_SIZE(csd3, csd2, csd1, csd0) (((csd2 & 0x0000003F) << 16) | ((csd1 & 0xFFFF0000) >> 16))
#define SD_CSD1_RDBLKLEN(csd3, csd2, csd1, csd0) ((csd2 & 0x000F0000) >> 16)
#define SD_CSD1_TRANSPEED(csd3, csd2, csd1, csd0) ((csd3 & 0x000000FF) >> 0)

#define SD_CARD1_DEV_SIZE(crd) SD_CSD1_DEV_SIZE((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD1_RDBLKLEN(crd) SD_CSD1_RDBLKLEN((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD1_TRANSPEED(crd) SD_CSD1_TRANSPEED((crd)->raw_csd[3], (crd)->raw_csd[2], (crd)->raw_csd[1], (crd)->raw_csd[0])
#define SD_CARD1_SIZE(crd) ((SD_CARD1_DEV_SIZE((crd)) + 1) * (512 * 1024))


/* Check RCA/status */
#define SD_RCA_ADDR(rca)             ((rca & 0xFFFF0000) >> 16)
#define SD_RCA_STAT(rca)             (rca & 0x0xFFFF)

/* Check pattern that can be used for card response validation */
#define SD_CHECK_PATTERN   0xAA

/* SD SCR related macros */
#define SD_VERSION_1P0		0
#define SD_VERSION_1P1		1
#define SD_VERSION_2P0		2
#define SD_BUS_WIDTH_1BIT	1
#define SD_BUS_WIDTH_4BIT	4

/* Helper macros */
/* Note card registers are big endian */
#define SD_CARD_VERSION(sdcard)		((sdcard)->raw_scr[0] & 0xF)
#define SD_CARD_BUSWIDTH(sdcard)	(((sdcard)->raw_scr[0] & 0xF00) >> 8)
#define GET_SD_CARD_BUSWIDTH(sdcard)  ((((sdcard.busWidth) & 0x0F) == 0x01) ? \
                                      0x1 : ((((sdcard).busWidth & 0x04) == \
                                      0x04) ? 0x04 : 0xFF))
#define GET_SD_CARD_FRE(sdcard)	      (((sdcard.tranSpeed) == 0x5A) ? 50 : \
                                      (((sdcard.tranSpeed) == 0x32) ? 25 : 0))

/* Cacheline size */
#ifndef SOC_CACHELINE_SIZE
#define SOC_CACHELINE_SIZE         128
#endif

/* CM6 Swith mode arguments for High Speed */
#define SD_SWITCH_MODE        0x80FFFFFF
#define SD_CMD6_GRP1_SEL      0xFFFFFFF0
#define SD_CMD6_GRP1_HS       0x1

/*
 * Function prototypes
 */

extern unsigned int MMCSDReadCmdSend(mmcsdCtrlInfo *ctrl, void *ptr, unsigned int block,
				                     unsigned int blks);
extern unsigned int MMCSDWriteCmdSend(mmcsdCtrlInfo *ctrl, void *ptr, unsigned int block,
				                       unsigned int blks);
extern unsigned int MMCSDAppCmdSend(mmcsdCtrlInfo *ctrl, mmcsdCmd *c);
extern unsigned int MMCSDCmdSend(mmcsdCtrlInfo *ctrl, mmcsdCmd *c);
extern unsigned int MMCSDTranSpeedSet(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDBusWidthSet(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDStopCmdSend(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDCardPresent(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDCardReset(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDCardInit(mmcsdCtrlInfo *ctrl);
extern unsigned int MMCSDCtrlInit(mmcsdCtrlInfo *ctrl);
extern void MMCSDIntEnable(mmcsdCtrlInfo *ctrl);
#ifdef __cplusplus
}
#endif

#endif
