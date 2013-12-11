/*-----------------------------------------------------------------------*/
/* Stellaris USB module                                                  */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/

/*
 * This file was modified from a sample available from the FatFs
 * web site. It was modified to work with the SitaraWare USB stack.
 */
#include "diskio.h"
#include "hw_types.h"
#include "mmcsd_proto.h"
#include "hs_mmcsdlib.h"
#include "ff.h"
#include "uartStdio.h"



typedef struct _fatDevice
{
    /* Pointer to underlying device/controller */
    void *dev;
    
    /* File system pointer */
    FATFS *fs;

	/* state */
	unsigned int initDone;

}fatDevice;


#define DRIVE_NUM_MMCSD     0
#define DRIVE_NUM_MAX      2


fatDevice fat_devices[DRIVE_NUM_MAX];


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS
disk_initialize(
    BYTE bValue)                /* Physical drive number (0) */
{
	unsigned int status;
   
    if (DRIVE_NUM_MAX <= bValue)
    {
        return STA_NODISK;
    }
    
    if ((DRIVE_NUM_MMCSD == bValue) && (fat_devices[bValue].initDone != 1))
    {
        mmcsdCardInfo *card = (mmcsdCardInfo *) fat_devices[bValue].dev;
        
        /* SD Card init */
        status = MMCSDCardInit(card->ctrl);

        if (status == 0)
        {
            UARTPuts("\r\nCard Init Failed \r\n", -1);
            
            return STA_NOINIT;
        }
        else
        {
#if DEBUG				
            if (card->cardType == MMCSD_CARD_SD)
            {
                UARTPuts("\r\nSD Card ", -1);
                UARTPuts("version : ",-1);
                UARTPutNum(card->sd_ver);
    
                if (card->highCap)
                {
                    UARTPuts(", High Capacity", -1);
                }
    
                if (card->tranSpeed == SD_TRANSPEED_50MBPS)
                {
                    UARTPuts(", High Speed", -1);
                }
            }
            else if (card->cardType == MMCSD_CARD_MMC)
            {
                UARTPuts("\r\nMMC Card ", -1);
            }
#endif            
            /* Set bus width */
            if (card->cardType == MMCSD_CARD_SD)
            {
                MMCSDBusWidthSet(card->ctrl);
            }
    
            /* Transfer speed */
            MMCSDTranSpeedSet(card->ctrl);
        }

		fat_devices[bValue].initDone = 1;
    }
        
    return 0;
}



/*-----------------------------------------------------------------------*/
/* Returns the current status of a drive                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE drv)                   /* Physical drive number (0) */
{
	return 0;
}



/*-----------------------------------------------------------------------*/
/* This function reads sector(s) from the disk drive                     */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE drv,               /* Physical drive number (0) */
    BYTE* buff,             /* Pointer to the data buffer to store read data */
    DWORD sector,           /* Physical drive nmuber (0) */
    BYTE count)             /* Sector count (1..255) */
{
	if (drv == DRIVE_NUM_MMCSD)
	{
		mmcsdCardInfo *card = fat_devices[drv].dev;

    	/* READ BLOCK */
		if (MMCSDReadCmdSend(card->ctrl, buff, sector, count) == 1)
		{
        	return RES_OK;
		}
    }

    return RES_ERROR;
}



/*-----------------------------------------------------------------------*/
/* This function writes sector(s) to the disk drive                     */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
    BYTE ucDrive,           /* Physical drive number (0) */
    const BYTE* buff,       /* Pointer to the data to be written */
    DWORD sector,           /* Start sector number (LBA) */
    BYTE count)             /* Sector count (1..255) */
{
	if (ucDrive == DRIVE_NUM_MMCSD)
	{
		mmcsdCardInfo *card = fat_devices[ucDrive].dev;
    	/* WRITE BLOCK */
	    if(MMCSDWriteCmdSend(card->ctrl,(BYTE*) buff, sector, count) == 1)
		{
        	return RES_OK;
		}
	}

    return RES_ERROR;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE drv,               /* Physical drive number (0) */
    BYTE ctrl,              /* Control code */
    void *buff)             /* Buffer to send/receive control data */
{
	return RES_OK;
}

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD get_fattime (void)
{
    return    ((2007UL-1980) << 25) // Year = 2007
            | (6UL << 21)           // Month = June
            | (5UL << 16)           // Day = 5
            | (11U << 11)           // Hour = 11
            | (38U << 5)            // Min = 38
            | (0U >> 1)             // Sec = 0
            ;
}
