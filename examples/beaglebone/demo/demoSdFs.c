/**
 * \file   hs_mmcsd_fs.c
 *
 * \brief  FS support for HS MMCSD Sample Application
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

#include "ff.h"
#include "cmdline.h"
#include "hs_mmcsd.h"
#include "consoleUtils.h"
#include "uartStdio.h"
#include "string.h"
#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "demoSdFs.h"
#include "mmcsd_proto.h"

/* Fat devices registered */
#ifndef fatDevice
typedef struct _fatDevice
{
    /* Pointer to underlying device/controller */
    void *dev;

    /* File system pointer */
    FATFS *fs;

    /* state */
    unsigned int initDone;

}fatDevice;
#endif
extern fatDevice fat_devices[2];

/*****************************************************************************
Defines the size of the buffers that hold the path, or temporary data from
the memory card.  There are two buffers allocated of this size.  The buffer
size must be large enough to hold the longest expected full path name,
including the file name, and a trailing null character.
******************************************************************************/
#define PATH_BUF_SIZE   512

/*****************************************************************************
Defines the size of the buffer that holds the command line.
******************************************************************************/
#define CMD_BUF_SIZE    512

/*****************************************************************************
Current FAT fs state.
******************************************************************************/
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment=SOC_CACHELINE_SIZE
static FATFS g_sFatFs;

#elif defined(__TMS470__)
#pragma DATA_ALIGN(g_sFatFs, SOC_CACHELINE_SIZE);
static FATFS g_sFatFs;

#elif defined(gcc)
static FATFS g_sFatFs  __attribute__ ((aligned (SOC_CACHELINE_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif

static DIR g_sDirObject;
static FILINFO g_sFileInfo;

#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment=SOC_CACHELINE_SIZE
static FIL g_sFileObject;

#elif defined(__TMS470__)
#pragma DATA_ALIGN(g_sFileObject, SOC_CACHELINE_SIZE);
static FIL g_sFileObject;

#elif defined(gcc)
static FIL g_sFileObject  __attribute__ ((aligned (SOC_CACHELINE_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif

volatile unsigned int sdCardAccessFlag = FALSE;
static volatile unsigned int sdCardState = FALSE;

/*****************************************************************************
A structure that holds a mapping between an FRESULT numerical code,
and a string representation.  FRESULT codes are returned from the FatFs
FAT file system driver.

******************************************************************************/
typedef struct
{
    FRESULT fresult;
    char *pcResultStr;
}
tFresultString;
/*****************************************************************************
A macro to make it easy to add result codes to the table.
******************************************************************************/
#define FRESULT_ENTRY(f)        { (f), (#f) }

/*****************************************************************************
A table that holds a mapping between the numerical FRESULT code and
it's name as a string.  This is used for looking up error codes for
printing to the console.
******************************************************************************/
tFresultString g_sFresultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_RW_ERROR),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_MKFS_ABORTED)
};

/*****************************************************************************
A macro that holds the number of result codes.
******************************************************************************/
#define NUM_FRESULT_CODES (sizeof(g_sFresultStrings) / sizeof(tFresultString))


/*****************************************************************************
This buffer holds the full path to the current working directory.  Initially
it is root ("/").
******************************************************************************/
static char g_cCwdBuf[PATH_BUF_SIZE] = "/";

/*****************************************************************************
A temporary data buffer used when manipulating file paths, or reading data
from the memory card.
******************************************************************************/
#ifdef __IAR_SYSTEMS_ICC__
#pragma data_alignment=SOC_CACHELINE_SIZE
static char g_cTmpBuf[PATH_BUF_SIZE];

#elif defined(__TMS470__)
#pragma DATA_ALIGN(g_cTmpBuf, SOC_CACHELINE_SIZE);
static char g_cTmpBuf[PATH_BUF_SIZE];

#elif defined(gcc)
static char g_cTmpBuf[PATH_BUF_SIZE] 
                      __attribute__ ((aligned (SOC_CACHELINE_SIZE)));

#else
#error "Unsupported Compiler. \r\n"

#endif

/*****************************************************************************
The buffer that holds the command line.
******************************************************************************/
static char g_cCmdBuf[CMD_BUF_SIZE];

volatile unsigned int g_sPState = 0;
volatile unsigned int g_sCState = 0;

/* SD Controller info structure */
extern mmcsdCtrlInfo  ctrlInfo;

extern unsigned int HSMMCSDCardPresent(mmcsdCtrlInfo *ctrl);

/*******************************************************************************
**
** This function reads a line of text from the console.
**
*******************************************************************************/
void
ReadLine(void)
{
    unsigned long ulIdx;
    unsigned char ucChar;

    /*
    ** Start reading at the beginning of the command buffer and print a prompt.
    */
    g_cCmdBuf[0] = '\0';
    ulIdx = 0;
    sdCardState = TRUE;

    /*
    ** Loop forever.  This loop will be explicitly broken out of when the line
    ** has been fully read.
    */
    while(TRUE == sdCardState)
    {
        /*
        ** Attempt to open the directory.
        */
        if((HSMMCSDCardPresent(&ctrlInfo)) == 1)
        {
            if(g_sCState == 0)
            {
                if(f_opendir(&g_sDirObject, g_cCwdBuf) != FR_OK)
                {
                    ConsoleUtilsPrintf("\nFailed to open directory.\n");
                    g_sCState = 0;
                    g_sPState = 0;
                    g_cCmdBuf[0] = '\0';
                    sdCardState = FALSE;
                    sdCardAccessFlag = FALSE;
                    return;
                }
                else
                {
                    g_sCState = 1;
                }
            }

            if (g_sCState != g_sPState)
            {
                if (g_sCState == 0)
                {
                    ConsoleUtilsPrintf("%s>", "UNKNOWN");
                    g_sPState = 0;

                    return;
                }
                else
                {
                    ConsoleUtilsPrintf("%s> %s", g_cCwdBuf, g_cCmdBuf);
                    g_sPState = 1;
                }
            }
        }
        else
        {
            g_sCState = 0;
            g_sPState = 0;
            g_cCmdBuf[0] = '\0';
            sdCardState = FALSE;
            sdCardAccessFlag = FALSE;
            return;
        }

        /*
        ** Loop while there are characters that have been received from the
        ** UART.
        */
        while(TRUE == UARTCharsAvail(SOC_UART_0_REGS))
        {
            /*
            ** Read the next character from the UART.
            */
            ucChar = UARTGetc();

            /*
            ** See if this character is a backspace and there is at least one
            ** character in the input line.
            */
            if((ucChar == '\b') && (ulIdx != 0))
            {
                /*
                ** Erase the last character from the input line.
                */
                ConsoleUtilsPrintf("\b \b");
                ulIdx--;
                g_cCmdBuf[ulIdx] = '\0';
            }

            /*
            ** See if this character is a newline.
            */
            else if((ucChar == '\r') || (ucChar == '\n'))
            {
                /*
                ** Return to the caller.
                */
                ConsoleUtilsPrintf("\n");
                return;
            }

            /*
            ** See if this character is an escape or Ctrl-U.
            */
            else if((ucChar == 0x1b) || (ucChar == 0x15))
            {
                sdCardState = FALSE;
                return;

            }

            /*
            ** See if this is a printable ASCII character.
            */
            else if((ucChar >= ' ') && (ucChar <= '~') &&
                    (ulIdx < (sizeof(g_cCmdBuf) - 1)))
            {
                /*
                ** Add this character to the input buffer.
                */
                g_cCmdBuf[ulIdx++] = ucChar;
                g_cCmdBuf[ulIdx] = '\0';
                ConsoleUtilsPrintf("%c", ucChar);
            }
        }
    }
}


/*****************************************************************************
This function returns a string representation of an error code that was
returned from a function call to FatFs.  It can be used for printing human
readable error messages.
*****************************************************************************/
const char *
StringFromFresult(FRESULT fresult)
{
    unsigned int uIdx;

    /* Enter a loop to search the error code table for a matching error code. */
    for(uIdx = 0; uIdx < NUM_FRESULT_CODES; uIdx++)
    {
        /*
        ** If a match is found, then return the string name of the error code.
        */
        if(g_sFresultStrings[uIdx].fresult == fresult)
        {
            return(g_sFresultStrings[uIdx].pcResultStr);
        }
    }

    /*
    ** At this point no matching code was found, so return a string indicating
    ** unknown error.
    */
    return("UNKNOWN ERROR CODE");
}

/*****************************************************************************
This function implements the "ls" command.  It opens the current directory
and enumerates through the contents, and prints a line for each item it
finds.  It shows details such as file attributes, time and date, and the
file size, along with the name.  It shows a summary of file sizes at the end
along with free space.
*****************************************************************************/
int
Cmd_ls(int argc, char *argv[])
{
    unsigned long ulTotalSize;
    unsigned long ulFileCount;
    unsigned long ulDirCount;
    FRESULT fresult;
    FATFS *pFatFs;

    /*
    ** Open the current directory for access.
    */
    fresult = f_opendir(&g_sDirObject, g_cCwdBuf);

    /*
    ** Check for error and return if there is a problem.
    */
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    ulTotalSize = 0;
    ulFileCount = 0;
    ulDirCount = 0;

    /*
    ** Enter loop to enumerate through all directory entries.
    */
    while(1)
    {
        /*
        ** Read an entry from the directory.
        */
        fresult = f_readdir(&g_sDirObject, &g_sFileInfo);

        /*
        ** Check for error and return if there is a problem.
        */
        if(fresult != FR_OK)
        {
            return(fresult);
        }

        /*
        ** If the file name is blank, then this is the end of the listing.
        */
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        /*
        ** If the attribute is directory, then increment the directory count.
        */
        if(g_sFileInfo.fattrib & AM_DIR)
        {
            ulDirCount++;
        }

        /*
        ** Otherwise, it is a file.  Increment the file count, and add in the
        ** file size to the total.
        */
        else
        {
            ulFileCount++;
            ulTotalSize += g_sFileInfo.fsize;
        }

        /*
        ** Print the entry information on a single line with formatting to show
        ** the attributes, date, time, size, and name.
        */
        ConsoleUtilsPrintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n",
                           (g_sFileInfo.fattrib & AM_DIR) ? 'D' : '-',
                           (g_sFileInfo.fattrib & AM_RDO) ? 'R' : '-',
                           (g_sFileInfo.fattrib & AM_HID) ? 'H' : '-',
                           (g_sFileInfo.fattrib & AM_SYS) ? 'S' : '-',
                           (g_sFileInfo.fattrib & AM_ARC) ? 'A' : '-',
                           (g_sFileInfo.fdate >> 9) + 1980,
                           (g_sFileInfo.fdate >> 5) & 15,
                            g_sFileInfo.fdate & 31,
                           (g_sFileInfo.ftime >> 11),
                           (g_sFileInfo.ftime >> 5) & 63,
                            g_sFileInfo.fsize,
                            g_sFileInfo.fname);
    }

    /*
    ** Print summary lines showing the file, dir, and size totals.
    */
    ConsoleUtilsPrintf("\n%4u File(s),%10u bytes total\n%4u Dir(s)",
                       ulFileCount, ulTotalSize, ulDirCount);

    /*
    ** Get the free space.
    */
    fresult = f_getfree("/", (DWORD *)&ulTotalSize, &pFatFs);

    /*
    ** Check for error and return if there is a problem.
    */
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    /*
    ** Display the amount of free space that was calculated.
    */
    ConsoleUtilsPrintf(", %10uK bytes free\n",
                       ulTotalSize * pFatFs->sects_clust / 2);

    /*
    ** Made it to here, return with no errors.
    */
    return(0);
}

/*****************************************************************************

This function implements the "cd" command.  It takes an argument that
specifies the directory to make the current working directory.  Path
separators must use a forward slash "/".  The argument to cd can be one of
the following:

 * root ("/")
 * a fully specified path ("/my/path/to/mydir")
 * a single directory name that is in the current directory ("mydir")
 * parent directory ("..")

It does not understand relative paths, so don't try something like this:
("../my/new/path")

Once the new directory is specified, it attempts to open the directory to
make sure it exists.  If the new path is opened successfully, then the
current working directory (cwd) is changed to the new path.

*****************************************************************************/
int
Cmd_cd(int argc, char *argv[])
{
    unsigned int uIdx;
    FRESULT fresult;

    /*
    ** Copy the current working path into a temporary buffer so it can be
    ** manipulated.
    */
    strcpy(g_cTmpBuf, g_cCwdBuf);

    /*
    ** If the first character is /, then this is a fully specified path, and it
    ** should just be used as-is.
    */
    if(argv[1][0] == '/')
    {
        /*
        ** Make sure the new path is not bigger than the cwd buffer.
        */
        if(strlen(argv[1]) + 1 > sizeof(g_cCwdBuf))
        {
            ConsoleUtilsPrintf("Resulting path name is too long\n");
            return(0);
        }

        /*
        ** If the new path name (in argv[1])  is not too long, then copy it
        ** into the temporary buffer so it can be checked.
        */
        else
        {
            strncpy(g_cTmpBuf, argv[1], sizeof(g_cTmpBuf));
        }
    }

    /*
    ** If the argument is .. then attempt to remove the lowest level on the
    ** CWD.
    */
    else if(!strcmp(argv[1], ".."))
    {
        /*
        ** Get the index to the last character in the current path.
        */
        uIdx = strlen(g_cTmpBuf) - 1;

        /*
        ** Back up from the end of the path name until a separator (/) is
        ** found, or until we bump up to the start of the path.
        */
        while((g_cTmpBuf[uIdx] != '/') && (uIdx > 1))
        {
            /*
            ** Back up one character.
            */
            uIdx--;
        }

        /*
        ** Now we are either at the lowest level separator in the current path,
        ** or at the beginning of the string (root).  So set the new end of
        ** string here, effectively removing that last part of the path.
        */
        g_cTmpBuf[uIdx] = 0;
    }

    /*
    ** Otherwise this is just a normal path name from the current directory,
    ** and it needs to be appended to the current path.
    */
    else
    {
        /*
        ** Test to make sure that when the new additional path is added on to
        ** the current path, there is room in the buffer for the full new path.
        ** It needs to include a new separator, and a trailing null character.
        */
        if(strlen(g_cTmpBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cCwdBuf))
        {
            ConsoleUtilsPrintf("Resulting path name is too long\n");
            return(0);
        }

        /*
        ** The new path is okay, so add the separator and then append the new
        ** directory to the path.
        */
        else
        {
            /*
            ** If not already at the root level, then append a /
            */
            if(strcmp(g_cTmpBuf, "/"))
            {
                strcat(g_cTmpBuf, "/");
            }

            /*
            ** Append the new directory to the path.
            */
            strcat(g_cTmpBuf, argv[1]);
        }
    }

    /*
    ** At this point, a candidate new directory path is in chTmpBuf.  Try to
    ** open it to make sure it is valid.
    */
    fresult = f_opendir(&g_sDirObject, g_cTmpBuf);

    /*
    ** If it can't be opened, then it is a bad path.  Inform user and return.
    */
    if(fresult != FR_OK)
    {
        ConsoleUtilsPrintf("cd: %s\n", g_cTmpBuf);
        return(fresult);
    }

    /*
    ** Otherwise, it is a valid new path, so copy it into the CWD.
    */
    else
    {
        strncpy(g_cCwdBuf, g_cTmpBuf, sizeof(g_cCwdBuf));
    }

    /*
    ** Return success.
    */
    return(0);
}

/*******************************************************************************
**
** This function implements the "pwd" command.  It simply prints the current
** working directory.
**
*******************************************************************************/
int
Cmd_pwd(int argc, char *argv[])
{
    /*
    ** Print the CWD to the console.
    */
    ConsoleUtilsPrintf("%s\n", g_cCwdBuf);

    return(0);
}

/*******************************************************************************
**
** This function implements the "exit" command.
**
*******************************************************************************/
int Cmd_exit(int argc, char *argv[])
{
    return 0;
}

/*******************************************************************************
**
** This function implements the "cat" command.  It reads the contents of a file
** and prints it to the console.  This should only be used on text files.  If
** it is used on a binary file, then a bunch of garbage is likely to printed on
** the console.
**
*******************************************************************************/
int
Cmd_cat(int argc, char *argv[])
{
    FRESULT fresult;
    unsigned short usBytesRead;

    /*
    ** First, check to make sure that the current path (CWD), plus the file
    ** name, plus a separator and trailing null, will all fit in the temporary
    ** buffer that will be used to hold the file name.  The file name must be
    ** fully specified, with path, to FatFs.
    */
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        ConsoleUtilsPrintf("Resulting path name is too long\n");
        return(0);
    }

    /*
    ** Copy the current path to the temporary buffer so it can be manipulated.
    */
    strcpy(g_cTmpBuf, g_cCwdBuf);

    /*
    ** If not already at the root level, then append a separator.
    */
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    /*
    ** Now finally, append the file name to result in a fully specified file.
    */
    strcat(g_cTmpBuf, argv[1]);

    /*
    ** Open the file for reading.
    */
    fresult = f_open(&g_sFileObject, g_cTmpBuf, FA_READ);

    /*
    ** If there was some problem opening the file, then return an error.
    */
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    /*
    ** Enter a loop to repeatedly read data from the file and display it, until
    ** the end of the file is reached.
    */
    do
    {
        /*
        ** Read a block of data from the file.  Read as much as can fit in the
        ** temporary buffer, including a space for the trailing null.
        */
        fresult = f_read(&g_sFileObject, g_cTmpBuf, sizeof(g_cTmpBuf) - 1,
                         &usBytesRead);

        /*
        ** If there was an error reading, then print a newline and return the
        ** error to the user.
        */
        if(fresult != FR_OK)
        {
            ConsoleUtilsPrintf("\n");
            return(fresult);
        }

        /*
        ** Null terminate the last block that was read to make it a null
        ** terminated string that can be used with printf.
        */
        g_cTmpBuf[usBytesRead] = 0;

        /*
        ** Print the last chunk of the file that was received.
        */
        ConsoleUtilsPrintf("%s", g_cTmpBuf);

        /*
        ** Continue reading until less than the full number of bytes are read.
        ** That means the end of the buffer was reached.
        */
    }
    while(usBytesRead == sizeof(g_cTmpBuf) - 1);

    /*
    ** Close the file.
    */
    fresult = f_close(&g_sFileObject);

    /*
    ** Return success.
    */
    return(0);
}

/*******************************************************************************
**
** This function implements the "help" command.  It prints a simple list of the
** available commands with a brief description.
**
*******************************************************************************/
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    /*
    ** Print some header text.
    */
    ConsoleUtilsPrintf("\nAvailable commands\n");
    ConsoleUtilsPrintf("------------------\n");

    /*
    ** Point at the beginning of the command table.
    */
    pEntry = &g_sCmdTable[0];

    /*
    ** Enter a loop to read each entry from the command table.  The end of the
    ** table has been reached when the command name is NULL.
    */
    while(pEntry->pcCmd)
    {
        /*
        ** Print the command name and the brief description.
        */
        ConsoleUtilsPrintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        /*
        ** Advance to the next entry in the table.
        */
        pEntry++;
    }

    /*
    ** Return success.
    */
    return(0);
}



void HSMMCSDFsMount(unsigned int driveNum, void *ptr)
{
    strcpy(g_cCwdBuf, "/");
    g_sPState = 0;
    g_sCState = 0;
    f_mount(driveNum, &g_sFatFs);
    fat_devices[driveNum].dev = ptr;
    fat_devices[driveNum].fs = &g_sFatFs;
    fat_devices[driveNum].initDone = 0;
}    
    
/*******************************************************************************
**
** This is the table that holds the command names, implementing functions, and
** brief description.
**
*******************************************************************************/
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   Cmd_help,      "  : Display list of commands" },
    { "h",      Cmd_help,   "     : alias for help" },
    { "?",      Cmd_help,   "     : alias for help" },
    { "ls",     Cmd_ls,      "    : Display list of files" },
    { "chdir",  Cmd_cd,         " : Change directory" },
    { "cd",     Cmd_cd,      "    : alias for chdir" },
    { "pwd",    Cmd_pwd,      "   : Show current working directory" },
    { "cat",    Cmd_cat,      "   : Show contents of a text file" },
    { "Ctrl+u", Cmd_exit,        ": Exit console" },
    { 0, 0, 0 }
};


void HSMMCSDFsProcessCmdLine(void)
{
    int iStatus;

    ConsoleUtilsPrintf("\n\r\n\rMMCSD command console entry!\n\r"
                       "Type 'help' for the list of commands supported \n\r"
                       "Press ESC key or input Crl+U to exit the console.\n\r");

    while(1)
    {
        //
        // Get a line of text from the user.
        //
        ReadLine();
        if(g_cCmdBuf[0] != '\0')
        {

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        iStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(iStatus == CMDLINE_BAD_CMD)
        {
            ConsoleUtilsPrintf("Bad command!\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(iStatus == CMDLINE_TOO_MANY_ARGS)
        {
            ConsoleUtilsPrintf("Too many arguments for command processor!\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(iStatus != 0)
        {
            ConsoleUtilsPrintf("Command returned error code %s\n",
                        StringFromFresult((FRESULT)iStatus));
        }

        g_cCmdBuf[0] = '\0';
        ConsoleUtilsPrintf("%s> %s", g_cCwdBuf, g_cCmdBuf);
       }

       if(FALSE == sdCardState)
       {
           ConsoleUtilsPrintf("\n\rMMCSD command console exit! \n\r");
           sdCardAccessFlag = FALSE;
           return;
       }
    }
}
