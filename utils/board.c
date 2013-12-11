/**
 * \file   board.c
 *
 * \brief  This file contains functions which is used to determine the version
 *         and boardId information.
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include"board.h"

/*****************************************************************************
**                   GLOBAL VARIABLE DEFINITIONS
*****************************************************************************/

unsigned char boardVersion[EEPROM_SIZE_VERSION];
unsigned char boardName[EEPROM_SIZE_BOARD_NAME];
BOARDINFO *boardData;
unsigned int boardId = BOARD_UNKNOWN;

/*****************************************************************************
**                    FUNCTION DEFINITIONS
*****************************************************************************/

/* returns version of the board */
unsigned char *BoardVersionGet(void)
{
    return boardVersion;
}

/* returns Name of the board */
unsigned char *BoardNameGet(void)
{
    return boardName;
}

/* returns Name of the board */
unsigned int BoardIdGet(void)
{
    return boardId;
}

/* Reads EEPROM and validates the board information */
unsigned int BoardInfoInit(void)
{
    unsigned int index;
    unsigned char boardInfo[MAX_DATA];

    BoardInfoRead(boardInfo);

    boardData = (BOARDINFO *)boardInfo;

    for(index = 0; index < (EEPROM_SIZE_VERSION - 1); index++)
    {
         boardVersion[index] = boardData->version[index];
    }

    boardVersion[index] = '\0';

    for(index = 0; index < (EEPROM_SIZE_BOARD_NAME - 1); index++)
    {
         boardName[index] = boardData->boardName[index];
    }

    boardName[index] = '\0';

    boardId = BoardInfoCheck(boardName, boardVersion);

    return boardId;
}
