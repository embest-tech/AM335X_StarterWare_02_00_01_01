/**
 * \file   board.c
 *
 * \brief  This file contains functions which are used to determine the version
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

#include "board.h"
#include "evmAM335x.h"
#include <string.h>

/*****************************************************************************
**                   GLOBAL VARIABLE DEFINITIONS
*****************************************************************************/

/*****************************************************************************
**                    FUNCTION DEFINITIONS
*****************************************************************************/

unsigned int BoardInfoCheck(unsigned char *boardId, unsigned char *boardVer)
{
    if(!(strcmp(EVM_BOARD_NAME, (char *)boardId)))
    {
        if(!(strncmp(EVM_1_5X_VERSION, (char *)boardVer, BOARD_ID_CMP_SIZE)))
        {
            return BOARD_ID_EVM_DDR3;
        }
        else if(!(strncmp(EVM_1_2X_VERSION, (char *)boardVer,
                          BOARD_ID_CMP_SIZE)))
        {
            return BOARD_ID_EVM_DDR2;
        }
        else if(!(strncmp(EVM_1_1X_VERSION, (char *)boardVer,
                          BOARD_ID_CMP_SIZE)))
        {
            return BOARD_ID_EVM_DDR2;
        }
        else
        {
            return BOARD_VER_UNKNOWN;
        }
    }
    else
    {
        return BOARD_UNKNOWN;
    }
}
