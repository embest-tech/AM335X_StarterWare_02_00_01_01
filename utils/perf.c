/**
 *  \file   perf.c
 *
 *  \brief  This file contains APIs for measuring performance using
 *          DMTimer.
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

#include "perf.h"

/****************************************************************************
**                        FUNCTION DEFINITION
****************************************************************************/
/**
 * \brief   This function configures a DMTimer instance for performance
 *          measurement.
 *
 * \param   None
 *
 * \return  None.
 *
 */
void PerfTimerSetup(void)
{
    SysPerfTimerSetup();
}

/**
 * \brief   This function starts performance timer
 *
 * \param   None
 *
 * \return  None
 */
void PerfTimerStart(void)
{
    SysPerfTimerConfig(1);
}

/**
 * \brief   This function stops performance timer and returns the time
 *          elapsed in number of ticks
 *
 * \param   None
 *
 * \return  Time elapsed in ticks
 */
unsigned int PerfTimerStop(void)
{
    return (SysPerfTimerConfig(0));
}


