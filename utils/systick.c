/**
 * \file   systick.c
 *
 * \brief  system timer tick routines. This can be used to call a function
 *		during specific intervels .
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


#include "systick.h"

/****************************************************************************
**                        FUNCTION DEFINITION                               
****************************************************************************/

/**
 * \brief   This function registers the periodic handler
 *          
 *          
 *          
 * \param  void (*pfnHandler)(void)  This is a pointer to the periodic handler.
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform 
 *            specific files
 *
 *
 */
void SystickConfigure(void (*pfnHandler)(void))
{    
	 TimerTickConfigure(pfnHandler);
}


/**
 * \brief   This function sets the  period to call the Handler
 *          
 *          
 *          
 * \param  milliSec This is the number of milli seconds for the period
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickPeriodSet(unsigned int milliSec)
{
	TimerTickPeriodSet(milliSec);
	
}

/**
 * \brief   This function Enables the Systick. This has to be called 
 *           to start the periodic function which is registered
 *          
 *          
 * \param  None
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickEnable(void)
{	

	/* Start the timer. Characters from cntArr will be sent from the ISR */
	TimerTickEnable();
}

/**
 * \brief   This function Disables the Systick. This has to be called 
 *           to stop the periodic function which is registered
 *          
 *          
 * \param  milliSec This is the number of milli seconds for the period
 *                          
 *
 *
 * \return  None.
 *
 * \Note   This a wrapper API, The Actual implementation can found in platform
 *            specific files
 *
 *
 */
void SystickDisable(void)
{
	TimerTickDisable();
}



