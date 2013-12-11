//*****************************************************************************
//
// ustdlib.h - Prototypes for simple standard library functions.
//
// This is part of AM1808 Sitaraware firmware package, modified and reused from revision 6288
// of the DK-LM3S9B96 Firmware Package.
//
//*****************************************************************************

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

#ifndef __USTDLIB_H__
#define __USTDLIB_H__

#include <stdarg.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup ustdlib_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! A structure that contains the broken down date and time.
//
//*****************************************************************************
typedef struct
{
    //
    //! The number of years since 0 AD.
    //
    unsigned short usYear;

    //
    //! The month, where January is 0 and December is 11.
    //
    unsigned char ucMon;

    //
    //! The day of the month.
    //
    unsigned char ucMday;

    //
    //! The day of the week, where Sunday is 0 and Saturday is 6.
    //
    unsigned char ucWday;

    //
    //! The number of hours.
    //
    unsigned char ucHour;

    //
    //! The number of minutes.
    //
    unsigned char ucMin;

    //
    //! The number of seconds.
    //
    unsigned char ucSec;
}
tTime;

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern int uvsnprintf(char *pcBuf, unsigned int ulSize, const char *pcString,
                      va_list vaArgP);
extern int usprintf(char *pcBuf, const char *pcString, ...);
extern int usnprintf(char *pcBuf, unsigned int ulSize, const char *pcString,
                     ...);
extern void ulocaltime(unsigned int ulTime, tTime *psTime);
extern unsigned int ustrtoul(const char *pcStr, const char **ppcStrRet,
                              int iBase);
extern char *ustrstr(const char *pcHaystack, const char *pcNeedle);
extern int ustrnicmp(const char *pcStr1, const char *pcStr2, int iCount);
extern int ustrcasecmp(const char *pcStr1, const char *pcStr2);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USTDLIB_H__
