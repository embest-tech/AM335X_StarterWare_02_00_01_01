/**
 * \file   misc.c
 *
 * \brief  This file contain miscellaneous functions for standard operations.
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

#include "misc.h"

/*****************************************************************************
**                   MACRO DEFINITIONS
*****************************************************************************/

#define MASK_HOUR          			(0xFF000000)
#define MASK_MINUTE        			(0x00FF0000)
#define MASK_SECOND        			(0x0000FF00)
#define MASK_MERIDIEM      			(0x000000FF)

#define HOUR_SHIFT         			(24)
#define MINUTE_SHIFT       			(16)
#define SECOND_SHIFT       			(8)

#define MASK_DAY           			(0xFF000000)
#define MASK_MONTH         			(0x00FF0000)
#define MASK_YEAR          			(0x0000FF00)
#define MASK_DOTW          			(0x000000FF)

#define DAY_SHIFT          			(24)
#define MONTH_SHIFT        			(16)
#define YEAR_SHIFT         			(8)

/****************************************************************************
**                        FUNCTION DEFINITION
****************************************************************************/

/**
 * \brief   This function adds two BCD numbers.
 *
 * \param   bcd1  First BCD number to be added.
 *
 * \param   bcd2  Second BCD number to be added.
 *
 * \return  Sum of two given BCD numbers.
 */

unsigned short bcdAdd(unsigned char bcd1, unsigned char bcd2)
{
    unsigned int index = 0;
    unsigned int carry = 0;
    unsigned short result = 0;
    unsigned int tempVal = 0;
    unsigned int shift = 0;

    while((sizeof(bcd1)*2) > index)
    {
        shift = index * 4;

        tempVal = ((bcd1 & (0xF << shift)) + (bcd2 & (0xF << shift))) >> shift;
        tempVal += carry;
        carry = 0;

        tempVal = ((tempVal >= 10) ? (tempVal+6) : tempVal);
        carry = (tempVal & 0xF0) ? 1 : 0;

        result |= (tempVal & 0x0F) << shift;

        index++;
    }
    result += (carry * 0x100);
    return result;
}

/**
 * \brief   This function adds two time values in 24 hrs format and updates date.
 *
 * \param   time1  First time value (0xHHMMSSxx) in BCD format.
 *
 * \param   time2  Second time value (0xHHMMSSxx) in BCD format.
 *
 * \param   date   Input date for time1 and effective date after adding time2\n
 *                 in BCD format (0xDDMMYYWD).
 *
 * \return  Sum of two given times in BCD format.
 *
 * Note:
 * 0xHHMMSSxx  [31:24]-Hours, [23:16]-Minutes, [15:8]-Seconds.
 * 0xDDMMYYWD  [31:24]-Day of [23:16]-Month in [15:8]-Year, [7:0]-Day of Week.
 */

unsigned int addTime(unsigned int time1, unsigned int time2, unsigned int *date)
{
    unsigned int absAlarmSec = 0;
    unsigned int absAlarmMin = 0;
    unsigned int absAlarmHr = 0;

    unsigned int absAlarmDay = 0;
    unsigned int absAlarmMon = 0;
    unsigned int absAlarmYear = 0;
    unsigned int absAlarmDotw = 0;

    unsigned int resTime = 0;
    unsigned int lastDay = 0;

    /*	BCD addition	*/
    absAlarmSec = bcdAdd((unsigned char)((time1 & MASK_SECOND) >> SECOND_SHIFT),
                        (unsigned char)((time2 & MASK_SECOND) >> SECOND_SHIFT));
    if(absAlarmSec >= 0x60)
    {
        absAlarmMin++;
        absAlarmSec %= 0x60;
    }

    absAlarmMin = bcdAdd((unsigned char)((time1 & MASK_MINUTE) >> MINUTE_SHIFT),
          ((unsigned char)((time2 & MASK_MINUTE) >> MINUTE_SHIFT))+absAlarmMin);
    if(absAlarmMin >= 0x60)
    {
        absAlarmHr++;
        absAlarmMin %= 0x60;
    }

    absAlarmHr = bcdAdd((unsigned char)((time1 & MASK_HOUR) >> HOUR_SHIFT),
               ((unsigned char)((time2 & MASK_HOUR) >> HOUR_SHIFT))+absAlarmHr);
    if(absAlarmHr >= 0x24)
    {
        absAlarmDay++;
        absAlarmDotw++;
        absAlarmHr %= 0x24;
    }

    resTime = ((absAlarmHr << HOUR_SHIFT) |
               (absAlarmMin << MINUTE_SHIFT) |
               (absAlarmSec << SECOND_SHIFT));

    if(1 == absAlarmDay)
    {
        /*	adjust day	*/
        switch(((*date) & MASK_MONTH) >> MONTH_SHIFT)
        {
            case 1:
            case 3:
            case 5:
            case 7:
            case 8:
            case 10:
            case 12:
                lastDay = 0x31;
                break;

            case 4:
            case 6:
            case 9:
            case 11:
                lastDay = 0x30;
                break;

            case 2:
                /*	Leap year	*/
                if(0 == ((((*date) & MASK_YEAR) >> YEAR_SHIFT)%4))
                {
                    lastDay = 0x29;
                }
                else /*	Not leap year	*/
                {
                    lastDay = 0x28;
                }
                break;
        }

        if(lastDay == (((*date) & MASK_DAY) >> DAY_SHIFT))
        {
            absAlarmMon++;
            absAlarmDay = 1;
        }
        else
        {
            absAlarmDay += (((*date) & MASK_DAY) >> DAY_SHIFT);
        }

        /*	adjust date of week	*/
        absAlarmDotw++;
        absAlarmDotw = absAlarmDotw % 6;

        /*	adjust month	*/
        absAlarmMon += (((*date) & MASK_MONTH) >> MONTH_SHIFT);
        if(absAlarmMon >= 0x12)
        {
            absAlarmYear++;
            absAlarmMon = absAlarmMon - 0x12;
        }

        /*	adjust year	*/
        absAlarmYear += (((*date) & MASK_YEAR) >> YEAR_SHIFT);

        if((absAlarmDay & 0x0F) >= 10)
            absAlarmDay += 6;

        if((absAlarmMon & 0x0F) >= 10)
            absAlarmMon += 6;

        if((absAlarmYear & 0x0F) >= 10)
            absAlarmYear += 6;

        (*date) = ((absAlarmDay << DAY_SHIFT) |
                   (absAlarmMon << MONTH_SHIFT) |
                   (absAlarmYear << YEAR_SHIFT) |
                   (absAlarmDotw));
    }

    return resTime;
}
