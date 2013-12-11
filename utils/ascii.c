/**
 * \file   ascii.c
 *
 * \brief  This file contain functions which compute ASCII for standard
 *         operations.
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

#include "ascii.h"

/****************************************************************************
**                        FUNCTION DEFINITION
****************************************************************************/

/**
 * \brief   This function converts the ASCII value of a digit to hexadecimal
 *          or decimal number.
 *
 * \param   byte  ASCII value of a digit. Decimal digits ('0' - '9').
 *                Hexadecimal digits ('0' - '9', 'A' - 'F' or 'a' - 'f').
 *
 * \param   base  Base value selection of input.
 *                BASE_DECIMAL for Decimal.
 *                BASE_HEXADECIMAL for Hexadecimal.
 *
 * \return  Hexadecimal (0x0 - 0xF) or decimal (0 - 9) number.
 *          0xFF - Not a valid ASCII value of digit.
 */

unsigned char ASCIIToDigit(unsigned char byte, unsigned int base)
{
    unsigned char retVal = 0;

    if((base != BASE_DECIMAL) && (base != BASE_HEXADECIMAL))
    {
        ;
    }
    /* For numbers from 0x0 to 0x9.*/
    else if((byte >= 48) && (byte <= 57))
    {
        retVal = byte - 48;
        return retVal;
    }
    /* For alphabets from A to F.*/
    else if((base == BASE_HEXADECIMAL) && (byte >= 'A') && (byte <= 'F'))
    {
        retVal = byte - 55; // 55 = ('0' + ('A' - '9' - 1))
        return retVal;
    }
    /* For alphabets from a to f.*/
    else if((base == BASE_HEXADECIMAL) && (byte >= 'a') && (byte <= 'f'))
    {
       retVal = byte - 87; // 87 = ('0' + ('a' - '9' - 1))
        return retVal;
    }

    return 0xFF;
}

/**
 * \brief   This function gives the ASCII value of a digit to hexadecimal
 *          or decimal number.
 *
 * \param   byte  Hexadecimal or decimal digit. Upper 4-bits are don't care.
 *                Hexadecimal digits (0x0 - 0xF). Decimal digits (0 - 9).
 *
 * \param   base  Base value selection of input.
 *                BASE_DECIMAL for Decimal.
 *                BASE_HEXADECIMAL for Hexadecimal.
 *
 * \return  ASCII value of Hexadecimal ('0' - '9', 'A' - 'F') or decimal
 *                digits ('0' - '9') digits.
 *          0xFF - Not a valid digit.
 */

unsigned char DigitToASCII(unsigned char byte, unsigned int base)
{
    unsigned int retVal = 0;
    unsigned char nibble = 0x0F & byte;

    if((base != BASE_DECIMAL) && (base != BASE_HEXADECIMAL))
    {
        ;
    }
    /* For numbers from 0x0 to 0x9.*/
    else if(nibble <= 9)
    {
        retVal = nibble + 48;
        return retVal;
    }
    /* For alphabets from A to F.*/
    else if((base == BASE_HEXADECIMAL) && (nibble >= 10) && (nibble <= 15))
    {
        retVal = nibble + 55; // 55 = ('0' + ('A' - '9' - 1))
        return retVal;
    }

    return 0xFF;
}

/**
 * \brief   This function converts string to hexadecimal Ethernet Address.
 *          '-' is expected as delimitter like 01-23-45-67-89-ab.
 *
 * \param   strInput  Input String of Ethernet Address.
 *
 * \param   ethAddr   Hexadecimal Ethernet Address output.
 *
 * \return  Number of character read from input string.
 */

unsigned int StrToEthrAddr(unsigned char *strInput, unsigned char *ethAddr)
{
    unsigned int index = 0;
    unsigned char lower = 0;
    unsigned char upper = 0;

    for(index = 0; index < 6; index++)
    {
        lower = ASCIIToDigit(strInput[3 * index + 1], BASE_HEXADECIMAL);
        upper = ASCIIToDigit(strInput[3 * index], BASE_HEXADECIMAL);

        if((index != 5) && (strInput[3 * index + 2] != '-'))
        {
            return 3*index;
        }
        else if((lower != 0xFF) && (upper != 0xFF))
        {
            ethAddr[index] = (upper << 4) | lower;
        }
        else
        {
            return 3*index;
        }
    }

    return 3*index;
}

/**
 * \brief   This function converts Ethernet Address to string. '-' is provide
 *          as delimitter like 01-23-45-67-89-ab.
 *
 * \param   ethrAddrInput  Hexadecimal Ethernet Address output.
 *
 * \param   strOutput      Output String of Ethernet Address.
 *
 * \return  Number of bytes read from input.
 */

unsigned int EthrAddrToStr(unsigned char *ethrAddrInput,
                           unsigned char *strOutput)
{
    unsigned int index = 0;
    unsigned char lower = 0;
    unsigned char upper = 0;

    for(index = 0; index < 6; index++)
    {
        upper = DigitToASCII(ethrAddrInput[index] >> 4, BASE_HEXADECIMAL);
        lower = DigitToASCII(ethrAddrInput[index], BASE_HEXADECIMAL);

        if((lower != 0xFF) && (upper != 0xFF))
        {
            strOutput[3 * index + 1] = lower;
            strOutput[3 * index] = upper;

            if(index == 5)
                strOutput[3 * index + 2] = '\0';
            else
                strOutput[3 * index + 2] = '-';
        }
        else
        {
            return index;
        }
    }

    return index;
}

/**
 * \brief   This function converts Time to string.
 *
 * \param   timeInput      Time input as [15:8]-Sec, [23:16]-Min, [31:24]-Hour.
 *
 * \param   strOutput      Output String of Time in HH:MM:SS.
 *
 * \return  Number of bytes read from input.
 */

unsigned int TimeToStr(unsigned int timeInput, unsigned char *strOutput)
{
    unsigned int index = 0;
    unsigned char lower = 0;
    unsigned char upper = 0;

    for(index = 0; index < 3; index++)
    {
        upper = DigitToASCII(timeInput >> (8 * index + 12), BASE_DECIMAL);
        lower = DigitToASCII(timeInput >> (8 * index + 8), BASE_DECIMAL);

        if((lower != 0xFF) && (upper != 0xFF))
        {
            strOutput[3 * (2 - index) + 1] = lower;
            strOutput[3 * (2 - index)] = upper;

            if(index == 0)
                strOutput[3 * (2 - index) + 2] = '\0';
            else
                strOutput[3 * (2 - index) + 2] = ':';
        }
        else
        {
            return index;
        }
    }

    return index;
}

/**
 * \brief  This function converts Date to string.
 *
 * \param   dateInput      Date input as ([15:8]-Year,[23:16]-Mon,[31:24]-Day).
 *
 * \param   strOutput      Output String of Date (DD/MM/YY).
 *
 * \return  Number of bytes read from input.
 */

unsigned int DateToStr(unsigned int dateInput, unsigned char *strOutput)
{
    unsigned int index = 0;
    unsigned char lower = 0;
    unsigned char upper = 0;

    for(index = 0; index < 3; index++)
    {
        upper = DigitToASCII(dateInput >> (8 * index + 12), BASE_DECIMAL);
        lower = DigitToASCII(dateInput >> (8 * index + 8), BASE_DECIMAL);

        if((lower != 0xFF) && (upper != 0xFF))
        {
            strOutput[3 * (2 - index) + 1] = lower;
            strOutput[3 * (2 - index)] = upper;

            if(index == 0)
                strOutput[3 * (2 - index) + 2] = '\0';
            else
                strOutput[3 * (2 - index) + 2] = '/';
        }
        else
        {
            return index;
        }
    }

    return index;
}
