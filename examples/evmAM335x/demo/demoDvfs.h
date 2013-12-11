/**
 *  \file   demoDvfs.h
 *
 *  \brief  Functions used to change the opp at which core operates.
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


#ifndef _DVFS_H_
#define _DVFS_H_

/**************************************************************************
**                       Macro Definitions
**************************************************************************/

/*
** MACROS to configure SEL bit filed in VDD1_OP_REG of PMIC.
** Refer the datasheet of PMIC for voltage values.
*/
#define     PMIC_VOLT_SEL_0950MV      (0x1fu)
#define     PMIC_VOLT_SEL_1100MV      (0x2bu)
#define     PMIC_VOLT_SEL_1200MV      (0x33u)
#define     PMIC_VOLT_SEL_1260MV      (0x38u)
#define     PMIC_VOLT_SEL_1325MV      (0x3du)

/*
** Structure for IO Pad Configuration
*/
typedef struct oppConfig
{
    unsigned int pllMult;
    unsigned int pmicVolt;
} tOPPConfig;

/*
** Structure for IO Pad Configuration
*/
typedef struct valueStringMap
{
    unsigned int val;
    const char* str;
} tValStr;

/******************************************************************************
**                      EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned int mpuOpp;
extern unsigned int mpuFreq;
extern unsigned int mpuVdd1;

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern unsigned int DemoOppChange(unsigned int Opp);
extern void VddVoltageConfigure(unsigned int Opp);
extern unsigned int FrequencyGet(unsigned int Opp);
extern unsigned int VddVoltageGet(unsigned int Opp);
extern unsigned int ConfigMinimumOPP(void);
extern unsigned int DemoMaxOppGet(void);
extern unsigned int ConfigMaximumOPP(void);
extern void MPUConfigure(unsigned int freq);
extern unsigned int OppGet(void);
extern void PrintConfigDVFS(void);
extern void ActionDVFSOpp50(void);
extern void ActionDVFSOpp100(void);
extern void ActionDVFSOpp120(void);
extern void ActionDVFSSrTurbo(void);
void ActionDVFSNitro(void);

#endif
