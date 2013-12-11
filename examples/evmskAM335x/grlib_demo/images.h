//*****************************************************************************
//
// images.h - Prototypes for the images used by the application.
//
// This is part of revision 6288 of the DK-LM3S9B96 Firmware Package.
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

#ifndef __IMAGES_H__
#define __IMAGES_H__

//*****************************************************************************
//
// Prototypes for the image arrays.
//
//*****************************************************************************
extern const unsigned char g_pucLightOff[];
extern const unsigned char g_pucLightOn[];
extern const unsigned char g_pucRedSlider195x37[];
extern const unsigned char g_pucGreenSlider195x37[];
extern const unsigned char g_pucGettingHotter28x148[];
extern const unsigned char g_pucGettingHotter28x148Mono[];
// 16bpp
extern const unsigned char g_pucLogo[];
extern const unsigned char g_prev50x50[];
extern const unsigned char g_next50x50[];
extern const unsigned char g_TILogo[];
extern const unsigned char g_ledOFF[];
extern const unsigned char g_ledON[];
extern const unsigned char g_pucBlue37x37[];
extern const unsigned char g_pucBlue37x37Press[];

//32bpp (24bpp with hole)
extern const unsigned char baseImage[];

#endif // __IMAGES_H__
