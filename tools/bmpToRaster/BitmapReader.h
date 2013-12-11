/*
 * \file BitMapHeader.h
 *
 * \brief  contents of a BMP image
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


#pragma once

#include<stdio.h>
#include<stdlib.h>
#define RGB16(red, green, blue) ( ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3))
#define RGB24(red, green, blue)  ((red << 16) | (green << 8) | (blue))
#define BGR24(blue, green, red)  ((blue<< 16) | (green << 8) | (red))

#define BMPREADER_SUCCESS		0
#define BMPREADER_FAILURE		-1

#define COMPRESS

typedef enum BmpFormat
{
	Bit_1 = 1,
	Bit_4 = 4,
	Bit_8 = 8,
	Bit_24 = 24,
} BmpFormat;

#pragma pack (push, 1)

typedef struct tagBITMAPFILEHEADER 
{
	unsigned short int			bfType;		/*Magic Number used to identify the BMP file*/ 
	unsigned int				bfSize; 	/*Size of BMP file in bytes*/	
	unsigned short int			bfReserved1; 	/*Reserved*/
	unsigned short int			bfReserved2; 	/*Reserved*/
	unsigned int				bfOffBits; 	/*Offset i.e. starting address of the byte where bitmap image data can be found*/
} BITMAPFILEHEADER; 

typedef struct tagBITMAPINFOHEADER 
{ 
	unsigned int				biSize; 	/*Size of header*/
	unsigned int				biWidth; 	/*Bitmap width in pixels*/
	unsigned int 				biHeight; 	/*Bitmap height in pixels*/
	unsigned short int			biPlanes; 	/*Number of colour planes being used*/
	unsigned short int			biBitCount;	/*Number of bits per pixel (colour depth of image)(1,4,8,16,24,32)*/
	unsigned int				biCompression; 	/*Compression method being used*/
	unsigned int				biSizeImage; 	/*Image size (Size of Raw bitmap data)*/
	unsigned int 				biXPelsPerMeter;/*Horizontal Resolution of the image*/ 
	unsigned int 				biYPelsPerMeter;/*Vertical Resolution of the image*/ 
	unsigned int				biClrUsed; 	/*Number of colours in the colour palette*/
	unsigned int 				biClrImportant; /*Number of important colours used*/
} BITMAPINFOHEADER; 

typedef struct tagRGBQUAD 
{ 
	 unsigned char				rgbBlue;	/*8 Bits for BLUE*/
 	 unsigned char				rgbGreen;	/*8 Bits for GREEN*/ 
 	 unsigned char				rgbRed;		/*8 Bits for RED*/
 	 unsigned char				rgbReserved;	
} RGBQUAD;

typedef struct tagBITMAPINFO 
{ 
	BITMAPINFOHEADER bmiHeader; 
	RGBQUAD bmiColors[1]; 
} BITMAPINFO;

typedef struct tagIMAGEHOLDER
{
        unsigned short int* pwFrameBuffer_16;
        unsigned int* pwFrameBuffer_24;
}IMAGEPTR;

enum pixelOdering {RGB, BGR}; 

#pragma pack(pop)

int ReadBmpFile(const char* szFileName, IMAGEPTR* pwFrameBuffer);
/* void ProcessDataForFrameBuffer(Uint8* pData, Uint32 dwSize, Uint16* pwFrameBuffer); */
void ProcessDataForFrameBuffer(int fd, unsigned int dwSize, IMAGEPTR* pwFrameBuffer,FILE*fp);

/* ---------------------------------- End of File ---------------------- */


