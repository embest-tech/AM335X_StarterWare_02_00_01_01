/*
 * \file bmpToRaster.c
 *
 * \brief  Generates a header file for Raster from a BMP image
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



/* BitmapConvert.h */
#include<stdio.h>
#include "BitmapReader.h"
#include<sys/types.h>
#include<sys/stat.h>
#include<unistd.h>
#include<fcntl.h>
#include<unistd.h>

#define	BYTE_PER_PIXEL  3

BITMAPFILEHEADER		g_bmpFileHdr;	/*Buffer for Bitmap File header*/
BITMAPINFO			g_bmpInfo;	/*Buffer for Bitmap Header*/
BmpFormat			g_bitCount;	/*Bit Format for bitmap*/
IMAGEPTR                        Image;
enum pixelOdering               oderingType;
unsigned short int		format;

int main(int argc,char *argv[])
{
	int 			retCode = 0;
	int 			fd;
	unsigned int 		dataOffset;
	FILE			*fp;
	/*Check whether format given by user is correct or not*/
	if(argc != 7)
	{
		printf("Please execute the file in this format\n");
		printf("./a.out <No.of ROWS> <No.of Columns> <Source File Name> <Destination File name> <image format required 565/24>");
                printf("<pixel odering RGB/BGR>\n");
		exit(1);
	}
	/* Try to open the file now */
	fd = open(argv[3],O_RDONLY);
	if(fd < 0)
	{
		printf("\n BMP reader failed to open file\n");
		return -1;
	}
	else
	{
		/* Read the Bitmap file header first */
		read(fd,(void*)&g_bmpFileHdr,sizeof(BITMAPFILEHEADER));

		/* Now read the Bitmap Header */
		read(fd,(void*)&g_bmpInfo,sizeof(BITMAPINFO));

		/* Check the bit format of the bitmap */
		g_bitCount = (BmpFormat) g_bmpInfo.bmiHeader.biBitCount;

		if((g_bmpInfo.bmiHeader.biWidth != atoi(argv[1])) || (g_bmpInfo.bmiHeader.biHeight != atoi(argv[2])))
		{
			printf("Given Image is not in %s*%s format. Hence cannot process the image\n",argv[1],argv[2]);
			exit(0);
		}
		fp =(FILE* ) fopen(argv[4],"w");

		/* Get the offset of real data */
		dataOffset = g_bmpFileHdr.bfOffBits;

		/* Move the pointer by two bytes backword (4 byte alignment) */
		lseek(fd, dataOffset, SEEK_SET);

		/* Now read the raw data */
                if(!strcmp(argv[5],"565"))
                {
		        Image.pwFrameBuffer_16 = (unsigned short int *) malloc(((g_bmpInfo.bmiHeader.biWidth) * (g_bmpInfo.bmiHeader.biHeight) * 2));
                        format = 0;
                }
                else if(!(strcmp(argv[5],"24")))
                {
		        Image.pwFrameBuffer_24 = (unsigned int *) malloc(((g_bmpInfo.bmiHeader.biWidth) * (g_bmpInfo.bmiHeader.biHeight) * 4));

                        if(!(strcmp(argv[6],"RGB")))
                        {
                             oderingType = RGB;
                        }
                        else if(!(strcmp(argv[6],"BGR")))
                        {
                             oderingType = BGR;
                        }
                        else
                        {
                             printf("specify pixel odering RGB/BGR");
                        }
 
                        format = 1;
                }
                else
                {
                        printf("format is niether 565 nor 24");
                        return -1;
                }
 
		ProcessDataForFrameBuffer(fd,g_bmpInfo.bmiHeader.biSizeImage,&Image,fp);

		/* We need to close the file */
		close(fd);

	}
	return retCode;
}

void ProcessDataForFrameBuffer(int fd, unsigned int dwSize, IMAGEPTR* Image, FILE* fp)
{
	
	int				i, j, pos = 0,l = 0, flag = 0;
	int				row = g_bmpInfo.bmiHeader.biHeight;		
	int				col = g_bmpInfo.bmiHeader.biWidth;	
	unsigned char			blue, red, green;
        unsigned char*                  rowData;
	unsigned short int		rowMultiplier;
	unsigned short int*		pPixel_16;
	unsigned int*		        pPixel_24;
        #ifdef  COMPRESS
        unsigned int                    Pixel24Val;
        unsigned int                    Pixel24Cnt;
        #endif
	unsigned short int*		temp;
	unsigned char*			pData;
	char 				comma = ',',new_line = '\n';
	char a[]  = "unsigned short const image[] = {";
        #ifdef  COMPRESS
	char d[] = "0x01004000u, 0x07000001u, ";
        #else
	char d[] = "0x4000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u,";
        #endif
	char e[] = "0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u, 0x0000u,";
	char c[] = "};";
	char f[] = "unsigned int const image[] = {";

        /* Now read the raw data */
        if(format == 0)
        {
	      rowData = (unsigned char *) malloc((g_bmpInfo.bmiHeader.biWidth) * 2);	
        }
        else
        {
	      rowData = (unsigned char *) malloc((g_bmpInfo.bmiHeader.biWidth) * 3);	
        }

	pos = dwSize;
	rowMultiplier = (col)*(BYTE_PER_PIXEL);			/*rowMultiplier is size of each pixel(row) array*/
	if((col*BYTE_PER_PIXEL)%4)				/*If not a multilier of 4 then pad rowMultiplier with zeros */
	{
		int extraZeros = 4 - ((col*BYTE_PER_PIXEL)%4);
		rowMultiplier += extraZeros;
	}

	/*File to store Raw Pixel Data*/
        if(format == 0)
        {
	      fprintf(fp,"%s\n",a);
	      fprintf(fp,"%s\n",d);
	      fprintf(fp,"%s",e);
	      fprintf(fp,"%c",new_line);
        }
        else
        {
	      fprintf(fp,"%s\n", f);
	      fprintf(fp,"%s\n",d);
        }



	for(i = (row - 1); i >= 0; i--)
	{
		read(fd,(void*)rowData, rowMultiplier);

                if(format == 0)
                {
		   pPixel_16 = (unsigned short int *)&Image->pwFrameBuffer_16[i * g_bmpInfo.bmiHeader.biWidth];
                }
                else
                {                
		   pPixel_24 = (unsigned int *)&Image->pwFrameBuffer_24[i * g_bmpInfo.bmiHeader.biWidth];
                }

		pos = 0;
		pData = rowData;
		for(j = 0; j < col; j++)
		{
			blue = pData[pos++];

			green = pData[pos++];

			red = pData[pos++];
 
                        if(format == 0)
                        {  
		             *pPixel_16 = RGB16(red, green, blue);
	                      pPixel_16++; 
                        }
                        else
                        {    if(RGB == oderingType)
                             {
		                  *pPixel_24 = (unsigned int)RGB24(red, green, blue);
                                   pPixel_24 = pPixel_24 + 1;
                             }
                             else if(BGR == oderingType)
                             {
                                   *pPixel_24 = (unsigned int)BGR24(blue, green, red);
                                    pPixel_24 = pPixel_24 + 1;
                             }
                        }
		}
	}

        #ifdef   COMPRESS

        Pixel24Cnt = 0;

	for(i = 0;i <= (row - 1); i++)
	{
            pPixel_24 = (unsigned int *) &Image->pwFrameBuffer_24[i * g_bmpInfo.bmiHeader.biWidth];
            
	    for(j = 0;j < col; j++)
	    {
                Pixel24Val = *pPixel_24;
                Pixel24Cnt++;

                pPixel_24 = pPixel_24 + 1;
                //if((i+j) == (row + col - 2))
                  //   *pPixel_24 = Pixel24Val+1;

                if(Pixel24Val != *pPixel_24)
                {
                    if(Pixel24Cnt > 255)
                    {
                        fprintf(fp,"0x%08x", Pixel24Val);
		        fprintf(fp,"%c ",comma);
                        fprintf(fp,"0x%08x", Pixel24Cnt); 
		        fprintf(fp,"%c ",comma);
		        l += 2;
                    }
                    else
                    {
                        fprintf(fp,"0x%08x", ((Pixel24Val) | (Pixel24Cnt <<24)));
		        fprintf(fp,"%c ",comma);
		        l++;
                    }

                    Pixel24Cnt = 0;
                }

		if(l >= 10)
		{
		    fprintf(fp,"%c",new_line);
		    l = 0;
		}
	    }
	}

       #else

	for(i = 0;i <= (row - 1); i++)
	{
	        if(format == 0)
                {
		   pPixel_16 = (unsigned short int *) &Image->pwFrameBuffer_16[i * g_bmpInfo.bmiHeader.biWidth];
                }
                else
                {                
		   pPixel_24 = (unsigned int *) &Image->pwFrameBuffer_24[i * g_bmpInfo.bmiHeader.biWidth];
                }

		for(j = 0;j < col; j++)
		{
                         
                        if(format == 0)
                        {
			     fprintf(fp,"0x%04x", *pPixel_16);
			     pPixel_16++;
                        }
                        else
                        {
			     fprintf(fp,"0x%08x", *pPixel_24);
                             pPixel_24 = pPixel_24 + 1;

                        }
			fprintf(fp,"%c ",comma);
			l++;
			if(l == 10)
			{
				fprintf(fp,"%c",new_line);
				l = 0;
			}
		}
        }

        #endif

	fprintf(fp,"%s",c);
}
/* --------------------------------------- End of File --------------------------------*/
