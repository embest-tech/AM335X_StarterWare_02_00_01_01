//*****************************************************************************
//
// offscr16bpp.c - 16 BPP off-screen display buffer driver.
//
// Copyright (c) 2008-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************

#include "debug.h"
#include "grlib.h"

unsigned short offset;

//*****************************************************************************
//
//! \addtogroup primitives_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param ulValue is the 24-bit RGB color.  The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is the
//! red channel.
//!
//! This function translates a 24-bit RGB color into a value that can be
//! written into the display's frame buffer in order to reproduce that color,
//! or the closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//
//*****************************************************************************
static unsigned int
GrOffScreen24BPPColorTranslate(void *pvDisplayData, unsigned int ulValue)
{
	return ulValue;
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
static void
GrOffScreen24BPPRectFill(void *pvDisplayData, const tRectangle *pRect,
                        unsigned int ulValue)
{
    unsigned char *pucData, *pucColumn;
    int lBytesPerRow, lX, lY;

    //
    // Check the arguments.
    //
    ASSERT(pvDisplayData);
    ASSERT(pRect);

    //
    // Create a character pointer for the display-specific data (which points
    // to the image buffer).
    //
    pucData = (unsigned char *)pvDisplayData;

    //
    // Compute the number of bytes per row in the image buffer.
    //
    lBytesPerRow = (*(unsigned short *)(pucData)) * 4;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData +=  4 + (16 * 2) + ((lBytesPerRow * pRect->sYMin) + (pRect->sXMin * 4));

    //
    // Get the starting X and Y coordinate of the rectangle.
    // Also, get the number of pixels per line and the number of lines 
    //
    lX = pRect->sXMin;

    //
    // Now we are at a 4 byte aligned address. We can copy word wise. We start at
    // the first row and fill the entire column (this offset for each row)
    //
    while((lX) <= pRect->sXMax)
    {
        for(lY = pRect->sYMin, pucColumn = pucData; lY < pRect->sYMax; lY++, pucColumn += lBytesPerRow)
        {
            *(unsigned int *)pucColumn = ulValue;
        }
        pucData += 4;
		lX += 1;
    }
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX1 is the X coordinate of the start of the line.
//! \param lX2 is the X coordinate of the end of the line.
//! \param lY is the Y coordinate of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
GrOffScreen24BPPLineDrawH(void *pvDisplayData, int lX1, int lX2, int lY,
                         unsigned int ulValue)
{
    unsigned char *pucData;
    int lBytesPerRow;

    //
    // Check the arguments.
    //
    ASSERT(pvDisplayData);

    //
    // Create a character pointer for the display-specific data (which points
    // to the image buffer).
    //
    pucData = (unsigned char *)pvDisplayData;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    lBytesPerRow = (*(unsigned short *)(pucData)) * 4;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += ((lBytesPerRow * lY) + (lX1 * 4) + 4 + (16*2));

    //
    // Now we are at a 4 byte aligned address. We can copy word wise. We start at
    //
    while((lX1) <= lX2)
    {
        //
        // Draw 1 pixel
        //
        *(unsigned int *)pucData = ulValue;
        pucData += 4;
        lX1 += 1;
    }
}


//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the line.
//! \param lY1 is the Y coordinate of the start of the line.
//! \param lY2 is the Y coordinate of the end of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
GrOffScreen24BPPLineDrawV(void *pvDisplayData, int lX, int lY1, int lY2,
                         unsigned int ulValue)
{
    unsigned char *pucData;
    int lBytesPerRow;

    //
    // Check the arguments.
    //
    ASSERT(pvDisplayData);

    //
    // Create a character pointer for the display-specific data (which points
    // to the image buffer).
    //
    pucData = (unsigned char *)pvDisplayData;

    //
    // Compute the number of bytes per row in the image buffer.
    //
    lBytesPerRow = (*(unsigned short *)(pucData)) * 4;    

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += ((lBytesPerRow * lY1) + (lX * 4) + 4 + (16*2));

    //
    // Loop over the rows of the line.
    //
    for(; lY1 <= lY2; lY1++)
    {
        *(unsigned int *)pucData = ulValue;
        pucData += lBytesPerRow;
    }
}



//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void
GrOffScreen24BPPPixelDraw(void *pvDisplayData, int lX, int lY,
                         unsigned int ulValue)
{
    unsigned char *pucData;
    int lBytesPerRow;

    //
    // Check the arguments.
    //
    ASSERT(pvDisplayData);

    //
    // Create a character pointer for the display-specific data (which points
    // to the image buffer).
    //
    pucData = (unsigned char *)pvDisplayData;

    //
    // Get the offset to the byte of the image buffer that contains the pixel
    // in question.
    //
    lBytesPerRow = (*(unsigned short *)(pucData)) * 4;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += (lBytesPerRow * lY) + (lX * 4) + 4 + (16*2);
    //pucData += (*(unsigned short *)(pucData + 1) * lY * 2) + (lX * 2) + 4 + (16*2);

    //
    // Write this pixel into the image buffer.
    //
	*(unsigned int *)pucData = ulValue;
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the first pixel.
//! \param lY is the Y coordinate of the first pixel.
//! \param lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param lCount is the number of pixels to draw.
//! \param lBPP is the number of bits per pixel; must be 1, 4, 8, 16, or 24.
//! \param pucData is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pucPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void
GrOffScreen24BPPPixelDrawMultiple(void *pvDisplayData, int lX, int lY,
                                 int lX0, int lCount, int lBPP,
                                 const unsigned char *pucData,
                                 const unsigned char *pucPalette)
{
    unsigned int ulByte;
    int lBytesPerRow;
    unsigned char *pucPtr;	
    
    //
    // Check the arguments.
    //
    ASSERT(pvDisplayData);
    ASSERT(pucData);
    ASSERT(pucPalette);

    //
    // Create a character pointer for the display-specific data (which points
    // to the image buffer).
    //
    pucPtr = (unsigned char *)pvDisplayData;

    //
    // Compute the number of bytes per row in the image buffer.
    //
    lBytesPerRow = (*(unsigned short *)(pucPtr)) * 4;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucPtr += (lBytesPerRow * lY) + (lX * 4) + 4 + (16*2);

    //
    // Determine how much to shift to get to the nibble that contains this
    // pixel.
    //
    lX = (1 - (lX & 1)) * 4;

    //
    // Determine how to interpret the pixel data based on the number of bits
    // per pixel.
    //
    switch(lBPP)
    {
        //
        // The pixel data is in 1 bit per pixel format.
        //
        case 1:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount)
            {
                //
                // Get the next byte of image data.
                //
                ulByte = *pucData++;

                //
                // Loop through the pixels in this byte of image data.
                //
                for(; (lX0 < 8) && lCount; lX0++, lCount--)
                {
                    //
                    // Draw this pixel in the appropriate color.
                    //
                    ulByte = ((*pucPtr & ~(0xf << lX)) |
                               (((unsigned int *)pucPalette)[(ulByte >>
                                                               (7 - lX0)) &
                                                              1] << lX));
                    //
                    // Write this pixel to the screen.
                    //
                    *pucPtr++ = (ulByte & 0x00FF);
                    *pucPtr++ = (ulByte & 0xFF00) >> 8;
                    
                }

                //
                // Start at the beginning of the next byte of image data.
                //
                lX0 = 0;
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 4 bit per pixel format.
        //
        case 4:
        {
            //
            // Loop while there are more pixels to draw.  "Duff's device" is
            // used to jump into the middle of the loop if the first nibble of
            // the pixel data should not be used.  Duff's device makes use of
            // the fact that a case statement is legal anywhere within a
            // sub-block of a switch statement.  See
            // http://en.wikipedia.org/wiki/Duff's_device for detailed
            // information about Duff's device.
            //
            switch(lX0 & 1)
            {
                case 0:
                    while(lCount)
                    {
                        //
                        // Get the upper nibble of the next byte of pixel data
                        // and extract the corresponding entry from the
                        // palette.
                        //
                        #ifdef SUPPORT_UNALIGNED
                            ulByte = (*pucData >> 4) * 3;
                        #else
                            offset = (*pucData >> 4) * 3;
                            ulByte  = (*(unsigned char *)(pucPalette + offset));
                            ulByte |= (*((unsigned char *)(pucPalette + offset) + 1)) << 8;
                            ulByte |= *((unsigned char *)(pucPalette + offset) + 2) << 16;
                        #endif

                        //
                        // Write this pixel to the screen.
                        //
                        *(unsigned int *)pucPtr = (*(unsigned int *)(pucPalette + ulByte) & 0x00ffffff);
						pucPtr+=4;
                        
                        //
                        // Decrement the count of pixels to draw.
                        //
                        lCount--;

                        //
                        // See if there is another pixel to draw.
                        //
                        if(lCount)
                        {
                case 1:
                            //
                            // Get the lower nibble of the next byte of pixel
                            // data and extract the corresponding entry from
                            // the palette.
                            //
                            #ifdef SUPPORT_UNALIGNED
                                ulByte = (*pucData++ & 15) * 3;
                            #else
                                offset = (*pucData++ & 15) * 3;
                                ulByte  = (*(unsigned char *)(pucPalette + offset));
                                ulByte |= (*((unsigned char *)(pucPalette + offset) + 1)) << 8;
                                ulByte |= *((unsigned char *)(pucPalette + offset) + 2) << 16;
                            #endif

                            //
                            // Write this pixel to the screen.
                            //
                            *(unsigned int *)pucPtr = (*(unsigned int *)(pucPalette + ulByte) & 0x00ffffff);
							pucPtr+=4;

                            //
                            // Decrement the count of pixels to draw.
                            //
                            lCount--;
                        }
                    }
            }

            //
            // The image data has been drawn.
            //
            break;
        }

        //
        // The pixel data is in 8 bit per pixel format.
        //
        case 8:
        {
            //
            // Loop while there are more pixels to draw.
            //
            while(lCount--)
            {
                //
                // Get the next byte of pixel data and extract the
                // corresponding entry from the palette.
                //
                #ifdef SUPPORT_UNALIGNED
                    ulByte = *pucData++ * 3;
                    ulByte = *(unsigned int *)(pucPalette + ulByte) & 0x00ffffff;
                #else
                    offset = *pucData++ * 3;
                    ulByte  = (*(unsigned char *)(pucPalette + offset));
                    ulByte |= (*((unsigned char *)(pucPalette + offset) + 1)) << 8;
                    ulByte |= *((unsigned char *)(pucPalette + offset) + 2) << 16;
                #endif

                //
                // Translate this palette entry.
                //
                ulByte = GrOffScreen24BPPColorTranslate(pvDisplayData, ulByte);

                //
                // Write this pixel to the screen.
                //
                *pucPtr++ = (ulByte & 0x00FF);
                *pucPtr++ = (ulByte & 0xFF00) >> 8;
            }

				//
				// The image data has been drawn.
				//
				break;
			}
        case 16:
            {
                unsigned short pixColor = 0;	/*	16bpp	*/
                
                pucData++;
                while(lCount--)
                {
					pixColor = *(pucData+1);
					pixColor |= (*(pucData)) << 8;

					*pucPtr++ = (pixColor & 0x1F) << 3;			/* Blue */	
					*pucPtr++ = (pixColor & 0x07E0) >> 3;		/* Green */			
					*pucPtr++ = (pixColor & 0x0000F800) >> 8;   /* Red */	
					*pucPtr++ = 0x0;                        	/* Alpha */

					pucData += 2;
                }
                break;
            }
        case 32: /* 24bpp with 1-byte hole */
            {
                //pucData++;
                while(lCount--)
                {
                    *pucPtr++ = *(pucData+3);
                    *pucPtr++ = *(pucData+2);
                    *pucPtr++ = *(pucData+1);
                    *pucPtr++ = *(pucData+0);
					
					pucData += 4;
                }
                break;
            }
    }
}



//*****************************************************************************
//
//! Initializes a 24 BPP off-screen buffer.
//!
//! \param pDisplay is a pointer to the display structure to be configured for
//! the 24 BPP off-screen buffer.
//! \param pucImage is a pointer to the image buffer to be used for the
//! off-screen buffer.
//! \param lWidth is the width of the image buffer in pixels.
//! \param lHeight is the height of the image buffer in pixels.
//!
//! This function initializes a display structure, preparing it to draw into
//! the supplied image buffer.  The image buffer is assumed to be large enough
//! to hold an image of the specified geometry.
//!
//! \return None.
//
//*****************************************************************************
void
GrOffScreen24BPPInit(tDisplay *pDisplay, unsigned char *pucImage, int lWidth,
                    int lHeight)
{

    //
    // Initialize the display structure.
    //
    pDisplay->lSize = sizeof(tDisplay);
    pDisplay->pvDisplayData = pucImage;
    pDisplay->usWidth = lWidth;
    pDisplay->usHeight = lHeight;
    pDisplay->pfnPixelDraw = GrOffScreen24BPPPixelDraw;
    pDisplay->pfnPixelDrawMultiple = GrOffScreen24BPPPixelDrawMultiple;
    pDisplay->pfnLineDrawH = GrOffScreen24BPPLineDrawH;
    pDisplay->pfnLineDrawV = GrOffScreen24BPPLineDrawV;
    pDisplay->pfnRectFill = GrOffScreen24BPPRectFill;
    pDisplay->pfnColorTranslate = GrOffScreen24BPPColorTranslate;
    //pDisplay->pfnFlush = GrOffScreen24BPPFlush;

    /* IMAGE FORMAT */    
    *(unsigned short *)(pucImage + 0) = lWidth;
    *(unsigned short *)(pucImage + 2) = lHeight;
    
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
