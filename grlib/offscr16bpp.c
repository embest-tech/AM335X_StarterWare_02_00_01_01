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
//! Merges red, green and blue components in to 16bpp(5-6-5) format
//!
//! \param red - red component value
//! \param green - green component value
//! \param blue - blue component value
//!
//! This function merges red, green and blue components in to 16bpp(5-6-5) format.
//!
//! \return Returns the RGB in 5-6-5 format.
//
//*****************************************************************************
unsigned int RGB16(unsigned char red, unsigned char green, unsigned char blue)
{
	return (unsigned int)(((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3));
}

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
GrOffScreen16BPPColorTranslate(void *pvDisplayData, unsigned int ulValue)
{
	return RGB16((ulValue >> ClrRedShift) & 0xff, (ulValue >> ClrGreenShift) & 0xff,
												(ulValue >> ClrBlueShift) & 0xff);
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
GrOffScreen16BPPRectFill(void *pvDisplayData, const tRectangle *pRect,
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
    lBytesPerRow = (*(unsigned short *)(pucData)) * 2;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += ((lBytesPerRow * pRect->sYMin) + (pRect->sXMin * 2) + 4 + (16*2));

    //
    // Copy the pixel value into 2 pixels of the unsigned int.  This will
    // be used later to write multiple pixels into memory (as opposed to one at
    // a time).
    //
    ulValue = (ulValue << 16) | ulValue;

    //
    // Get the starting X coordinate of the rectangle.
    //
    lX = pRect->sXMin;

    //
    // See if the buffer pointer is not word aligned and there are at least
    // one pixel columns left to draw.
    //
    if(((unsigned int)pucData & 2) && ((pRect->sXMax - lX) > 0))
    {
        //
        // Draw one pixels in this column of the rectangle.
        //
        for(lY = pRect->sYMin, pucColumn = pucData; lY <= pRect->sYMax;
            lY++, pucColumn += lBytesPerRow)
        {
            *(unsigned short *)pucColumn = ulValue & 0xffff;
        }
        pucData += 2;
        lX +=1;
    }

    //
    // Loop while there are at least 2 pixel columns left to draw.
    //
    while((lX+1) <= pRect->sXMax)
    {
        //
        // Draw eight pixels in this column of the rectangle.
        //
        for(lY = pRect->sYMin, pucColumn = pucData; lY <= pRect->sYMax;
            lY++, pucColumn += lBytesPerRow)
        {
            *(unsigned int *)pucColumn = ulValue;
        }
		pucData += 4;
        lX += 2;
    }

    //
    // See if there are at least one pixel columns left to draw.
    //
    if(lX <= pRect->sXMax)
    {
        //
        // Draw four pixel columns, leaving the buffer pointer half-word
        // aligned.
        //
        for(lY = pRect->sYMin, pucColumn = pucData; lY <= pRect->sYMax;
            lY++, pucColumn += lBytesPerRow)
        {
            *(unsigned short *)pucColumn = ulValue & 0xffff;;
        }
        pucData += 2;
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
GrOffScreen16BPPLineDrawH(void *pvDisplayData, int lX1, int lX2, int lY,
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
	lBytesPerRow = (*(unsigned short *)(pucData)) * 2;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += ((lBytesPerRow * lY) + (lX1 * 2) + 4 + (16*2));
    //pucData += (*(unsigned short *)(pucData + 1) * lY * 2) + (lX1 * 2) + 4 + (16 * 2);

    //
    // Copy the pixel value into 2 pixels of the unsigned int.  This will
    // be used later to write multiple pixels into memory (as opposed to one at
    // a time).
    //
    ulValue = (ulValue << 16) | ulValue;


    //
    // See if the buffer pointer is not word aligned and there are at least one
    // pixels left to draw.
    //
    if(((unsigned int)pucData & 2) && ((lX2 - lX1) > 0))
    {
        //
        // Draw two pixels to word align the buffer pointer.
        //
        *(unsigned short *)pucData = ulValue & 0xffff;
        pucData += 2;
        lX1 += 1;
    }

    //
    // Loop while there are at least 2 pixels left to draw.
    //
    while((lX1 + 1) <= lX2)
    {
        //
        // Draw 2 pixels.
        //
        *(unsigned int *)pucData = ulValue;
        pucData += 4;
        lX1 += 2;
    }

    //
    // See if there are at least 1 pixels left to draw.
    //
    if(lX1 <= lX2)
    {
        //
        // Draw 1 pixels, leaving the buffer pointer half-word aligned.
        //
        *(unsigned short *)pucData = ulValue & 0xffff;
        pucData += 2;
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
GrOffScreen16BPPLineDrawV(void *pvDisplayData, int lX, int lY1, int lY2,
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
	lBytesPerRow = (*(unsigned short *)(pucData)) * 2;	

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += ((lBytesPerRow * lY1) + (lX * 2) + 4 + (16*2));

    //
    // Loop over the rows of the line.
    //
    for(; lY1 <= lY2; lY1++)
    {
        *(unsigned short *)pucData = ulValue;
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
GrOffScreen16BPPPixelDraw(void *pvDisplayData, int lX, int lY,
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
	lBytesPerRow = (*(unsigned short *)(pucData)) * 2;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucData += (lBytesPerRow * lY) + (lX * 2) + 4 + (16*2);
    //pucData += (*(unsigned short *)(pucData + 1) * lY * 2) + (lX * 2) + 4 + (16*2);

    //
    // Write this pixel into the image buffer.
    //
    *(unsigned short *)pucData = ulValue & 0xFFFF;
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
//! \param lBPP is the number of bits per pixel; must be 1, 4, or 8.
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
GrOffScreen16BPPPixelDrawMultiple(void *pvDisplayData, int lX, int lY,
                                 int lX0, int lCount, int lBPP,
                                 const unsigned char *pucData,
                                 const unsigned char *pucPalette)
{
    unsigned char *pucPtr;
	unsigned int ulByte;
    int lBytesPerRow;
	
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
    lBytesPerRow = (*(unsigned short *)(pucPtr)) * 2;

    //
    // Get the offset to the byte of the image buffer that contains the
    // starting pixel.
    //
    pucPtr += (lBytesPerRow * lY) + (lX * 2) + 4 + (16*2);

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
							ulByte = (*(unsigned int *)(pucPalette + ulByte) &
									0x00ffffff);
						#else
							offset = (*pucData >> 4) * 3;
							ulByte  = (*(unsigned char *)(pucPalette + offset));
							ulByte |= (*((unsigned char *)(pucPalette + offset) + 1)) << 8;
							ulByte |= *((unsigned char *)(pucPalette + offset) + 2) << 16;
						#endif

                        //
                        // Translate this palette entry.
                        //
                        ulByte = GrOffScreen16BPPColorTranslate(pvDisplayData,
                                                               ulByte);

                        //
                        // Write this pixel to the screen.
                        //
						*pucPtr++ = (ulByte & 0x00FF);
						*pucPtr++ = (ulByte & 0xFF00) >> 8;
						
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
								ulByte = (*(unsigned int *)(pucPalette + ulByte) &
										0x00ffffff);
							#else
								offset = (*pucData++ & 15) * 3;
								ulByte  = (*(unsigned char *)(pucPalette + offset));
								ulByte |= (*((unsigned char *)(pucPalette + offset) + 1)) << 8;
								ulByte |= *((unsigned char *)(pucPalette + offset) + 2) << 16;
							#endif

							//
							// Translate this palette entry.
							//
							ulByte = GrOffScreen16BPPColorTranslate(pvDisplayData,
																   ulByte);

							//
							// Write this pixel to the screen.
							//
							*pucPtr++ = (ulByte & 0x00FF);
							*pucPtr++ = (ulByte & 0xFF00) >> 8;							

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
                ulByte = GrOffScreen16BPPColorTranslate(pvDisplayData, ulByte);

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
				pucData++;
			    while(lCount--)
		    	{
			    	*(pucPtr+1) = *pucData++; // 2 bytes
					*(pucPtr) = *pucData++;
					pucPtr += 2;
		    	}
				break;
			}
    }
}



//*****************************************************************************
//
//! Initializes a 16 BPP off-screen buffer.
//!
//! \param pDisplay is a pointer to the display structure to be configured for
//! the 16 BPP off-screen buffer.
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
GrOffScreen16BPPInit(tDisplay *pDisplay, unsigned char *pucImage, int lWidth,
                    int lHeight)
{

    //
    // Initialize the display structure.
    //
    pDisplay->lSize = sizeof(tDisplay);
    pDisplay->pvDisplayData = pucImage;
    pDisplay->usWidth = lWidth;
    pDisplay->usHeight = lHeight;
    pDisplay->pfnPixelDraw = GrOffScreen16BPPPixelDraw;
    pDisplay->pfnPixelDrawMultiple = GrOffScreen16BPPPixelDrawMultiple;
    pDisplay->pfnLineDrawH = GrOffScreen16BPPLineDrawH;
    pDisplay->pfnLineDrawV = GrOffScreen16BPPLineDrawV;
    pDisplay->pfnRectFill = GrOffScreen16BPPRectFill;
    pDisplay->pfnColorTranslate = GrOffScreen16BPPColorTranslate;
    //pDisplay->pfnFlush = GrOffScreen16BPPFlush;
	
	*(unsigned short *)(pucImage + 0) = lWidth;
    *(unsigned short *)(pucImage + 2) = lHeight;
	
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
