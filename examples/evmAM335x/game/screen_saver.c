/**
 * \file  screen_saver.c
 *
 * \brief A screen saver for the OSRAM OLED display.
 */

/* Copyright (c) 2006-2010 Texas Instruments Incorporated.  All rights reserved.
 *
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 6288 of the EK-LM3S2965 Firmware Package.
 * This file is updated to make it work for StarterWare. */

#include "hw_types.h"
#include "globals.h"
#include "random.h"
#include "raster.h"
#include "screen_saver.h"

#ifdef _TMS320C6X
extern void CleanDSPCache_LCD (void);
#endif

//*****************************************************************************
//
// Extern variable declaration. Button pressed.
//
//*****************************************************************************
extern unsigned char g_ucSwitches;

//*****************************************************************************
//
// Extern function declarations. Functions used to control LCD backlight.
//
//*****************************************************************************
extern int ledBlink(void);
extern void ConfigRasterDisplay(void);
extern void DisableBackLight();
extern void EnableBackLight();
extern void updateFrameBuffer(void);
extern void getInputs(void);
extern void TouchInit(void);

//*****************************************************************************
//
// These arrays contain the starting and ending points of the lines on the
// display; this history buffer lists the lines from oldest to youngest.
//
//*****************************************************************************
static short g_pucScreenLinesX1[30];
static short g_pucScreenLinesY1[30];
static short g_pucScreenLinesX2[30];
static short g_pucScreenLinesY2[30];

//*****************************************************************************
//
// These variables contain the direction and rate of movement of the endpoints
// of the lines being drawn on the display.
//
//*****************************************************************************
static short g_cScreenDeltaX1;
static short g_cScreenDeltaY1;
static short g_cScreenDeltaX2;
static short g_cScreenDeltaY2;

//*****************************************************************************
//
// Draws a line in the local frame buffer using Bresneham's line drawing
// algorithm.
//
//*****************************************************************************
static void
ScreenSaverLine(int lX1, int lY1, int lX2, int lY2, unsigned int ulLevel)
{
    int lError, lDeltaX, lDeltaY, lYStep;
    tBoolean bSteep;

    //
    // Determine if the line is steep.  A steep line has more motion in the Y
    // direction than the X direction.
    //
    if(((lY2 > lY1) ? (lY2 - lY1) : (lY1 - lY2)) >
       ((lX2 > lX1) ? (lX2 - lX1) : (lX1 - lX2)))
    {
        bSteep = true;
    }
    else
    {
        bSteep = false;
    }

    //
    // If the line is steep, then swap the X and Y coordinates.
    //
    if(bSteep)
    {
        lError = lX1;
        lX1 = lY1;
        lY1 = lError;
        lError = lX2;
        lX2 = lY2;
        lY2 = lError;
    }

    //
    // If the starting X coordinate is larger than the ending X coordinate,
    // tehn swap the start and end coordinates.
    //
    if(lX1 > lX2)
    {
        lError = lX1;
        lX1 = lX2;
        lX2 = lError;
        lError = lY1;
        lY1 = lY2;
        lY2 = lError;
    }

    //
    // Compute the difference between the start and end coordinates in each
    // axis.
    //
    lDeltaX = lX2 - lX1;
    lDeltaY = (lY2 > lY1) ? (lY2 - lY1) : (lY1 - lY2);

    //
    // Initialize the error term to negative half the X delta.
    //
    lError = -lDeltaX / 2;

    //
    // Determine the direction to step in the Y axis when required.
    //
    if(lY1 < lY2)
    {
        lYStep = 1;
    }
    else
    {
        lYStep = -1;
    }

    //
    // Loop through all the points along the X axis of the line.
    //
    for(; lX1 <= lX2; lX1++)
    {
        //
        // See if this is a steep line.
        //
        if(bSteep)
        {
            //
            // Plot this point of the line, swapping the X and Y coordinates.
            //
            if(lY1 & 1)
            {
                g_pucFrame[54 + (lX1 * GAME_W_b) + (lY1 / 2)] =
                    ((g_pucFrame[(lX1 * GAME_W_b) + (lY1 / 2)] & 0xf0) |
                     (ulLevel & 0xf));
            }
            else
            {
                g_pucFrame[54 + (lX1 * GAME_W_b) + (lY1 / 2)] =
                    ((g_pucFrame[54 + (lX1 * GAME_W_b) + (lY1 / 2)] & 0x0f) |
                     ((ulLevel & 0xf) << 4));
            }
        }
        else
        {
            //
            // Plot this point of the line, using the coordinates as is.
            //
            if(lX1 & 1)
            {
                g_pucFrame[54 + (lY1 * GAME_W_b) + (lX1 / 2)] =
                    ((g_pucFrame[54 + (lY1 * GAME_W_b) + (lX1 / 2)] & 0xf0) |
                     (ulLevel & 0xf));
            }
            else
            {
                g_pucFrame[54 + (lY1 * GAME_W_b) + (lX1 / 2)] =
                    ((g_pucFrame[54 + (lY1 * GAME_W_b) + (lX1 / 2)] & 0x0f) |
                     ((ulLevel & 0xf) << 4));
            }
        }

        //
        // Increment the error term by the Y delta.
        //
        lError += lDeltaY;

        //
        // See if the error term is now greater than zero.
        //
        if(lError > 0)
        {
            //
            // Take a step in the Y axis.
            //
            lY1 += lYStep;

            //
            // Decrement the error term by the X delta.
            //
            lError -= lDeltaX;
        }
    }
}

//*****************************************************************************
//
// A screen saver to avoid damage to the OLED display (it has similar
// characteristics to a CRT with respect to image burn-in).  This implements a
// Qix-style chasing line that bounces about the display.
//
//*****************************************************************************
void
ScreenSaver(void)
{
    unsigned int ulCount, ulLoop;
	char touched = 0;

    //
    // Clear out the line history so that any lines from a previous run of the
    // screen saver are not used again.
    //
    for(ulLoop = 0; ulLoop < 29; ulLoop++)
    {
        g_pucScreenLinesX1[ulLoop] = 0;
        g_pucScreenLinesY1[ulLoop] = 0;
        g_pucScreenLinesX2[ulLoop] = 0;
        g_pucScreenLinesY2[ulLoop] = 0;
    }

    //
    // Choose random starting points for the first line.
    //
    g_pucScreenLinesX1[29] = GAME_W / 4;
    g_pucScreenLinesY1[29] = GAME_H / 4;
    g_pucScreenLinesX2[29] = 3 * (GAME_W / 4);
    g_pucScreenLinesY2[29] = 3 * (GAME_H / 4);

    //
    // Choose a random direction for each endpoint.  Make sure that the
    // endpoint does not have a zero direction vector.
    //
    g_cScreenDeltaX1 = (RandomNumber() >> 29);
    if(g_cScreenDeltaX1 < 1)
        g_cScreenDeltaX1--;
    else
        g_cScreenDeltaX1++;
    g_cScreenDeltaY1 = (RandomNumber() >> 29);
    if(g_cScreenDeltaY1 < 1)
        g_cScreenDeltaY1--;
    else
        g_cScreenDeltaY1++;
    g_cScreenDeltaX2 = (RandomNumber() >> 29);
    if(g_cScreenDeltaX2 < 1)
        g_cScreenDeltaX2--;
    else
        g_cScreenDeltaX2++;
    g_cScreenDeltaY2 = (RandomNumber() >> 29);
    if(g_cScreenDeltaY2 < 1)
        g_cScreenDeltaY2--;
    else
        g_cScreenDeltaY2++;

    //
    // Loop through the number of updates to the graphical screen saver to be
    // done before the display is turned off.
    //
    for(ulCount = 0; ulCount < (2 * 60 * 30); ulCount++)
    {
        //
        // Wait until an update has been requested.
        //
		Delay(0x7FFF);

        // See if the button has been pressed.
		getInputs();
		if((g_ucSwitches & 0x0f) == 0x0A)
		{
			return;
		}

        //
        // Shift the lines down one entry in the history buffer.
        //
        for(ulLoop = 1; ulLoop < 30; ulLoop++)
        {
            g_pucScreenLinesX1[ulLoop - 1] = g_pucScreenLinesX1[ulLoop];
            g_pucScreenLinesY1[ulLoop - 1] = g_pucScreenLinesY1[ulLoop];
            g_pucScreenLinesX2[ulLoop - 1] = g_pucScreenLinesX2[ulLoop];
            g_pucScreenLinesY2[ulLoop - 1] = g_pucScreenLinesY2[ulLoop];
        }

        //
        // Update the starting X coordinate of the youngest line.  If the edge
        // of the display has been reached, then choose a new travel speed in
        // the opposite direction.
        //
        g_pucScreenLinesX1[29] = g_pucScreenLinesX1[28] + g_cScreenDeltaX1;
        if(g_pucScreenLinesX1[29] > GAME_W_midx)
        {
            g_pucScreenLinesX1[29] = GAME_W_midx;
            g_cScreenDeltaX1 = -((RandomNumber() & 0x3) + 1);
        }
        else if (g_pucScreenLinesX1[29] < 0)
        {
            g_pucScreenLinesX1[29] = 0;
            g_cScreenDeltaX1 = (RandomNumber() & 0x3) + 1;
        }

        //
        // Update the starting Y coordinate of the youngest line.  If the edge
        // of the display has been reached, then choose a new travel speed in
        // the opposite direction.
        //
        g_pucScreenLinesY1[29] = g_pucScreenLinesY1[28] + g_cScreenDeltaY1;
        if(g_pucScreenLinesY1[29] > GAME_H_midx)
        {
            g_pucScreenLinesY1[29] = GAME_H_midx;
            g_cScreenDeltaY1 = -((RandomNumber() & 0x3) + 1);
        }
        else if (g_pucScreenLinesY1[29] < 0)
        {
            g_pucScreenLinesY1[29] = 0;
            g_cScreenDeltaY1 = (RandomNumber() & 0x3) + 1;
        }

        //
        // Update the ending X coordinate of the youngest line.  If the edge of
        // the display has been reached, then choose a new travel speed in the
        // opposite direction.
        //
        g_pucScreenLinesX2[29] = g_pucScreenLinesX2[28] + g_cScreenDeltaX2;
        if(g_pucScreenLinesX2[29] > GAME_W_midx)
        {
            g_pucScreenLinesX2[29] = GAME_W_midx;
            g_cScreenDeltaX2 = -((RandomNumber() & 0x3) + 1);
        }
        else if (g_pucScreenLinesX2[29] < 0)
        {
            g_pucScreenLinesX2[29] = 0;
            g_cScreenDeltaX2 = (RandomNumber() & 0x3) + 1;
        }

        //
        // Update the ending Y coordinate of the youngest line.  If the edge of
        // the display has been reached, then choose a new travel speed in the
        // opposite direction.
        //
        g_pucScreenLinesY2[29] = g_pucScreenLinesY2[28] + g_cScreenDeltaY2;
        if(g_pucScreenLinesY2[29] > GAME_H_midx)
        {
            g_pucScreenLinesY2[29] = GAME_H_midx;
            g_cScreenDeltaY2 = -((RandomNumber() & 0x3) + 1);
        }
        else if (g_pucScreenLinesY2[29] < 0)
        {
            g_pucScreenLinesY2[29] = 0;
            g_cScreenDeltaY2 = (RandomNumber() & 0x3) + 1;
        }

        //
        // Clear the local frame buffer.
        //
        for(ulLoop = 54; ulLoop < GAME_W * GAME_H / 2; ulLoop += 4)
        {
            *((unsigned int *)(g_pucFrame + ulLoop)) = 0;
        }

        //
        // Loop through the lines in the history buffer.
        //
        for(ulLoop = 0; ulLoop < 30; ulLoop++)
        {
            //
            // Draw this line if it "exists".  If both end points are at 0,0
            // then the line is assumed to not exist (i.e. the line history at
            // the very start).  There is a tiny likelyhood that the two
            // endpoints will converge on 0,0 at the same time in which case
            // there will be a small anomaly (though it would be extremely
            // difficult to visually discern that it occurred).
            //
            if(g_pucScreenLinesX1[ulLoop] || g_pucScreenLinesY1[ulLoop] ||
               g_pucScreenLinesX2[ulLoop] || g_pucScreenLinesY2[ulLoop])
            {
                ScreenSaverLine(g_pucScreenLinesX1[ulLoop],
                                g_pucScreenLinesY1[ulLoop],
                                g_pucScreenLinesX2[ulLoop],
                                g_pucScreenLinesY2[ulLoop], (ulLoop / 2) + 1);
            }
        }

        //
        // Copy the local frame buffer to the display.
        //
		updateFrameBuffer();
#ifdef _TMS320C6X
		CleanDSPCache_LCD();
#endif
    }

    // Clear the display and turn it off.
	ConfigRasterDisplay();
	DisableBackLight();


	//
	// Toggle the LED until the touch screen is pressed.
	// Since both the Touch screen and LED are interfaced on the same I2C (i2c0), 
	// the slave address and the ISR routine has to be updated every cycle
	//
	while(1)
	{

		while(ulLoop--)
		{
			// ReadAxis(2, &pressure, &touched);
			if(touched)	{break;}	// break when TS is touched.
		}
		if(touched) {break;}
	}


    // Turn on the display.
	EnableBackLight();
}
