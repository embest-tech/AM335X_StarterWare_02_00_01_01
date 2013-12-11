/*
 * board.js
 *
 * Board specific content used by the demo script.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
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
 */

/*
 * Define all icon objects
 */
var Icons = [
  {
    "id":  "intro",
    "txt": "INTRO",
    "img": "intro.png",
    //"act": "func:ShowPanel(0)",
	"act": "submit:INTRO",
    "tip": "Click to change the slide on target"
  }, 
  {
    "id":  "www",
    "txt": "WWW",
    "img": "www.png",
    "act": "submit:WWW",
    "tip": "Click to change the slide on target. The IP address will be acquired and displayed on the LCD"
  }, 
  {
    "id":  "mcasp",
    "txt": "MCASP",
    "img": "mcasp.png",
    "act": "submit:MCASP",
    "tip": "Click to change the slide on target. Insert Speaker to the line-out in target to listen to the audio played"
  }, 
  {
    "id":  "mmcsd",
    "txt": "MMCSD",
    "img": "mmcsd.png",
    "act": "submit:MMCSD",
    "tip":"Click to change the slide on target."
  },  
  {
    "id":  "uart",
    "txt": "UART",
    "img": "uart.png",
    "act": "submit:UART",
    "tip": "Click to change the slide on target."
  }, 
  {
    "id":  "rtc",
    "txt": "RTC",
    "img": "rtc.png",
    "act": "submit:RTC",
    "tip":"Click to change the slide on target. Press the icon on LCD to enter time and date."
  },   
  {
    "id":  "timer",
    "txt": "TIMER",
    "img": "timer.png",
    "act": "submit:TIMER",
    "tip":"Click to change the slide on target. Observe the time variation interval of color change on target."
  }, 
  {
    "id":  "eth",
    "txt": "ETHERNET",
    "img": "eth.png",
    "act": "submit:ETHERNET",
    "tip":"Click to change the slide on target."
  }, 
  {
    "id":  "ecap",
    "txt": "ECAP",
    "img": "ecap.png",
    "act": "submit:ECAP",
    "tip":"Click to change the slide on target. Press the icon on LCD to see change in brightness which is controlled with eCAP."
  },
  {
    "id":  "gpio",
    "txt": "GPIO",
    "img": "gpio.png",
    "act": "submit:GPIO",
    "tip":"Click to change the slide on target. Observe blinking of the LED D1."
  },
  {
    "id":  "i2c",
    "txt": "I2C",
    "img": "i2c.png",
    "act": "submit:I2C",
    "tip":"Click to change the slide on target. Accelerometer is demonstrated using I2C."
  },
  {
    "id":  "pm",
    "txt": "PM",
    "img": "pm.png",
    "act": "submit:PM",
    "tip":"Click to change the slide on target. Dleep Sleep 0 is demonstrated in this slide."
  },
  {
    "id":  "dvfs",
    "txt": "DVFS",
    "img": "dvfs.png",
    "act": "submit:DVFS",
    "tip":"Click to change the slide on target. Various OPP (MPU) are demonstrated in this slide."
  }      
];

/*
 * List of icons that are not currently supported on this board.
 * They will be filtered in the final UI.
 */
var Unused = new Array();
