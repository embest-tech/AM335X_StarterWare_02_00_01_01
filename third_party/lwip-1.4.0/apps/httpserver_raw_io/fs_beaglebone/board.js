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
    "id":  "led",
    "txt": "LED",
    "img": "led.png",
    //"act": "func:ShowPanel(0)",
	"act": "submit:LED",
    "tip": "Click to Switch ON LED on the target."
  },
  {
    "id":  "rtc",
    "txt": "RTC",
    "img": "rtc.png",
    "act": "submit:RTC",
    "tip":"Click to enter time and date in serial terminal."
  }, 
  {
    "id":  "timer",
    "txt": "TIMER",
    "img": "timer.png",
    "act": "submit:TIMER",
    "tip":"Click to enable timer demo. The LED on target is toggled at different time intervals (which is configured through timer)"
  }, 
  {
    "id":  "mmcsd",
    "txt": "MMCSD",
    "img": "mmcsd.png",
    "act": "submit:MMCSD",
    "tip":"Click to enable MMCSD demo. fatfs console is displayed on serial terminal. Exit from console to move to other demo."
  },
  {
    "id":  "pm",
    "txt": "PM",
    "img": "pm.png",
    "act": "submit:PM",
    "tip":"Click to enable PM demo. Slecet the power mode and wakeup source on serial terminal. Exit from console to move to other demo."
  }
//  {
//    "id":  "uart",
//    "txt": "UART",
//    "img": "uart.png",
//    "act": "submit:UART",
//    "tip": "All the user input (for RTC and MMC/SD demos) are taken through uart."
//  }, 
//  {
//    "id":  "eth",
//    "txt": "ETHERNET",
//    "img": "eth.png",
//    "act": "submit:ETHERNET",
//    "tip":"This page is served from target through ethernet port."
//  }, 
];

/*
 * List of icons that are not currently supported on this board.
 * They will be filtered in the final UI.
 */
//var Unused = new Array("uart", "eth");
var Unused = new Array();
