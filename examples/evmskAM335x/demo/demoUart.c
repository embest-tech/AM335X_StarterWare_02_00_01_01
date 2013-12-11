/**
 * \file    demoUart.c
 *
 * \brief   This file contains Uart related functions.
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

#include "soc_AM335x.h"
#include "interrupt.h"
#include "evmAM335x.h"
#include "uart_irda_cir.h"
#include "demoUart.h"
#include "demoCfg.h"

/*******************************************************************************
**                     FUNCTION DEFINITIONS
*******************************************************************************/

/* Enable Uart wakeup */
void enableUartWakeup(void)
{
    /* enable rx interrupt */
    UARTIntEnable(SOC_UART_0_REGS,
                  ((UART_IER_RHR_IT_ENABLE << UART_IER_RHR_IT_SHIFT) |
                   (UART_IER_LINE_STS_IT_ENABLE < UART_IER_LINE_STS_IT_SHIFT)));

    /* Uart Wake up enable */
    UARTWakeUpControl(SOC_UART_0_REGS,
                      (UART_SYSC_ENAWAKEUP_ENABLE << UART_SYSC_ENAWAKEUP_SHIFT));
    /* Uart idle mode */
    UARTIdleModeConfigure(SOC_UART_0_REGS,
                          (UART_SYSC_IDLEMODE_WAKEUP << UART_SYSC_IDLEMODE_SHIFT));
}

/* Disable Uart wakeup */
void disableUartWakeup(void)
{
    /* disable rx interrupt */
    UARTIntDisable(SOC_UART_0_REGS,
                   ((UART_IER_RHR_IT_ENABLE << UART_IER_RHR_IT_SHIFT) |
                    (UART_IER_LINE_STS_IT_ENABLE < UART_IER_LINE_STS_IT_SHIFT)));

    /* Uart Wake up disable */
    UARTWakeUpControl(SOC_UART_0_REGS,
                      (UART_SYSC_ENAWAKEUP_DISABLE << UART_SYSC_ENAWAKEUP_SHIFT));

    /* Uart idle mode */
    UARTIdleModeConfigure(SOC_UART_0_REGS,
                          (UART_SYSC_IDLEMODE_SMART << UART_SYSC_IDLEMODE_SHIFT));
}
