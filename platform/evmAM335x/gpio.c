/**
 * \file   gpio.c
 *
 * \brief  This file contains functions which performs the platform specific
 *         configurations of GPIO.
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


#include "hw_control_AM335x.h"
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "evmAM335x.h"
#include "hw_cm_per.h"
#include "hw_types.h"

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO pin GPIO0[6]
 *         for use. GPIO0[6] means 6th pin of GPIO0 instance.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 *
 */

unsigned int GPIO0Pin6PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 0, 1, 2, 3, 4, and 6. */
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 6:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_CS1) =
                (CONTROL_CONF_SPI0_CS1_CONF_SPI0_CS1_RXACTIVE |
                 CONTROL_CONF_SPI0_CS1_CONF_SPI0_CS1_PUTYPESEL |
                 CONTROL_CONF_MUXMODE(7));
            status = TRUE;
        break;

        /* Fall through for case 5. */
        case 5:
        default:
        break; 
    }
    
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO pin
 *         GPIO1[30] for use. GPIO1[30] means 30th pin of GPIO1 instance.
 *         This pin can be used to control the Audio Buzzer.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 *
 * \note   Either of GPIO1[23] or GPIO1[30] pins could be used to control the
 *         Audio Buzzer.
 */

unsigned int GPIO1Pin30PinMuxSetup(void)
{
    unsigned int profile = 1;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 1, 4 and 6. */
        case 1:
        case 4:
        case 6:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_CSN(1)) =
                (CONTROL_CONF_GPMC_CSN0_CONF_GPMC_CSN0_PUTYPESEL |
                 CONTROL_CONF_MUXMODE(7));
            status = TRUE;

        break;

        /* Fall through for cases 0, 2, 3 and 5. */
        case 0:
        case 2:
        case 3:
        case 5:
        default:
        break;
    }
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO pin
 *         GPIO1[23] for use. GPIO1[23] means 23rd pin of GPIO1 instance.
 *         This pin can be used to control the Audio Buzzer.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 *
 */

unsigned int GPIO1Pin23PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet();

    switch(profile)
    {
        /* Fall through for cases 0,1,2,4,5 and 6.  */
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_A(7)) =
                CONTROL_CONF_MUXMODE(7);
            status = TRUE;
        break;

        case 3:
        default:
        break;
    }

    return status;
}

/**
 * \brief  This function does the Pin Multiplexing for the GPIO pin GPIO1[16]
 *         i.e. 16th pin of GPIO1 instance and selects it for use. This pin is
 *         used to generate Sensor Interrupts to the CPU.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 *
 * \note   Either GPIO1[16] or GPIO1[28] could be used to generate Sensor
 *         interrupts to the CPU.
 */

unsigned int GPIO1Pin16PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 0 and 3. */
        case 0:
        case 3:
        case 5:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_A(0)) =
                (CONTROL_CONF_GPMC_A0_CONF_GPMC_A0_RXACTIVE |
                 CONTROL_CONF_MUXMODE(7));
            status = TRUE;
        break;

        /* Fall through for cases 1, 2, 4, 6 and 7. */
        case 1:
        case 2:
        case 4:
        case 6:
        case 7:
        default:
        break;
    }
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO1[28]
 *         i.e. 28th pin of GPIO1 instance for use. This pin is used to
 *         generate Sensor interrupts to the CPU.
 *
 * \patam  None
 *
 * \return TRUE/FALSE
 *
 * \note   Either GPIO1[16] or GPIO1[28] could be used to generate Sensor
 *         interrupts to the CPU.
 */

unsigned int GPIO1Pin28PinMuxSetup(void)
{
    unsigned int profile = 1;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 1, 2, 4 and 6. */
        case 1:
        case 2:
        case 4:
        case 6:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_BE1N) =
                (CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_RXACTIVE |
                 CONTROL_CONF_GPMC_BE1N_CONF_GPMC_BE1N_PUTYPESEL |
                  CONTROL_CONF_MUXMODE(7));
            status = TRUE;
        break;

        /* Fall through for cases 0, 3, and 5. */
        case 0:
        case 3:
        case 5:
        default:
        break;
    }
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing for the GPIO pin GPIO0[7]
 *         i.e. 7th pin of GPIO0 instance and selects it for use.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 *
 */

unsigned int GPIO0Pin7PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 0, 1, and 2. */
        case 0:
        case 1:
        case 2:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_ECAP0_IN_PWM0_OUT) =	
                (CONTROL_CONF_ECAP0_IN_PWM0_OUT_CONF_ECAP0_IN_PWM0_OUT_RXACTIVE |
                 CONTROL_CONF_MUXMODE(7));
            status = TRUE;
        break;

        /* Fall through for cases 3, 4, 5 and 6. */
        case 3:
        case 4:
        case 5:
        case 6:
        default:
        break;
    }
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO0[19]
 *         for use. By GPIO0[19], we mean 19th pin of GPIO0 instance.
 *         This GPIO Pin will be used to control an LED present on the
 *         AM335x baseboard.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 */

unsigned int GPIO0Pin19PinMuxSetup(void)
{
    unsigned int profile = 7;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 

    switch(profile)
    {
        /* Fall through for cases 0, 1, 2, 4, 5 and 6. */
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        default:
        break;

        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_XDMA_EVENT_INTR(0)) =
                CONTROL_CONF_MUXMODE(7);
            status = TRUE;
        break;
    }
    return status;
}

/**
 * \brief  This function does the Pin Multiplexing and selects GPIO0[2]
 *         for use. By GPIO0[2], we mean 2nd pin of GPIO0 instance.
 *
 * \param  None
 *
 * \return TRUE/FALSE
 */

unsigned int GPIO0Pin2PinMuxSetup(void)
{
    unsigned int profile = 7;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 
	
    switch(profile)
    {
        case 0:
		case 3:
				HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_SCLK) = 
					((CONTROL_CONF_SPI0_SCLK_CONF_SPI0_SCLK_RXACTIVE |
					 CONTROL_CONF_SPI0_SCLK_CONF_SPI0_SCLK_PUDEN |
					  CONTROL_CONF_MUXMODE(7)) & (~CONTROL_CONF_SPI0_SCLK_CONF_SPI0_SCLK_PUTYPESEL));
				status = TRUE;
				break;
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:		
        default:
        break;
    }
    return status;
}

unsigned int GPIO1Pin2PinMuxSetup(void)
{
    unsigned int profile = 7;
    unsigned int status = FALSE;

    profile = EVMProfileGet(); 
	
    switch(profile)
    {
        case 0:
		case 3:
				//HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_D0) = 
				HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(2)) = 
					((CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_RXACTIVE |
					 CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN |
					  CONTROL_CONF_MUXMODE(7)) & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL));
				status = TRUE;
				break;
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:		
        default:
        break;
    }
    return status;
}

/**
 * \brief   This function does the Pin Multiplexing and selects GPIO1[20]
 *          i.e. 20th Pin of GPIO1 instance for use. This pin can be used
 *          to control the rotation of a Haptics Motor present on AM335x EVM.
 *
 * \param   None
 *
 * \return  TRUE - if the intended Pin Multiplexing was done successfully.\n
 *          FALSE - if the intended Pin Multiplexing was not done due to
 *                  incorrect settings of the Profile switch on the AM335x EVM.
 *
 * \note    Either GPIO1[20] or GPIO2[24] could be used to control the
 *          rotation of the Haptics Motor.
 */

unsigned int GPIO1Pin20PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet();

    switch(profile)
    {
        /* Fall through for cases 0, 1, 2, 4, 5 and 6. */
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_A(4)) =
                CONTROL_CONF_MUXMODE(7);
            status = TRUE;
        break;

        case 3:
        default:
        break;
    }

    return status;
}

/**
 * \brief   This function does the Pin Multiplexing and selects GPIO2[24]
 *          i.e. 24th Pin of GPIO2 instance for use. This pin can be used
 *          to control the rotation of a Haptics Motor present on AM335x EVM.
 *
 * \param   None
 *
 * \return  TRUE - if the intended Pin Multiplexing was done successfully.\n
 *          FALSE - if the intended Pin Multiplexing was not done due to
 *                  incorrect settings of the Profile switch on the AM335x EVM.
 *
 * \note    Either GPIO2[24] or GPIO1[20] could be used to control the
 *          rotation of the Haptics Motor.
 */

unsigned int GPIO2Pin24PinMuxSetup(void)
{
    unsigned int profile = 0;
    unsigned int status = FALSE;

    profile = EVMProfileGet();

    switch(profile)
    {
        case 3:
            HWREG(SOC_CONTROL_REGS + CONTROL_CONF_LCD_PCLK) =
                CONTROL_CONF_MUXMODE(7);
            status = TRUE;
        break;

        /* Fall through for cases 0, 1, 2, 4, 5, 6 and 7. */
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
        case 7:
        default:
        break;
    }

    return status;
}


/*
** This function enables the L3 and L4_WKUP interface clocks.
** This also enables the functional clock for GPIO0 instance.
*/

void GPIO0ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_WKUP_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_GPIO0_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_MODULEMODE));

    /*
    ** Writing to OPTFCLKEN_GPIO0_GDBCLK field of CM_WKUP_GPIO0_CLKCTRL
    ** register.
    */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) |=
        CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK;

    /* Waiting for OPTFCLKEN_GPIO0_GDBCLK field to reflect the written value. */
    while(CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_OPTFCLKEN_GPIO0_GDBCLK));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));


    /* Writing to IDLEST field in CM_WKUP_GPIO0_CLKCTRL register. */
    while((CM_WKUP_GPIO0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_GPIO0_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_GPIO0_CLKCTRL) &
           CM_WKUP_GPIO0_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO0_GDBCLK field in CM_WKUP_GPIO0_CLKCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_GPIO0_GDBCLK));
}

/*
** This function enables the L3 and L4_PER interface clocks.
** This also enables functional clocks of GPIO1 instance.
*/

void GPIO1ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field in CM_PER_OCPWP_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_CLKCTRL) |=
          CM_PER_OCPWP_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_OCPWP_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_CLKCTRL) &
           CM_PER_OCPWP_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));


    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for STBYST bit in CM_PER_OCPWP_CLKCTRL register to attain
    ** the desired value.
    */
    /* while((CM_PER_OCPWP_CLKCTRL_STBYST_FUNC <<
           CM_PER_OCPWP_CLKCTRL_STBYST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_CLKCTRL) &
            CM_PER_OCPWP_CLKCTRL_STBYST)); */

    /*
    ** Waiting for IDLEST field in CM_PER_OCPWP_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_OCPWP_CLKCTRL_IDLEST_FUNC <<
           CM_PER_OCPWP_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_CLKCTRL) &
            CM_PER_OCPWP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));


    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));

    /* Configuring L4 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L4LS_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) |=
          CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) &
           CM_PER_L4LS_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L4LS_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) |=
          CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL));

    /* Verifying if other configurations are correct. */

    /*
    ** Waiting for IDLEST field in CM_PER_L4LS_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L4LS_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L4LS_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) &
           CM_PER_L4LS_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4LS_GCLK bit in CM_PER_L4LS_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L4_GCLK bit in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    /* while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK)); */

    /* Performing configurations for GPIO1 instance. */

    /* Writing to MODULEMODE field of CM_PER_GPIO1_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_MODULEMODE));
    /*
    ** Writing to OPTFCLKEN_GPIO_1_GDBCLK bit in CM_PER_GPIO1_CLKCTRL
    ** register.
    */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |=
          CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK;

    /*
    ** Waiting for OPTFCLKEN_GPIO_1_GDBCLK bit to reflect the desired
    ** value.
    */
    while(CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
           CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK));

    /*
    ** Waiting for IDLEST field in CM_PER_GPIO1_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_GPIO1_CLKCTRL_IDLEST_FUNC <<
           CM_PER_GPIO1_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) &
            CM_PER_GPIO1_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO_1_GDBCLK bit in CM_PER_L4LS_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK));
}


/*
** This function enables GPIO1 pins
*/
unsigned int GPIO1PinMuxSetup(unsigned int pinNo)
{
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_GPMC_AD(pinNo)) =
        (CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_SLEWCTRL |     /* Slew rate slow */
        CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_RXACTIVE |    /* Receiver enabled */
        (CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUDEN)) | /* PU_PD enabled */
        (CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL & (~CONTROL_CONF_GPMC_AD_CONF_GPMC_AD_PUTYPESEL)) | /* PD */
        (CONTROL_CONF_MUXMODE(7))    /* Select mode 7 */
        );
     return TRUE;
}

/**
 * \brief  This function does the pin multiplexing for any GPIO Pin.
 *
 * \param  offsetAddr   This is the offset address of the Pad Control Register
 *                      corresponding to the GPIO pin. These addresses are
 *                      offsets with respect to the base address of the
 *                      Control Module.
 * \param  padConfValue This is the value to be written to the Pad Control
 *                      register whose offset address is given by 'offsetAddr'.
 *
 * Note: The 'offsetAddr' and 'padConfValue' can be obtained from macros defined
 *       in the file 'include/armv7a/am335x/pin_mux.h'.\n
 *
 * \return  None.
 */
void GpioPinMuxSetup(unsigned int offsetAddr, unsigned int padConfValue)
{
    HWREG(SOC_CONTROL_REGS + offsetAddr) = (padConfValue);
}
