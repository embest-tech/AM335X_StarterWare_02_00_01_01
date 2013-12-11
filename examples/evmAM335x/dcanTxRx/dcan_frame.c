/**
 * \file   dcan_frame.c
 *
 * \brief  This file consists of wrapper functions which internally call 
 *         DCAN APIs.
 */

/*
 * Copyright (C) 2005 Marc Kleine-Budde, Pengutronix
 * Copyright (C) 2006 Andrey Volkov, Varma Electronics
 * Copyright (C) 2008-2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Include the necessary header files */
#include "dcan_frame.h"
#include "hw_dcan.h"
#include "dcan.h"

#define DCAN_ID_MASK           (0x0u)

/**
 * \brief   This function will configure a message object in the DCAN message 
 *          RAM. This function will internally call 'CANTxObjectConfig' or 
 *          'CANRxObjectConfig' functions depending on the flags sent by user.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   canPtr        Pointer to can_frame structure.
 *
 **/
void CANMsgObjectConfig(unsigned int baseAdd, can_frame* canPtr)
{
    if((canPtr->flag & CAN_DATA_FRAME) && (canPtr->flag & CAN_MSG_DIR_TX))
    {
        /* Configure a transmit message object */
        CANTxObjectConfig(baseAdd, canPtr);
    }
    else if((canPtr->flag & CAN_DATA_FRAME) && (canPtr->flag & CAN_MSG_DIR_RX))
    {
        /* Configure a receive message object */
        CANRxObjectConfig(baseAdd, canPtr);
    }
    else
    {

    }
}

/**
 * \brief   This function will configure a message object in DCAN message RAM as a 
 *          transmit message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   canPtr        Pointer to can_frame structure.
 *
**/
void CANTxObjectConfig(unsigned int baseAdd, can_frame* canPtr)
{
    unsigned int msgNum;
    unsigned int idLen;

    idLen = (canPtr->flag & CAN_EXT_FRAME) ? DCAN_29_BIT_ID : DCAN_11_BIT_ID;
    
    /* Set the message valid bit */
    DCANMsgObjValidate(baseAdd, DCAN_IF1_REG);

    /* Set the message id of the frame to be transmitted */
    DCANMsgIdSet(baseAdd, canPtr->id, idLen, DCAN_IF1_REG);

    /* Set the message object direction as transmit */    
    DCANMsgDirectionSet(baseAdd, DCAN_TX_DIR, DCAN_IF1_REG);

    /* Set the data length code */
    DCANDataLengthCodeSet(baseAdd, canPtr->dlc, DCAN_IF1_REG);

    /* Write data to the DCAN data registers */
    DCANDataWrite(baseAdd, (canPtr->data), DCAN_IF1_REG);

    /* Enable the transmit interrupt for the message object */
    DCANMsgObjIntEnable(baseAdd, DCAN_TRANSMIT_INT, DCAN_IF1_REG);

    /* Enable the DCAN FIFO End of block */
    DCANFIFOEndOfBlockControl(baseAdd, DCAN_END_OF_BLOCK_ENABLE, DCAN_IF1_REG);   

    /* Get the transmit request status */
    msgNum = DCANTxRqstStatGet(baseAdd);

    /* Configure the command register */
    DCANCommandRegSet(baseAdd, (DCAN_DAT_A_ACCESS | DCAN_MSG_WRITE | DCAN_TXRQST_ACCESS | 
                                DCAN_DAT_B_ACCESS | DCAN_ACCESS_CTL_BITS | 
                                DCAN_ACCESS_ARB_BITS), msgNum, DCAN_IF1_REG);    
}

/**
 * \brief   This function will configure a message object in DCAN message RAM 
 *          as a receive message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   canPtr        Pointer to can_frame structure.
 *
**/
void CANRxObjectConfig(unsigned int baseAdd, can_frame* canPtr)
{
    unsigned int idLen;
    unsigned int msgIndex;

    msgIndex = (CAN_NUM_OF_MSG_OBJS / 2);

    idLen = (canPtr->flag & CAN_EXT_FRAME) ? DCAN_29_BIT_ID : DCAN_11_BIT_ID;

    /* Use Acceptance mask. */
    DCANUseAcceptanceMaskControl(baseAdd, DCAN_MASK_USED, DCAN_IF2_REG);

    /* Configure the DCAN mask registers for acceptance filtering. */
    DCANMsgObjectMskConfig(baseAdd, DCAN_IDENTIFIER_MSK(DCAN_ID_MASK,
                           DCAN_ID_MSK_11_BIT), DCAN_MSK_MSGDIR_DISABLE,
                           DCAN_MSK_EXT_ID_ENABLE, DCAN_IF2_REG);

    /* Set the message valid bit */
    DCANMsgObjValidate(baseAdd, DCAN_IF2_REG);    

    /* Set the message id of the frame to be received */
    DCANMsgIdSet(baseAdd, canPtr->id, idLen, DCAN_IF2_REG);

    /* Set the message object direction as receive */
    DCANMsgDirectionSet(baseAdd, DCAN_RX_DIR, DCAN_IF2_REG);

    /* Enable the receive interrupt for the message object */
    DCANMsgObjIntEnable(baseAdd, DCAN_RECEIVE_INT, DCAN_IF2_REG);

    /* Enable the FIFO end of block */
    DCANFIFOEndOfBlockControl(baseAdd, DCAN_END_OF_BLOCK_ENABLE, DCAN_IF2_REG);

    /* Check for the message valid status for receive objects */    
    while((DCANMsgValidStatusGet(baseAdd, msgIndex)) && 
          (msgIndex <= (CAN_NUM_OF_MSG_OBJS - 1)))
    {
        msgIndex++;    
    }

    /* Configure the command register */
    DCANCommandRegSet(baseAdd, (DCAN_ACCESS_CTL_BITS | DCAN_MSG_WRITE |
                      DCAN_ACCESS_MSK_BITS | DCAN_ACCESS_ARB_BITS), 
                      msgIndex, DCAN_IF2_REG);
}

/**
 * \brief   Read data from a message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   data          Pointer to an unsigned integer. Used to fetch data 
 *                        bytes from data registers.
 * \param   ifReg         Interface register set used.
 *
*/
void CANReadMsgObjData(unsigned int baseAdd, unsigned int msgNum, 
                       unsigned int* data, unsigned int ifReg)
{
    /* Read a message object from CAN message RAM to Interface register */
    DCANCommandRegSet(baseAdd, (DCAN_DAT_A_ACCESS | DCAN_DAT_B_ACCESS | 
                                DCAN_TXRQST_ACCESS | DCAN_CLR_INTPND | 
                                DCAN_ACCESS_CTL_BITS | DCAN_ACCESS_ARB_BITS | 
                                DCAN_ACCESS_MSK_BITS | DCAN_MSG_READ), 
                                msgNum, ifReg);

    /* Clear the NewData bit */
    DCANNewDataControl(baseAdd, DCAN_NEW_DAT_CLR, ifReg);

    /* Read data bytes from interface register */
    DCANDataRead(baseAdd, data, ifReg);
}

/**
 * \brief   This function should be used to clear the interrupt pending status 
 *          of receive objects after a new message is received. This will clear
 *          the IntPnd status of the message object represented by msgNum.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANClrIntPndStat(unsigned int baseAdd, unsigned int msgNum, 
                      unsigned int ifReg)
{
    /* Clear the IntPnd bit of DCAN_IFMCTL register */
    DCANClrIntPnd(baseAdd, ifReg);

    /* Set the ClrIntPnd bit of DCAN_IFCMD register */
    DCANCommandRegSet(baseAdd, DCAN_CLR_INTPND, msgNum, ifReg);
}


/**
 * \brief   This function can be used by the user to validate a message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANValidateMsgObject(unsigned int baseAdd, unsigned int msgNum, 
                          unsigned int ifReg)
{
    /* Set the MsgVal bit of DCAN_IFARB register */
    DCANMsgObjValidate(baseAdd, ifReg);

    /* Set the Arb bit of DCAN_IFCMD register */
    DCANCommandRegSet(baseAdd, (DCAN_ACCESS_ARB_BITS | DCAN_MSG_WRITE), 
                      msgNum, ifReg);
}    

/**
 * \brief   This function can be used by the user to invalidate a message 
 *          object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANInValidateMsgObject(unsigned int baseAdd, unsigned int msgNum, 
                            unsigned int ifReg)
{
    /* Clear the MsgVal bit of DCAN_IFARB register */
    DCANMsgObjInvalidate(baseAdd, ifReg);

    /* Set the Arb bit of DCAN_IFCMD register */
    DCANCommandRegSet(baseAdd, (DCAN_ACCESS_ARB_BITS | DCAN_MSG_WRITE), 
                      msgNum, ifReg);
}

/**
 * \brief   This function can be used to disable the transmit interrupt of a 
 *          message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANTxIntDisable(unsigned int baseAdd, unsigned int msgNum, 
                     unsigned int ifReg)
{
    /* Disable the message object transmit interrupt */
    DCANMsgObjIntDisable(baseAdd, DCAN_TRANSMIT_INT, ifReg);

    /* Set the CTL bit of the command register */
    DCANCommandRegSet(baseAdd, (DCAN_ACCESS_CTL_BITS | DCAN_MSG_WRITE), 
                      msgNum, ifReg);    
}

/**
 * \brief   This function can be used to disable the receive interrupt of a 
 *          message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANRxIntDisable(unsigned int baseAdd, unsigned int msgNum,
                     unsigned int ifReg)
{
    /* Disable the message object receive interrupt */
    DCANMsgObjIntDisable(baseAdd, DCAN_RECEIVE_INT, ifReg);

    /* Set the CTL bit of the command register */
    DCANCommandRegSet(baseAdd, (DCAN_ACCESS_CTL_BITS | DCAN_MSG_WRITE), 
                      msgNum, ifReg);
}

/**
 * \brief   This function can be used to update the data bytes of a transmit 
 *          object and set TxRqst for this message object.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   dataPtr       Pointer to unsigned integer. Used to write data 
 *                        bytes to data registers.
 * \param   msgNum        Message object number.
 * \param   ifReg         Interface register set used.
 *
**/
void CANUpdateDataBytes(unsigned int baseAdd, unsigned int* dataPtr, 
                        unsigned int msgNum, unsigned int ifReg)
{
    /* Populate the data bytes in the data registers */
    DCANDataWrite(baseAdd, dataPtr, ifReg);

    /* Set the DataA, DataB, TxRqst and WR of the IF_CMD register */
    DCANCommandRegSet(baseAdd, (DCAN_DAT_A_ACCESS | DCAN_DAT_B_ACCESS | 
                                DCAN_TXRQST_ACCESS | DCAN_MSG_WRITE), 
                                msgNum, ifReg);
}

/**
 * \brief   This function takes I/P Clk frequency, required bit-rate on the 
 *          CAN bus and calculates the value to be programmed to BTR register
 *          and sends the BTR value to 'DCANBitTimingConfig' API.
 *
 * \param   baseAdd       Base Address of the DCAN Module Registers.
 * \param   clkFreq       I/P clock frequency to DCAN controller.
 * \param   bitRate       Required bit-rate on the CAN bus.
 *
 * \return  Returns the error value if error is present.
 *
**/
unsigned int CANSetBitTiming(unsigned int baseAdd, unsigned int clkFreq,
                             unsigned int bitRate)
{
    unsigned int errVal = 0, btrValue = 0, tSeg1 = 0, tSeg2 = 0;
    struct _dcan_bittiming bit_time_values;
    struct _dcan_hw_params *btc;
    struct _dcan_bittiming *bt;

    static struct _dcan_hw_params dcan_hw_params = {
    /* tseg1Min = */ 1,
    /* tseg1Max = */ 16,
    /* tseg2Min = */ 1,
    /* tseg2Max = */ 8,
    /* sjwMax   = */ 4,
    /* brpMin   = */ 1,
    /* brpMax   = */ 1024,
    /* brpInc   = */ 1,
    };

    bt = &bit_time_values;
    btc = &dcan_hw_params;

    bt->bitRate = bitRate;

    errVal = CANbitTimeCalculator(btc, bt, clkFreq);

    /* Calculate Time Segment2 value */
    tSeg2 = (bt->phaseSeg2 - 1);

    /* Calculate Time Segment1 value */
    tSeg1 = (bt->phaseSeg1 + bt->propSeg - 1);

    /* Write the BRP value */
    btrValue |= ((bt->brp - 1) & DCAN_BTR_BRP);

    /* Write the BRPE value */
    btrValue |= (((bt->brp - 1) & EXTRACT_BRPE_VAL) << BRPE_SHIFT);

    /* Write the Time Segment2 value */
    btrValue |= ((tSeg2 << DCAN_BTR_TSEG2_SHIFT) & DCAN_BTR_TSEG2);

    /* Write the Time Segment1 value */
    btrValue |= ((tSeg1 << DCAN_BTR_TSEG1_SHIFT) & DCAN_BTR_TSEG1);

    /* Set the BTR value to the DCAN bittiming register */
    DCANBitTimingConfig(baseAdd, btrValue);

    return errVal;
}

/**
 * \brief   This function will calculate the bit-timing parameters for the 
 *          DCAN controller based on the input frequency and required bit-rate 
 *          on the CAN bus.
 *
 * \param   btc                 Pointer to _dcan_hw_params structure \n
 * \param   bt                  Pointer to _dcan_bittiming structure \n
 * \param   clkFreq             Clock frequency to DCAN controller in MHz \n
 *
 * \return  'errVal' based on the bit-rate error \n
 *
 **/
unsigned int CANbitTimeCalculator(struct _dcan_hw_params *btc,
                               struct _dcan_bittiming *bt,
                               unsigned int clkFreq)
{
    int sampl_pt, spt_error = 1000, tsegall, tseg = 0, tseg1 = 0, tseg2 = 0;
    int brp = 0, spt = 0, best_tseg = 0, best_brp = 0;
    long error = 0, best_error = 1000000000;
    unsigned int errVal = NO_BIT_RATE_ERR;
    unsigned long rate, timeQuanta;

    if(bt->bitRate > 800000)
    {
        sampl_pt = 750;
    }

    else if(bt->bitRate > 500000)
    {
        sampl_pt = 800;
    }

    else
    {
        sampl_pt = 875;
    }

    for(tseg = (btc->tseg1Max + btc->tseg2Max) * 2 + 1;
        tseg >= (btc->tseg1Min + btc->tseg2Min) * 2; tseg--)
    {
        tsegall = 1 + tseg / 2;
        /* Compute all possible tseg choices (tseg=tseg1+tseg2) */
        brp = clkFreq / (tsegall * bt->bitRate) + tseg % 2;
        /* chose brp step which is possible in system */
        brp = (brp / btc->brpInc) * btc->brpInc;
        if((brp < btc->brpMin) || (brp > btc->brpMax))
            continue;
        rate = clkFreq / (brp * tsegall);
        error = bt->bitRate - rate;
        /* tseg brp biterror */
        if(error < 0)
            error = -error;
        if(error > best_error)
            continue;
        best_error = error;
        if(error == 0)
        {
            spt = canUpdatSamPt(btc, sampl_pt, tseg / 2,
                                 &tseg1, &tseg2);
            error = sampl_pt - spt;
            if(error < 0)
                error = -error;
            if(error > spt_error)
                continue;
            spt_error = error;
        }
        best_tseg = tseg / 2;
        best_brp = brp;
        if(error == 0)
            break;
    }

    if(best_error)
    {
        /* Error in one-tenth of a percent */
        error = (best_error * 1000) / bt->bitRate;
        if(error > CAN_CALC_MAX_ERROR)
        {
            errVal = BIT_RATE_ERR_MAX;
        }
        else 
        {
            errVal = BIT_RATE_ERR_WARN;
        }
    }

    /* real sample point */
    bt->samplePnt = canUpdatSamPt(btc, sampl_pt, best_tseg,
                                      &tseg1, &tseg2);

    timeQuanta = best_brp * 1000000000UL;

    bt->tq = timeQuanta;
    bt->propSeg = tseg1 / 2;
    bt->phaseSeg1 = tseg1 - bt->propSeg;
    bt->phaseSeg2 = tseg2;
    bt->sjw = 1;
    bt->brp = best_brp;
    /* real bit-rate */
    bt->bitRate = clkFreq / (bt->brp * (tseg1 + tseg2 + 1));

    return errVal;
}

/**
 * \brief   This function will update the sampling point based on time 
 *          segment values \n
 *
 * \return  Updated sample point value \n
 *
 **/
int canUpdatSamPt(const struct _dcan_hw_params *ptr,
                  int sampl_pt, int tseg, int *tseg1, int *tseg2)
{
    *tseg2 = tseg + 1 - (sampl_pt * (tseg + 1)) / 1000;

    if(*tseg2 < ptr->tseg2Min)
    {
        *tseg2 = ptr->tseg2Min;
    }

    if(*tseg2 > ptr->tseg2Max)
    {
        *tseg2 = ptr->tseg2Max;
    }

    *tseg1 = tseg - *tseg2;

    if (*tseg1 > ptr->tseg1Max)
    {
        *tseg1 = ptr->tseg1Max;
        *tseg2 = tseg - *tseg1;
    }

    return(1000 * (tseg + 1 - *tseg2) / (tseg + 1));
}


