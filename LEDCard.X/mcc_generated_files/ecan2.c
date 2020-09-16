/**
  ECAN2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ecan2.c

  @Summary
    This is the generated driver implementation file for the ECAN2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for ECAN2.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.166.0
        Device            :  dsPIC33EP512GM604
        Driver Version    :  1.00
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.41
        MPLAB 	          :  MPLAB X v5.30
*/

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "ecan2.h"
#include "can2.h"
#include "dma.h"

/**
  Section: ECAN2 APIs
*****************************************************************************************/

void ECAN2_Initialize(void)
{

    CAN2_Initialize();
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_TransmitEnable
*    Description:       Setup the DMA for Transmit from the CAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/

void ECAN2_TransmitEnable()
{
    CAN2_TransmitEnable();
}
/******************************************************************************
*                                                                             
*    Function:		ECAN2_ReceiveEnable
*    Description:       Setup the DMA for Receive on the CAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/

void ECAN2_ReceiveEnable()
{
    CAN2_ReceiveEnable();
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_transmit
*    Description:       Transmits the message from user buffer to CAN buffer
*                       as per the buffer number allocated.
*                       Allocation of the buffer number is done by user 
*                                                                             
*    Arguments:		priority : priority of the message to be transmitted
*                       sendCanMsg: pointer to the message object
*                                             
*    Return Value:      true - Transmit successful
*                       false - Transmit failure                                                                              
******************************************************************************/

bool ECAN2_transmit(ECAN2_TX_PRIOIRTY priority, uCAN2_MSG *sendCanMsg) 
{
    return CAN2_transmit((CAN_TX_PRIOIRTY) priority, (uCAN_MSG *) sendCanMsg);
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_receive
*    Description:       Receives the message from CAN buffer to user buffer 
*                                                                             
*    Arguments:		recCanMsg: pointer to the message object
*                                             
*    Return Value:      true - Receive successful
*                       false - Receive failure                                                                              
******************************************************************************/

bool ECAN2_receive(uCAN2_MSG *recCanMsg) 
{           
    return CAN2_receive((uCAN_MSG *)recCanMsg) ;
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_isBusOff
*    Description:       Checks whether the transmitter in Bus off state
*                                                                             
                                             
*    Return Value:      true - Transmitter in Bus Off state
*                       false - Transmitter not in Bus Off state                                                                              
******************************************************************************/

bool ECAN2_isBusOff() 
{
    return CAN2_isBusOff();	
}
/******************************************************************************
*                                                                             
*    Function:		ECAN2_isRXErrorPassive
*    Description:       Checks whether the receive in error passive state
*                                             
*    Return Value:      true - Receiver in Error Passive state
*                       false - Receiver not in Error Passive state                                                                              
******************************************************************************/

bool ECAN2_isRXErrorPassive()
{
    return CAN2_isRXErrorPassive();   
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_isTXErrorPassive
*    Description:       Checks whether the transmitter in error passive state                                                                          
*                                             
*    Return Value:      true - Transmitter in Error Passive state
*                       false - Transmitter not in Error Passive state                                                                              
******************************************************************************/

bool ECAN2_isTXErrorPassive()
{
    return CAN2_isTXErrorPassive();
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_messagesInBuffer
*    Description:       returns the number of messages that are received                                                                           
*                                             
*    Return Value:      Number of message received
******************************************************************************/

uint8_t ECAN2_messagesInBuffer() 
{
    return CAN2_messagesInBuffer();
}

/******************************************************************************
*                                                                             
*    Function:		ECAN2_sleep
*    Description:       Puts ECAN2 module in disable mode.
*                                                                       
******************************************************************************/

void ECAN2_sleep(void) 
{
    CAN2_sleep(); 
}

/**
 End of File
*/