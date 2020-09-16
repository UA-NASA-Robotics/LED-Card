/**
  CAN2 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    can2.h

  @Summary
    This is the generated header file for the CAN2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for CAN2.
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

#ifndef _CAN2_H
#define _CAN2_H

/**
  Section: Included Files
*/

#include "can_types.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
  Section: CAN2 Module APIs
*/

/**
  @Summary
    Initializes the CAN2_Initialize.

  @Description
    This routine initializes the CAN2_Initialize.
    This routine must be called before any other CAN2 routine is called.
    This routine should only be called once during system initialization.

  @Preconditions
    None

  @Param
    None

  @Returns
    None

  @Comment
    

 @Example
    <code>
    CAN2_Initialize();
    </code>
 */
void CAN2_Initialize(void);

/******************************************************************************
*                                                                             
*    Function:		CAN2_receive
*    Description:       Receives the message from CAN buffer to user buffer                                                                         
*    Arguments:		recCanMsg: pointer to the message object
*    Return Value:      true - Receive successful
*                       false - Receive failure                                                                              
******************************************************************************/
bool CAN2_receive(uCAN_MSG *recCanMsg);

/******************************************************************************
*                                                                             
*    Function:		CAN2_transmit
*    Description:       Transmits the message from user buffer to CAN buffer                                                                        
*    Arguments:		priority: priority of the message to be transmitted
*                       sendCanMsg: pointer to the message object                                        
*    Return Value:      true - Transmit successful
*                       false - Transmit failure                                                                              
******************************************************************************/
bool CAN2_transmit(CAN_TX_PRIOIRTY priority, 
                                    uCAN_MSG *sendCanMsg);

/******************************************************************************                                                                       
*    Function:          CAN2_isBusOff
*    Description:       Checks whether the transmitter in Bus off state                                                                                                   
*    Return Value:      true - Transmitter in Bus Off state
*                       false - Transmitter not in Bus Off state                                                                              
******************************************************************************/
bool CAN2_isBusOff();

/******************************************************************************                                                                 
*    Function:		CAN2_isRXErrorPassive
*    Description:       Checks whether the receive in error passive state                                    
*    Return Value:      true - Receiver in Error Passive state
*                       false - Receiver not in Error Passive state                                                                              
******************************************************************************/
bool CAN2_isRXErrorPassive();

/******************************************************************************                                                                      
*    Function:		CAN2_isTXErrorPassive
*    Description:       Checks whether the transmitter in error passive state                                                                                                             
*    Return Value:      true - Transmitter in Error Passive state
*                       false - Transmitter not in Error Passive state                                                                              
******************************************************************************/
bool CAN2_isTXErrorPassive();

/******************************************************************************                                                                    
*    Function:		CAN2_messagesInBuffer
*    Description:       returns the number of messages that are received                                                                                                               
*    Return Value:      Number of message received
******************************************************************************/
uint8_t CAN2_messagesInBuffer();

/******************************************************************************
*                                                                             
*    Function:		CAN2_sleep
*    Description:       Puts CAN2 module in disable mode.
*    Return Value:      None
*                                                                       
******************************************************************************/
void CAN2_sleep();

/******************************************************************************                                                                    
*    Function:		CAN2_TransmitEnable
*    Description:       Enables Transmit for CAN2                                                                                                                
*    Return Value:      None
******************************************************************************/
void CAN2_TransmitEnable();

/******************************************************************************                                                                    
*    Function:		CAN2_ReceiveEnable
*    Description:       Enables Receive for CAN2                                                                                                                
*    Return Value:      None
******************************************************************************/
void CAN2_ReceiveEnable();

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  //_CAN2_H
/**
 End of File
*/

