/**
  ECAN2 Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    ecan2.h

  @Summary
    This is the generated header file for the ECAN2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for ECAN2.
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

#ifndef _ECAN2_H
#define _ECAN2_H

/**
  Section: Included Files
*/

#include "can_types.h"

#warning deprecate ("\nThis will be removed in future MCC releases. \nUse can_types.h file CAN message type identifiers instead. ")
/* ECAN message type identifiers */
#define CAN2_MSG_DATA    0x01
#define CAN2_MSG_RTR     0x02
#define CAN2_FRAME_EXT	0x03
#define CAN2_FRAME_STD	0x04
#define CAN2_BUF_FULL	0x05
#define CAN2_BUF_EMPTY	0x06

typedef union {
    struct {
        uint32_t id;
        uint8_t idType;
        uint8_t msgtype;
        uint8_t dlc;
        uint8_t data0;
        uint8_t data1;
        uint8_t data2;
        uint8_t data3;
        uint8_t data4;
        uint8_t data5;
        uint8_t data6;
        uint8_t data7;
    } frame;
    unsigned char array[16];
} uCAN2_MSG __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse can_types.h file uCAN_MSG instead. ")));

/* Operation modes */
typedef enum
{
	CAN2_NORMAL_OPERATION_MODE = 0,
	CAN2_DISABLE_MODE = 1,
	CAN2_LOOPBACK_MODE = 2,
	CAN2_LISTEN_ONLY_MODE = 3,
	CAN2_CONFIGURATION_MODE = 4,
	CAN2_LISTEN_ALL_MESSAGES_MODE = 7
}ECAN2_OP_MODES __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse can_types.h file CAN_OP_MODES instead. ")));

typedef enum{
    ECAN2_PRIORITY_HIGH = 0b11,
    ECAN2_PRIORITY_MEDIUM = 0b10,
    ECAN2_PRIORITY_LOW = 0b01,
    ECAN2_PRIORITY_NONE = 0b00
} ECAN2_TX_PRIOIRTY __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse can_types.h file CAN_TX_PRIOIRTY instead. ")));

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
  Section: ECAN2 Module APIs
*/

/**
  @Summary
    Initializes the ECAN2_Initialize.

  @Description
    This routine initializes the ECAN2_Initialize.
    This routine must be called before any other ECAN2 routine is called.
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
    ECAN2_Initialize();
    </code>
 */
void ECAN2_Initialize(void) __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_Initialize instead. ")));

/******************************************************************************
*                                                                             
*    Function:		ECAN2_receive
*    Description:       Receives the message from CAN buffer to user buffer                                                                         
*    Arguments:		recCanMsg: pointer to the message object
*    Return Value:      true - Receive successful
*                       false - Receive failure                                                                              
******************************************************************************/
bool ECAN2_receive(uCAN2_MSG *recCanMsg) __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_receive instead. "))); 

/******************************************************************************
*                                                                             
*    Function:		ECAN2_transmit
*    Description:       Transmits the message from user buffer to CAN buffer                                                                        
*    Arguments:		priority: priority of the message to be transmitted
*                       sendCanMsg: pointer to the message object                                        
*    Return Value:      true - Transmit successful
*                       false - Transmit failure                                                                              
******************************************************************************/
bool ECAN2_transmit(ECAN2_TX_PRIOIRTY priority, 
                                    uCAN2_MSG *sendCanMsg) __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_transmit instead. "))); 

/******************************************************************************                                                                       
*    Function:          ECAN2_isBusOff
*    Description:       Checks whether the transmitter in Bus off state                                                                                                   
*    Return Value:      true - Transmitter in Bus Off state
*                       false - Transmitter not in Bus Off state                                                                              
******************************************************************************/
bool ECAN2_isBusOff() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_isBusOff instead. "))); 

/******************************************************************************                                                                 
*    Function:		ECAN2_isRXErrorPassive
*    Description:       Checks whether the receive in error passive state                                    
*    Return Value:      true - Receiver in Error Passive state
*                       false - Receiver not in Error Passive state                                                                              
******************************************************************************/
bool ECAN2_isRXErrorPassive() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_isRXErrorPassive instead. ")));

/******************************************************************************                                                                      
*    Function:		ECAN2_isTXErrorPassive
*    Description:       Checks whether the transmitter in error passive state                                                                                                             
*    Return Value:      true - Transmitter in Error Passive state
*                       false - Transmitter not in Error Passive state                                                                              
******************************************************************************/
bool ECAN2_isTXErrorPassive() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_isTXErrorPassive instead. ")));

/******************************************************************************                                                                    
*    Function:		ECAN2_messagesInBuffer
*    Description:       returns the number of messages that are received                                                                                                               
*    Return Value:      Number of message received
******************************************************************************/
uint8_t ECAN2_messagesInBuffer() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_messagesInBuffer instead. ")));

/******************************************************************************
*                                                                             
*    Function:		ECAN2_sleep
*    Description:       Puts ECAN2 module in disable mode.
*    Return Value:      None
*                                                                       
******************************************************************************/
void ECAN2_sleep() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_sleep instead. ")));

/******************************************************************************                                                                    
*    Function:		ECAN2_TransmitEnable
*    Description:       Enables Transmit for ECAN2                                                                                                                
*    Return Value:      None
******************************************************************************/
void ECAN2_TransmitEnable() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_TransmitEnable instead. ")));

/******************************************************************************                                                                    
*    Function:		ECAN2_ReceiveEnable
*    Description:       Enables Receive for ECAN2                                                                                                                
*    Return Value:      None
******************************************************************************/
void ECAN2_ReceiveEnable() __attribute__((deprecate ("\nThis will be removed in future MCC releases. \nUse CAN2_ReceiveEnable instead. ")));


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  //_ECAN2_H
/**
 End of File
*/

