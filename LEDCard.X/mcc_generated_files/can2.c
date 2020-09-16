/**
  CAN2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    can2.c

  @Summary
    This is the generated driver implementation file for the CAN2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for CAN2.
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

#include "can2.h"
#include "dma.h"

#define CAN2_TX_DMA_CHANNEL DMA_CHANNEL_1
#define CAN2_RX_DMA_CHANNEL DMA_CHANNEL_0

/* Valid options are 4, 6, 8, 12, 16, 24, or 32. */
#define CAN2_MESSAGE_BUFFERS         32

#define CAN2_TX_BUFFER_COUNT 1

/******************************************************************************/

/******************************************************************************/
/* Private type definitions                                               */
/******************************************************************************/
typedef struct __attribute__((packed))
{
    unsigned priority                   :2;
    unsigned remote_transmit_enable     :1;
    unsigned send_request               :1;
    unsigned error                      :1;
    unsigned lost_arbitration           :1;
    unsigned message_aborted            :1;
    unsigned transmit_enabled           :1;
} CAN2_TX_CONTROLS;

/******************************************************************************/
/* Private variable definitions                                               */
/******************************************************************************/
/* This alignment is required because of the DMA's peripheral indirect 
 * addressing mode. */
static unsigned int can2msgBuf [CAN2_MESSAGE_BUFFERS][8] __attribute__((aligned(32 * 8 * 2)));

/******************************************************************************/
/* Private function prototypes                                                */
/******************************************************************************/
static void CAN2_DMACopy(uint8_t buffer_number, uCAN_MSG *message);
static void CAN2_MessageToBuffer(uint16_t* buffer, uCAN_MSG* message);

void __attribute__((__interrupt__, no_auto_psv)) _C2Interrupt(void)  
{   

    if(C2INTFbits.WAKIF)
    {
        C2INTFbits.WAKIF = 0;
    }
   
    IFS3bits.C2IF = 0;
}

/**
  Section: CAN2 APIs
*****************************************************************************************/

void CAN2_Initialize(void)
{
    // Disable interrupts before the Initialization
    IEC3bits.C2IE = 0;
    C2INTE = 0;

    // set the CAN2_initialize module to the options selected in the User Interface

    /* put the module in configuration mode */
    C2CTRL1bits.REQOP = CAN_CONFIGURATION_MODE;
    while(C2CTRL1bits.OPMODE != CAN_CONFIGURATION_MODE);

    /* Set up the baud rate*/	
    C2CFG1 = 0x31;	//BRP TQ = (2 x 50)/FCAN; SJW 1 x TQ; 
    C2CFG2 = 0x2BA;	//WAKFIL disabled; SEG2PHTS Freely programmable; SEG2PH 3 x TQ; SEG1PH 8 x TQ; PRSEG 3 x TQ; SAM Once at the sample point; 
    C2FCTRL = 0xC001;	//FSA Transmit/Receive Buffer TRB1; DMABS 32; 
    C2FEN1 = 0x00;	//FLTEN8 disabled; FLTEN7 disabled; FLTEN9 disabled; FLTEN0 disabled; FLTEN2 disabled; FLTEN10 disabled; FLTEN1 disabled; FLTEN11 disabled; FLTEN4 disabled; FLTEN3 disabled; FLTEN6 disabled; FLTEN5 disabled; FLTEN12 disabled; FLTEN13 disabled; FLTEN14 disabled; FLTEN15 disabled; 
    C2CTRL1 = 0x00;	//CANCKS FOSC/2; CSIDL disabled; ABAT disabled; REQOP Sets Normal Operation Mode; WIN Uses buffer window; CANCAP disabled; 

    /* Filter configuration */
    /* enable window to access the filter configuration registers */
    /* use filter window*/
    C2CTRL1bits.WIN=1;	   
    
    /* Configure the masks */
    C2RXM0SIDbits.SID = 0x0; 
    C2RXM1SIDbits.SID = 0x0; 
    C2RXM2SIDbits.SID = 0x0; 
    
    C2RXM0SIDbits.EID = 0x0; 
    C2RXM1SIDbits.EID = 0x0; 
    C2RXM2SIDbits.EID = 0x0; 
    
    C2RXM0EID = 0x00;     	
    C2RXM1EID = 0x00;     	
    C2RXM2EID = 0x00;     	
    
    C2RXM0SIDbits.MIDE = 0x0; 
    C2RXM1SIDbits.MIDE = 0x0; 
    C2RXM2SIDbits.MIDE = 0x0; 
    
    /* clear window bit to access CAN2 control registers */
    C2CTRL1bits.WIN=0;    

    /*configure CAN2 Transmit/Receive buffer settings*/
    C2TR01CONbits.TXEN0 = 0x1; // Buffer 0 is a Transmit Buffer 
    C2TR01CONbits.TXEN1 = 0x0; // Buffer 1 is a Receive Buffer 
    C2TR23CONbits.TXEN2 = 0x0; // Buffer 2 is a Receive Buffer 
    C2TR23CONbits.TXEN3 = 0x0; // Buffer 3 is a Receive Buffer 
    C2TR45CONbits.TXEN4 = 0x0; // Buffer 4 is a Receive Buffer 
    C2TR45CONbits.TXEN5 = 0x0; // Buffer 5 is a Receive Buffer 
    C2TR67CONbits.TXEN6 = 0x0; // Buffer 6 is a Receive Buffer 
    C2TR67CONbits.TXEN7 = 0x0; // Buffer 7 is a Receive Buffer 

    C2TR01CONbits.TX0PRI = 0x0; // Message Buffer 0 Priority Level
    C2TR01CONbits.TX1PRI = 0x0; // Message Buffer 1 Priority Level
    C2TR23CONbits.TX2PRI = 0x0; // Message Buffer 2 Priority Level
    C2TR23CONbits.TX3PRI = 0x0; // Message Buffer 3 Priority Level
    C2TR45CONbits.TX4PRI = 0x0; // Message Buffer 4 Priority Level
    C2TR45CONbits.TX5PRI = 0x0; // Message Buffer 5 Priority Level
    C2TR67CONbits.TX6PRI = 0x0; // Message Buffer 6 Priority Level
    C2TR67CONbits.TX7PRI = 0x0; // Message Buffer 7 Priority Level

    /* clear the buffer and overflow flags */   
    C2RXFUL1 = 0x0000;
    C2RXFUL2 = 0x0000;
    C2RXOVF1 = 0x0000;
    C2RXOVF2 = 0x0000;	

    /* configure the device to interrupt on the receive buffer full flag */
    /* clear the buffer full flags */ 	
    C2INTFbits.RBIF = 0;  

    /* put the module in normal mode */
    C2CTRL1bits.REQOP = CAN_NORMAL_OPERATION_MODE;
    while(C2CTRL1bits.OPMODE != CAN_NORMAL_OPERATION_MODE);	
	
    /* Enable CAN2 Interrupt */
    IEC3bits.C2IE = 1;

}

/******************************************************************************
*                                                                             
*    Function:		CAN2_TransmitEnable
*    Description:       Setup the DMA for Transmit from the CAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/

void CAN2_TransmitEnable()
{
    /* setup channel 0 for peripheral indirect addressing mode 
    normal operation, word operation and select as Tx to peripheral */

    /* DMA_PeripheralIrqNumberSet and DMA_TransferCountSet would be done in the 
    DMA */
    
    /* setup the address of the peripheral CAN2 (C2TXD) */ 
    DMA_PeripheralAddressSet(CAN2_TX_DMA_CHANNEL, (uint16_t) &C2TXD);

    /* DPSRAM start address offset value */ 
    DMA_StartAddressASet(CAN2_TX_DMA_CHANNEL, __builtin_dmaoffset(&can2msgBuf));

    /* enable the channel */
    DMA_ChannelEnable(CAN2_TX_DMA_CHANNEL);
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_ReceiveEnable
*    Description:       Setup the DMA for Receive on the CAN module.  The 
*                       relevant DMA module APIs are grouped in this function 
*                       and this API needs to be called after DMA_Initialize 
*                       and CAN_Initialize
*                                                                                                                                                       
******************************************************************************/

void CAN2_ReceiveEnable()
{
    /* setup DMA channel for peripheral indirect addressing mode 
    normal operation, word operation and select as Rx to peripheral */
    
    /* setup the address of the peripheral CAN2 (C2RXD) */     
    /* DMA_TransferCountSet and DMA_PeripheralIrqNumberSet would be set in 
    the DMA_Initialize function */

    DMA_PeripheralAddressSet(CAN2_RX_DMA_CHANNEL, (uint16_t) &C2RXD);

    /* DPSRAM start address offset value */ 
    DMA_StartAddressASet(CAN2_RX_DMA_CHANNEL, __builtin_dmaoffset(&can2msgBuf) );	  

    /* enable the channel */
    DMA_ChannelEnable(CAN2_RX_DMA_CHANNEL);
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_transmit
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

bool CAN2_transmit(CAN_TX_PRIOIRTY priority, uCAN_MSG *sendCanMsg) 
{
    CAN2_TX_CONTROLS * pTxControls = (CAN2_TX_CONTROLS*)&C2TR01CON;
    uint_fast8_t i;
    bool messageSent = false;

    if(CAN2_TX_BUFFER_COUNT > 0)
    {
        for(i=0; i<CAN2_TX_BUFFER_COUNT; i++)
        {
            if(pTxControls->transmit_enabled == 1)
            {
                if (pTxControls->send_request == 0)
                {
                    CAN2_MessageToBuffer(&can2msgBuf[i][0], sendCanMsg);
                    pTxControls->priority = priority;

                    /* set the message for transmission */
                    pTxControls->send_request = 1; 

                    messageSent = true;
                    break;
                }
            }

            pTxControls++;
        }
    }    
    return messageSent;
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_receive
*    Description:       Receives the message from CAN buffer to user buffer 
*                                                                             
*    Arguments:		recCanMsg: pointer to the message object
*                                             
*    Return Value:      true - Receive successful
*                       false - Receive failure                                                                              
******************************************************************************/

bool CAN2_receive(uCAN_MSG *recCanMsg) 
{   
    /* We use a static buffer counter so we don't always check buffer 0 first
     * resulting in potential starvation of later buffers.
     */
    static uint_fast8_t currentDedicatedBuffer = 0;
    uint_fast8_t i;
    bool messageReceived = false;
    uint16_t receptionFlags;
    
    receptionFlags = C2RXFUL1;
	
    if (receptionFlags != 0)  
    {
        /* check which message buffer is free */  
        for (i=0 ; i < 16; i++)
        {
            if (((receptionFlags >> currentDedicatedBuffer ) & 0x1) == 0x1)
            {           
               CAN2_DMACopy(currentDedicatedBuffer, recCanMsg);
           
               C2RXFUL1 &= ~(1 << currentDedicatedBuffer);
  
               messageReceived = true;
            }
            
            currentDedicatedBuffer++;
            
            if(currentDedicatedBuffer >= 16)
            {
                currentDedicatedBuffer = 0;
            }
            
            if(messageReceived == true)
            {
                break;
            }
        }
    }
        
    return (messageReceived);
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_isBusOff
*    Description:       Checks whether the transmitter in Bus off state
*                                                                             
                                             
*    Return Value:      true - Transmitter in Bus Off state
*                       false - Transmitter not in Bus Off state                                                                              
******************************************************************************/

bool CAN2_isBusOff() 
{
    return C2INTFbits.TXBO;	
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_isRXErrorPassive
*    Description:       Checks whether the receive in error passive state
*                                             
*    Return Value:      true - Receiver in Error Passive state
*                       false - Receiver not in Error Passive state                                                                              
******************************************************************************/

bool CAN2_isRXErrorPassive()
{
    return C2INTFbits.RXBP;   
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_isTXErrorPassive
*    Description:       Checks whether the transmitter in error passive state                                                                          
*                                             
*    Return Value:      true - Transmitter in Error Passive state
*                       false - Transmitter not in Error Passive state                                                                              
******************************************************************************/

bool CAN2_isTXErrorPassive()
{
    return (C2INTFbits.TXBP);
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_messagesInBuffer
*    Description:       returns the number of messages that are received                                                                           
*                                             
*    Return Value:      Number of message received
******************************************************************************/

uint8_t CAN2_messagesInBuffer() 
{
    uint_fast8_t messageCount;
    uint_fast8_t currentBuffer;
    uint16_t receptionFlags;
   
    messageCount = 0;

    /* Check any message in buffer 0 to buffer 15*/
    receptionFlags = C2RXFUL1;
    if (receptionFlags != 0) 
    {
        /* check whether a message is received */  
        for (currentBuffer=0 ; currentBuffer < 16; currentBuffer++)
        {
            if (((receptionFlags >> currentBuffer ) & 0x1) == 0x1)
            {
                messageCount++;
            }
        }
    }
            
    return (messageCount);
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_sleep
*    Description:       Puts CAN2 module in disable mode.
*                                                                       
******************************************************************************/
void CAN2_sleep(void) 
{
    C2INTFbits.WAKIF = 0;
    C2INTEbits.WAKIE = 1;

    /* put the module in disable mode */
    C2CTRL1bits.REQOP = CAN_DISABLE_MODE;
    while(C2CTRL1bits.OPMODE != CAN_DISABLE_MODE);
    
    //Wake up from sleep should set the CAN module straight into Normal mode
}

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/******************************************************************************
*                                                                             
*    Function:		CAN2_DMACopy
*    Description:       moves the message from the DMA memory to RAM
*                                                                             
*    Arguments:		*message: a pointer to the message structure in RAM 
*			that will store the message. 
*	                                                                 
*                                                                              
******************************************************************************/

static void CAN2_DMACopy(uint8_t buffer_number, uCAN_MSG *message)
{
    uint16_t ide=0;
    uint16_t rtr=0;
    uint32_t id=0;

    /* read word 0 to see the message type */
    ide=can2msgBuf[buffer_number][0] & 0x0001U;			

    /* check to see what type of message it is */
    /* message is standard identifier */
    if(ide==0U)
    {
        message->frame.id=(can2msgBuf[buffer_number][0] & 0x1FFCU) >> 2U;		
        message->frame.idType = CAN_FRAME_STD;
        rtr=can2msgBuf[buffer_number][0] & 0x0002U;
    }
    /* message is extended identifier */
    else
    {
        id=can2msgBuf[buffer_number][0] & 0x1FFCU;		
        message->frame.id = id << 16U;
        message->frame.id += ( ((uint32_t)can2msgBuf[buffer_number][1] & (uint32_t)0x0FFF) << 6U );
        message->frame.id += ( ((uint32_t)can2msgBuf[buffer_number][2] & (uint32_t)0xFC00U) >> 10U );		
        message->frame.idType = CAN_FRAME_EXT;
        rtr=can2msgBuf[buffer_number][2] & 0x0200;
    }
    /* check to see what type of message it is */
    /* RTR message */
    if(rtr != 0U)
    {
        /* to be defined ?*/
        message->frame.msgtype = CAN_MSG_RTR;	
    }
    /* normal message */
    else
    {
        message->frame.msgtype = CAN_MSG_DATA;
        message->frame.data0 =(unsigned char)can2msgBuf[buffer_number][3];
        message->frame.data1 =(unsigned char)((can2msgBuf[buffer_number][3] & 0xFF00U) >> 8U);
        message->frame.data2 =(unsigned char)can2msgBuf[buffer_number][4];
        message->frame.data3 =(unsigned char)((can2msgBuf[buffer_number][4] & 0xFF00U) >> 8U);
        message->frame.data4 =(unsigned char)can2msgBuf[buffer_number][5];
        message->frame.data5 =(unsigned char)((can2msgBuf[buffer_number][5] & 0xFF00U) >> 8U);
        message->frame.data6 =(unsigned char)can2msgBuf[buffer_number][6];
        message->frame.data7 =(unsigned char)((can2msgBuf[buffer_number][6] & 0xFF00U) >> 8U);
        message->frame.dlc =(unsigned char)(can2msgBuf[buffer_number][2] & 0x000FU);
    }
}

/******************************************************************************
*                                                                             
*    Function:		CAN2_MessageToBuffer
*    Description:       This function takes the input message, reformats it, 
*                       and copies it to the specified CAN module buffer
*                                                                             
*    Arguments:		*buffer: a pointer to the buffer where the message 
*                       would be stored 
*                       *message: pointer to the input message that is received
*                       by the CAN module 	                                                                 
*                                                                              
******************************************************************************/

static void CAN2_MessageToBuffer(uint16_t* buffer, uCAN_MSG* message)
{   
    if(message->frame.idType == CAN_FRAME_STD)
    {
        buffer[0]= (message->frame.id & 0x000007FF) << 2;
        buffer[1]= 0;
        buffer[2]= message->frame.dlc & 0x0F;
    }
    else
    {
        buffer[0]= ( ( (uint16_t)(message->frame.id >> 16 ) & 0x1FFC ) ) | 0b1;
        buffer[1]= (uint16_t)(message->frame.id >> 6) & 0x0FFF;
        buffer[2]= (message->frame.dlc & 0x0F) + ( (uint16_t)(message->frame.id << 10) & 0xFC00);
    }

    buffer[3]= ((message->frame.data1)<<8) + message->frame.data0;
    buffer[4]= ((message->frame.data3)<<8) + message->frame.data2;
    buffer[5]= ((message->frame.data5)<<8) + message->frame.data4;
    buffer[6]= ((message->frame.data7)<<8) + message->frame.data6;
}

/**
 End of File
*/