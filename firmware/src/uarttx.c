/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uarttx.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "uarttx.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

UARTTX_DATA uarttxData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UARTTX_Initialize ( void )

  Remarks:
    See prototype in uarttx.h.
 */

void UARTTX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uarttxData.state = UARTTX_STATE_INIT;
    uarttxData.reset_msg[0] = 'm';
    MessageQueueWout = xQueueCreate(2, 8*sizeof(char));
    DRV_TMR0_Start();
    wait_on_ack = false;
    tx_counter = 10;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

void SendMessage(uint8_t first, uint8_t second)
{
    uarttxData.debug_data[0] = 0x8;
    uarttxData.debug_data[1] = first;
    uarttxData.debug_data[2] = second;
    uarttxData.debug_data[3] = 0x0;
    uarttxData.debug_data[4] = 0x0;
    uarttxData.debug_data[5] = 0x0;
    uarttxData.debug_data[6] = 0x0;
    uarttxData.debug_data[7] = 0x0;
    xQueueSend( MessageQueueWout, uarttxData.debug_data, pdFAIL );
    
}

/******************************************************************************
  Function:
    void UARTTX_Tasks ( void )

  Remarks:
    See prototype in uarttx.h.
 */

void ReSendMessage()
{
    xQueueSendFromISR( MessageQueueWout, uarttxData.tx_data, pdFAIL );
}

void TransmitTheMessage ()
{
    //PLIB_USART_TransmitterByteSend(USART_ID_1, 'm');
    if (uarttxData.tx_data[0] != 'm') 
    {
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[0]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[1]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[2]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[3]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[4]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[5]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[6]);
        PLIB_USART_TransmitterByteSend(USART_ID_1, uarttxData.tx_data[7]);
    }
        
}

void UARTTX_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( uarttxData.state )
    {
        /* Application's initial state. */
        case UARTTX_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                uarttxData.state = UARTTX_STATE_SERVICE_TASKS;
            }
            break;
        }

        case UARTTX_STATE_SERVICE_TASKS:
        {
            if (uxQueueMessagesWaiting(MessageQueueWout)) {
                
                xQueueReceive(MessageQueueWout, uarttxData.tx_data, portMAX_DELAY);
                if ((uarttxData.tx_data[0] & INT_MSG) == INT_MSG)     //Convert internal messages to debug
                {
                    int i = 0;
                    for (i = 4; i >= 0; i--)
                    {
                        uarttxData.tx_data[i + 1] = uarttxData.tx_data[i];
                    }
                    uarttxData.tx_data[0] = 0x00 | (PIC_ID << 3) | 0x00;
                    //                      debug     SRC          DST
                }
                uarttxData.tx_data[7] = checksumCreator(uarttxData.tx_data, 7);
                if (uarttxData.tx_data[0] & 0x80)
                {
                    uarttxData.tx_data[6] = tx_counter;
                    tx_counter = tx_counter + 1;
                    if (tx_counter == 20)
                        tx_counter = 0;
                    receiveState = WAIT_ON_ACK;
                    wait_on_ack = true;
                    uarttxData.state = UARTTX_STATE_WAIT;
                }
                //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
                
                
            }
            break;
        }
        
        case UARTTX_STATE_WAIT:
        {
            if (!wait_on_ack)
                uarttxData.state = UARTTX_STATE_SERVICE_TASKS;
            if (uxQueueMessagesWaiting(MessageQueueWout)) {
                xQueueReceive(MessageQueueWout, uarttxData.tx_data, portMAX_DELAY);
                uarttxData.tx_data[7] = checksumCreator(uarttxData.tx_data, 7);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
