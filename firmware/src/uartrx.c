/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uartrx.c

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

#include "uartrx.h"

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

UARTRX_DATA uartrxData;

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
    void UARTRX_Initialize ( void )

  Remarks:
    See prototype in uartrx.h.
 */

void UARTRX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    uartrxData.state = UARTRX_STATE_INIT;
    receiveState = WAIT_ON_MESSAGE;
    MessageQueueWin = xQueueCreate(2, 8*sizeof(char));
    makeCRCTable();
    
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void UARTRX_Tasks ( void )

  Remarks:
    See prototype in uartrx.h.
 */

void SendToTheQueue()
{
    uartrxData.rx_data[0] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
    if (uartrxData.rx_data[0] != 'm')
    {
        
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[1] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[2] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[3] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[4] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[5] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[6] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        while (!PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){};
        uartrxData.rx_data[7] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        
        if ((uartrxData.rx_data[0] & 0x07) == PIC_ID) {
            switch (receiveState) {
                case WAIT_ON_MESSAGE:
                    if (crcMatches(uartrxData.rx_data, 7) && (uartrxData.rx_data[0] & 0x80)) {
                        //send ack
                        uartrxData.tx_data[0] = (0x40 | (PIC_ID << 3) | ((uartrxData.rx_data[0] & 0x38) >> 3));
                        int i;
                        for (i = 1; i < 7; i++) {
                            uartrxData.tx_data[i] = 0x00;
                        }
                        xQueueSendFromISR( MessageQueueWout, uartrxData.tx_data, pdFAIL);
                        //pass data in
                        xQueueSendFromISR( MessageQueueWin, uartrxData.rx_data, pdFAIL );
                    }
                    break;
                case WAIT_ON_ACK:
                    
                    if ((uartrxData.rx_data[0] & 0x80)) {
                        break;
                    }
                    // xQueueSendFromISR( MessageQueueWin, uartrxData.rx_data, pdFAIL );
                    wait_on_ack = false;
                    receiveState = WAIT_ON_MESSAGE;
                    break;
            }
        }
                
    }
    else
    {
        xQueueSendFromISR( MessageQueueWout, uartrxData.rx_data, pdFAIL );
    }
}

void UARTRX_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( uartrxData.state )
    {
        /* Application's initial state. */
        case UARTRX_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized)
            {
            
                uartrxData.state = UARTRX_STATE_SERVICE_TASKS;
            }
            break;
        }

        case UARTRX_STATE_SERVICE_TASKS:
        {
            
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
