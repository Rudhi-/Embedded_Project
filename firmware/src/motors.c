/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motors.c

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

#include "motors.h"
#include "header.h"


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

MOTORS_DATA motorsData;

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
    void MOTORS_Initialize ( void )

  Remarks:
    See prototype in motors.h.
 */

void MOTORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motorsData.state = MOTORS_STATE_INIT;
    dbgOutputVal(IN_TASK_ONE);

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void MOTORS_Tasks ( void )

  Remarks:
    See prototype in motors.h.
 */


void MOTORS_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( motorsData.state )
    {
        /* Application's initial state. */
        case MOTORS_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                motorsData.state = MOTORS_STATE_RECEIVE_MESSAGE;
            }
            break;
        }

        case MOTORS_STATE_RECEIVE_MESSAGE:
        {
            // Wait till the message has been received before moving to the 
            // Next state
            if (uxQueueMessagesWaiting(MessageQueueWin))
            {
                
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, motorsData.rx_data, portMAX_DELAY);
                
                int i;
                for (i = 0; i < 40000000; i++)
                {
                    int j = i;
                    i = j;
                }
                // Go to the work on data state
                GetMagnetometerData();
                motorsData.state = MOTORS_STATE_WORK_ON_DATA;
                //PLIB_TMR_Counter16BitClear(TMR_ID_5);
                //PLIB_TMR_Start(TMR_ID_5);
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        case MOTORS_STATE_WORK_ON_DATA:
        {
            if (uxQueueMessagesWaiting(MessageQueueDin)) 
            {
                xQueueReceive(MessageQueueDin, motorsData.dx_data, portMAX_DELAY);
                motorsData.tx_data[0] = 0x8b;
                motorsData.tx_data[1] = motorsData.dx_data[0];
                motorsData.tx_data[2] = motorsData.dx_data[1];
                motorsData.tx_data[3] = motorsData.dx_data[2];
                motorsData.tx_data[4] = motorsData.dx_data[3];
                motorsData.tx_data[5] = 0xaa;
                motorsData.tx_data[6] = 0xaa;
                motorsData.tx_data[7] = 0xaa;
                motorsData.state = MOTORS_STATE_TRANSMIT_DATA;
            }
            break;
        }
        
        case MOTORS_STATE_TRANSMIT_DATA:
        {
            //wait_on_ack = true;
            xQueueSend( MessageQueueWout, motorsData.tx_data, pdFAIL );
            //StoreMessage(motorsData.tx_data);
            motorsData.state = MOTORS_STATE_WAIT_ACK;
            break;
        }
        
        case MOTORS_STATE_WAIT_ACK:
        {
            dbgOutputVal(IN_TASK_THREE);
            if (!wait_on_ack)
            {
                
                //Do some sort of check
                
                //Go back to waiting for data
                motorsData.state = MOTORS_STATE_RECEIVE_MESSAGE;
            }
        }

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
