/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    control.c

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

#include "control.h"
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

CONTROL_DATA controlData;

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
    void CONTROL_Initialize ( void )

  Remarks:
    See prototype in control.h.
 */

void CONTROL_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    controlData.state = CONTROL_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}



/******************************************************************************
  Function:
    void CONTROL_Tasks ( void )

  Remarks:
    See prototype in control.h.
 */


void CONTROL_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( controlData.state )
    {
        /* Application's initial state. */
        case CONTROL_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                controlData.state = CONTROL_STATE_RECEIVE_MESSAGE;
            }
            break;
        }
        /*State used to wait till a valid message has been received from the controller*/
        case CONTROL_STATE_RECEIVE_MESSAGE:
        {
            // Wait till the message has been received before moving to the 
            if (uxQueueMessagesWaiting(MessageQueueWin))
            {
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, controlData.rx_data, portMAX_DELAY);
                
                // Reset the transmit bits that will be used to send data
                controlData.tx_data[1] = 0;
                controlData.tx_data[2] = 0;
                
                // Check to see if the angle needs to be altered
                if (controlData.rx_data[1] != 0)
                {
                    //turn_left();
                    xQueueReset(MessageQueueDin);                           // Clear the magnetometer register
                    GetMagnetometerAq();                                  // Send a message to get initial angle
                    controlData.state = CONTROL_STATE_GET_CURRENT_DEGREES;  // Go to the wait on degree state
                }
                // If it deosnt have to be altered then just go to the move state
                else
                {
                    move_forward(controlData.rx_data[2], controlData.rx_data[2]);   // Set the distance that needs to be moved
                    controlData.state = CONTROL_STATE_MOVE_FORWARD;                 // Go to the move forward state
                }
            }
            break;
        }
        /*State used to get the current*/
        case CONTROL_STATE_GET_CURRENT_DEGREES:
        {
            if (uxQueueMessagesWaiting(MessageQueueDin)) 
            {
                xQueueReceive(MessageQueueDin, controlData.dx_data, portMAX_DELAY);
                controlData.curr_degrees = (controlData.dx_data[0] << 8) | controlData.dx_data[1];
                controlData.new_degrees = controlData.curr_degrees + controlData.rx_data[1];
                controlData.first = controlData.curr_degrees;
                
                SendMessage(controlData.first/2,0);
                dbgOutputVal(controlData.curr_degrees/2);
                if (controlData.rx_data[1] < 0)
                {
                    //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                    turn_right();
                }
                else
                {
                    turn_left();
                }
                controlData.state = CONTROL_STATE_TURN;
            }
            break;
        }
        
        case CONTROL_STATE_TURN:
        {
            if ( controlData.rx_data[1] < 0)
            {
                if (controlData.new_degrees < 0)
                {
                    controlData.new_degrees = 360 + controlData.new_degrees;
                    while (!(controlData.curr_degrees < controlData.new_degrees) || !(controlData.curr_degrees > controlData.new_degrees - 20))
                    {
                        GetMagnetometerData();
                        while (!uxQueueMessagesWaiting(MessageQueueDin));
                        xQueueReceive(MessageQueueDin, controlData.dx_data, portMAX_DELAY);
                        controlData.curr_degrees = (controlData.dx_data[0] << 8) | controlData.dx_data[1];
                        SendMessage(controlData.curr_degrees/2,1);
                        dbgOutputVal(controlData.curr_degrees/2);
                    }
                    controlData.tx_data[1] = controlData.first - (360 - controlData.curr_degrees);
                    SendMessage(controlData.curr_degrees/2,1);
                }
                else
                {
                    while (controlData.curr_degrees > controlData.new_degrees)
                    {
                        GetMagnetometerData();
                        while (!uxQueueMessagesWaiting(MessageQueueDin));
                        xQueueReceive(MessageQueueDin, controlData.dx_data, portMAX_DELAY);
                        controlData.curr_degrees = (controlData.dx_data[0] << 8) | controlData.dx_data[1];
                        SendMessage(controlData.curr_degrees/2,1);
                        dbgOutputVal(controlData.curr_degrees/2);
                    }
                    controlData.tx_data[1] = controlData.curr_degrees - controlData.first;
                    SendMessage(controlData.curr_degrees/2,1);
                }
                StopGettingMagData();
                move_stop();
            }
            else
            {
                if ( controlData.new_degrees > 360)
                {
                    
                    controlData.new_degrees = controlData.new_degrees - 360;
                    //dbgOutputVal(controlData.new_degrees/2);
                    while (!(controlData.curr_degrees >= controlData.new_degrees) || !(controlData.curr_degrees <= controlData.new_degrees + 20))
                    {
                        
                        GetMagnetometerData();
                        while (!uxQueueMessagesWaiting(MessageQueueDin));
                        xQueueReceive(MessageQueueDin, controlData.dx_data, portMAX_DELAY);
                        controlData.curr_degrees = (controlData.dx_data[0] << 8) | controlData.dx_data[1];
                        SendMessage(controlData.curr_degrees/2,1);
                        dbgOutputVal(controlData.curr_degrees/2);
                    }
                    controlData.tx_data[1] = (360 - controlData.first) + controlData.curr_degrees;
                    SendMessage(controlData.curr_degrees/2,1);
                }
                else
                {
                    while (controlData.curr_degrees < controlData.new_degrees)
                    {  
                        GetMagnetometerData();
                        while (!uxQueueMessagesWaiting(MessageQueueDin));
                        xQueueReceive(MessageQueueDin, controlData.dx_data, portMAX_DELAY);
                        controlData.curr_degrees = (controlData.dx_data[0] << 8) | controlData.dx_data[1];
                        SendMessage(controlData.curr_degrees/2,1);
                        dbgOutputVal(controlData.curr_degrees/2);
                    }
                    controlData.tx_data[1] = controlData.curr_degrees - controlData.first;
                    SendMessage(controlData.curr_degrees/2,1);
                }
                StopGettingMagData();
                move_stop();
                
                //dbgOutputVal(controlData.tx_data[1]);
            }
            if (controlData.rx_data[2] != 0)
            {
                move_forward(controlData.rx_data[2], controlData.rx_data[2]);
                controlData.state = CONTROL_STATE_MOVE_FORWARD;
            }
            else
                controlData.state = CONTROL_STATE_WORK_ON_DATA;
            break;
        }
        
        case CONTROL_STATE_MOVE_FORWARD:
        {
            
            if (getMoveState() == WAIT)
            {
                controlData.tx_data[2] = get_distance(LEFT);
                controlData.state = CONTROL_STATE_WORK_ON_DATA;
            }
            break;
        }
        
        case CONTROL_STATE_WORK_ON_DATA:
        {
            
            controlData.tx_data[0] = 0x8b;
            controlData.tx_data[3] = 0xaa;
            controlData.tx_data[4] = 0xaa;
            controlData.tx_data[5] = 0xaa;
            controlData.tx_data[6] = 0xaa;
            controlData.tx_data[7] = 0xaa;
            controlData.state = CONTROL_STATE_TRANSMIT_DATA;
            break;
        }
        
        case CONTROL_STATE_TRANSMIT_DATA:
        {
            xQueueSend( MessageQueueWout, controlData.tx_data, pdFAIL );
            controlData.state = CONTROL_STATE_WAIT_ACK;
            break;
        }
        
        case CONTROL_STATE_WAIT_ACK:
        {
            if (!wait_on_ack)
            {
                
                //Do some sort of check
                
                //Go back to waiting for data
                controlData.state = CONTROL_STATE_RECEIVE_MESSAGE;
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
