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

bool isRunning() {
    return controlData.running;
}


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
    controlData.sendData = true;
    controlData.running = false;
    controlData.prevData = 0x00;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    MessageQueueControl = xQueueCreate(2, 8*sizeof(char));
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
            
                controlData.state = CONTROL_WAIT;
            }
            break;
        }

        case CONTROL_WAIT:
        {
            controlData.running = false;
            if (getMoveState() == WAIT) {
                if (uxQueueMessagesWaiting(MessageQueueWin)) {
                    //received start message
                    xQueueReceive(MessageQueueWin, controlData.rx_data, portMAX_DELAY);
                    if ((controlData.rx_data[1] == 0xF0) && (controlData.rx_data[2] == 0xF0) && (controlData.rx_data[3] == 0xF0)
                            && (controlData.rx_data[4] == 0xF0) && (controlData.rx_data[5] == 0xF0)) {
                        startReflectance();
                        controlData.state = CONTROL_RUN;
                    }
                }
#ifdef DEBUGGING
                startReflectance();
                controlData.state = CONTROL_RUN;
                //controlData.sendData = false;
#endif
            }
            break;
        }
        
        case CONTROL_RUN:
        {
            controlData.running = true;
            if (uxQueueMessagesWaiting(MessageQueueWin)) {
                xQueueReceive(MessageQueueWin, controlData.rx_data, portMAX_DELAY);
                //received start message
                if ((controlData.rx_data[1] == 0x0F) && (controlData.rx_data[2] == 0x0F) && (controlData.rx_data[3] == 0x0F)
                        && (controlData.rx_data[4] == 0x0F) && (controlData.rx_data[5] == 0x0F)) {
                    stopReflectance();
                    controlData.state = CONTROL_WAIT;
                    move_stop();
                    break;
                }
            }
            if (uxQueueMessagesWaiting(MessageQueueControl)) {   
                    xQueueReceive(MessageQueueControl, controlData.rx_data, portMAX_DELAY);
                    
                    if (controlData.rx_data[0] == (INT_MSG | 0x00 << 4 | REFLECTANCE_THREAD_ID << 2 | CONTROL_THREAD_ID )) {
                        PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                        if (controlData.rx_data[1] && (controlData.rx_data[1] != 0xFF)) {
                            if ((controlData.rx_data[1] & 0xc0) && (controlData.rx_data[1] & 0x03)) {
                                //stop bad state
                                move_stop();
                            } else if ((controlData.rx_data[1] & 0x30) && (controlData.rx_data[1] & 0x0c)) {
                                //move straight
                                set_speed(JOG, JOG);
                                move_start();
                            } else if (controlData.rx_data[1] & 0x03){
                                //turn sharp left
                                set_speed(SPRINT, STOP);
                                move_start();
                            } else if (controlData.rx_data[1] & 0x0c) {
                                //turn left
                                set_speed(SPRINT, CRAWL);
                                move_start();
                            } else if (controlData.rx_data[1] & 0xc0) {
                                //turn sharp right
                                set_speed(STOP, SPRINT);
                                move_start();
                            } else if (controlData.rx_data[1] & 0x30) {
                                //turn right
                                set_speed(CRAWL, SPRINT);
                                move_start();
                            }
                        } else {
                            move_stop();
                        }
                        //send data to server
                        if (controlData.sendData && controlData.rx_data[1] != controlData.prevData) {
                            int i;
                            for (i = 0; i < 8; i++)
                            {
                                controlData.tx_data[i] = controlData.tx_data[i];
                            }
                            controlData.prevData = controlData.rx_data[1];
                            xQueueSend( MessageQueueWout, controlData.rx_data, pdFAIL );
                        }
                    } else if (controlData.rx_data[0] == (INT_MSG | 0x02 << 4 | CONTROL_THREAD_ID << 2 | CONTROL_THREAD_ID )) {
                        controlData.sendData = (bool) controlData.rx_data[1];
                    }
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
