/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    brains.c

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

#include "brains.h"

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

BRAINS_DATA brainsData;
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
    void BRAINS_Initialize ( void )

  Remarks:
    See prototype in brains.h.
 */

void BRAINS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    brainsData.state = BRAINS_STATE_INIT;
    brainsData.initMagnitude = 0; 
    brainsData.initDirection = 0; 

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void BRAINS_Tasks ( void )

  Remarks:
    See prototype in brains.h.
 */

void BRAINS_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( brainsData.state )
    {
        /* Application's initial state. */
        case BRAINS_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                brainsData.state = BRAINS_STATE_SERVICE_TASKS;
            }
            break;
        }
        case BRAINS_STATE_SERVICE_TASKS:
        {
           brainsData.state = WAIT_ON_START;
            break;
        }
        case WAIT_ON_START:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)) {
                
                xQueueReceive(MessageQueueWin, brainsData.receivedMessage, portMAX_DELAY);
                if(brainsData.receivedMessage[0] == 0x83){ // from raspi to pic 3
                    //Storing destination information 
                    brainsData.initMagnitude = (brainsData.receivedMessage[2] << 8) | brainsData.receivedMessage[1];
                    brainsData.initDirection = brainsData.receivedMessage[3];

                    // sending Start command to PIC 2
                    brainsData.sendMessage[0] = 0x9A;
                    brainsData.sendMessage[1] = 0xFF;
                    brainsData.sendMessage[2] = 0xFF;
                    brainsData.sendMessage[3] = 0xFF;
                    brainsData.sendMessage[4] = 0xFF;
                    brainsData.sendMessage[5] = 0xFF;
                    brainsData.sendMessage[6] = 0xFF;
                    brainsData.sendMessage[7] = checksumCreator(brainsData.sendMessage,7);
                    xQueueSendToBack(MessageQueueWout,brainsData.sendMessage,pdFAIL);

                    // Blocking portion 
                    while(wait_on_ack){};

                    // Send initial set of movement commands to PIC 1
                    brainsData.sendMessage[0] = 0x99;
                    brainsData.sendMessage[1] = 0xFF;
                    brainsData.sendMessage[2] = 0xFF;
                    brainsData.sendMessage[3] = 0xFF;
                    brainsData.sendMessage[4] = 0xFF;
                    brainsData.sendMessage[5] = 0xFF;
                    brainsData.sendMessage[6] = 0xFF;
                    brainsData.sendMessage[7] = checksumCreator(brainsData.sendMessage,7);
                    xQueueSendToBack(MessageQueueWout,brainsData.sendMessage,pdFAIL);

                    // advance to next state 
                    brainsData.state=WAIT_ON_DATA;
                }
            }

            break;
        }
        case WAIT_ON_DATA:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)) {
                xQueueReceive(MessageQueueWin, brainsData.receivedMessage, portMAX_DELAY);
                if(crcMatches(brainsData.receivedMessage,7)){                           
                    /*
                     * If destination is to PIC 3 
                     * then message is relevant
                     * in which case move onto next state                            
                     */
                   // if((brainsData.receivedMessage[0]&) == 0x43){
                        if(brainsData.receivedMessage[1] == 0x00 &&
                           brainsData.receivedMessage[2] == 0x00 &&
                           brainsData.receivedMessage[3] == 0x00 &&
                           brainsData.receivedMessage[4] == 0x00 &&
                           brainsData.receivedMessage[5] == 0x00 &&
                           brainsData.receivedMessage[6] == 0x00){ // was stop signal received? 
                           brainsData.state=STOP;
                        }
                        else{ // stop signal not received
                            brainsData.state=COMPUTING_DATA;
                        }                                
                    //}
                }
                else{
                    // Response needs to be sent again
                }
            }   
            break;
        }
        case COMPUTING_DATA:
        {
            /*TODO Implement some sort of calculations*/
            int i = 0;
            int j = 0;
            for(i=0;i<1000000;i++){
                j =i;
                i = j;
            }
            // sending data to PIC 1
            brainsData.sendMessage[0] = 0x99;
            brainsData.sendMessage[1] = 0xAA;
            brainsData.sendMessage[2] = 0xAA;
            brainsData.sendMessage[3] = 0xAA;
            brainsData.sendMessage[4] = 0xBB;
            brainsData.sendMessage[5] = 0xBB;
            brainsData.sendMessage[6] = 0xBB;
            brainsData.sendMessage[7] = checksumCreator(brainsData.sendMessage,7);
            xQueueSendToBack(MessageQueueWout,brainsData.sendMessage,pdFAIL);
            while(wait_on_ack){};
            brainsData.state=WAIT_ON_DATA;
            break;
        }
        case ACK:
        {
            // literally does nothing, simply stay in this state until 
            // we get switched to a different state
            break;
        }
        case STOP:
        {
            // Send STOP signal to PIC 1
            brainsData.sendMessage[0] = 0x99;
            brainsData.sendMessage[1] = 0x00;
            brainsData.sendMessage[2] = 0x00;
            brainsData.sendMessage[3] = 0x00;
            brainsData.sendMessage[4] = 0x00;
            brainsData.sendMessage[5] = 0x00;
            brainsData.sendMessage[6] = 0x00;
            brainsData.sendMessage[7] = checksumCreator(brainsData.sendMessage,7);
            xQueueSendToBack(MessageQueueWout,brainsData.sendMessage,pdFAIL);
            while(wait_on_ack){};

            //sending signal to raspi
            brainsData.sendMessage[0] = 0x98;
            brainsData.sendMessage[7] = checksumCreator(brainsData.sendMessage,7);
            xQueueSendToBack(MessageQueueWout,brainsData.sendMessage,pdFAIL);
            while(wait_on_ack){};
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
