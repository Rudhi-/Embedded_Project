/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    maintask.c

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

#include "maintask.h"
#include "MainTask_public.h"
#include "debug.h"
#include "queue.h"

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

MAINTASK_DATA maintaskData;

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

QueueHandle_t queue;
void milestone_update() {
    char a;
    dbgOutputLoc(SENDING_TO_QUEUE);
    xQueueSendToBackFromISR(queue, &a, NULL);
    dbgOutputLoc(SENT_TO_QUEUE);
}
/*******************************************************************************
  Function:
    void MAINTASK_Initialize ( void )

  Remarks:
    See prototype in maintask.h.
 */

void MAINTASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    maintaskData.state = MAINTASK_STATE_INIT;
    dbgOutputLoc(RECEIVING_FROM_QUEUE);
    queue = xQueueCreate(2, sizeof(char));
    if (queue == 0) {
        maintaskData.state = -1;
    }
    dbgOutputLoc(RECEIVED_FROM_QUEUE);
    dbgOutputLocReset();
    DRV_TMR0_Start();
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void MAINTASK_Tasks ( void )

  Remarks:
    See prototype in maintask.h.
 */

void MAINTASK_Tasks ( void )
{
    dbgOutputLoc(ENTERING_TASK);
    char *name = "TEAM 9";
    int i = 0;
    dbgOutputLoc(BEFORE_WHILE_LOOP);
    /* Check the application's current state. */
    switch ( maintaskData.state )
    {
        /* Application's initial state. */
        case MAINTASK_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                maintaskData.state = MAINTASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MAINTASK_STATE_SERVICE_TASKS:
        {
            char a;                 
            xQueueReceive(queue, &a, portMAX_DELAY);
            if (a == 'X') {
                dbgOutputLoc(ERROR_HAS_OCCURED);
            }
            dbgOutputVal(name[i]);
            i++;
            if (i > 5) 
                i = 0;
               
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
