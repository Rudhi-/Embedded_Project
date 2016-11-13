/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    main_task.c

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

#include "main_task.h"
#include "driver/tmr/drv_tmr_mapping.h"

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

MAIN_TASK_DATA main_taskData;

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
    void MAIN_TASK_Initialize ( void )

  Remarks:
    See prototype in main_task.h.
 */

void MAIN_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    main_taskData.state = MAIN_TASK_STATE_INIT;

    int i = 0;
    for(i = 0; i < 7; i++){
        packet_tx_data[i] = 0;
    }
    
    timer_LED_ON = xTimerCreate("timer_LED_ON", pdMS_TO_TICKS( 100 ), pdTRUE, ( void * ) 0, callback_LED_ON);
    xTimerStart(timer_LED_ON, 0);
    
    timer_LED_OFF = xTimerCreate("timer_LED_OFF", pdMS_TO_TICKS( 4 ), pdTRUE, ( void * ) 0, callback_LED_OFF);
    
    timer_LED_INPUT = xTimerCreate("timer_LED_INPUT", pdMS_TO_TICKS( 4 ), pdTRUE, ( void * ) 0, callback_LED_INPUT);

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void MAIN_TASK_Tasks ( void )

  Remarks:
    See prototype in main_task.h.
 */

void MAIN_TASK_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( main_taskData.state )
    {
        /* Application's initial state. */
        case MAIN_TASK_STATE_INIT:
        {
            bool appInitialized = true;
            
            //DRV_TMR1_Start();
            ultrasonic_finished = 0;
            reflectance_finished = 0;
            
            packet_tx_data[0] = 0x13;
        
            if (appInitialized)
            {
            
                main_taskData.state = MAIN_TASK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MAIN_TASK_STATE_SERVICE_TASKS:
        {
            /*
            if(ultrasonic_finished == 1 && reflectance_finished == 1){
                dbgOutputVal(0x45);
            }
            else{
                dbgOutputVal(0x44);
            }
            */
            
            if(start_senddata){
                start_senddata = 0;
                
                /*
                int i = 0;
                for(i = 0; i < 7; i++){
                    dbgOutputVal(packet_tx_data[4]);
                    dbgOutputVal(0x00);
                    dbgOutputVal(0x00);
                }
                */
                
                //if(!wait_on_ack){
                if(1){
                    xQueueSend( MessageQueueWout, packet_tx_data, pdFAIL );
                }
                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
            }
            /*
            if (uxQueueMessagesWaiting(MessageQueueWout)) 
            {
                xQueueReceive(MessageQueueWin, main_taskData.rx_data, portMAX_DELAY);
            }
            */
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
