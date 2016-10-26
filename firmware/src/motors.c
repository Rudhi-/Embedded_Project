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
#include "system_config/default/framework/driver/tmr/drv_tmr_static.h"

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

// Helper functions
//initialize motors to be able to run
void init_motors() {
    //right motor enable
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0);
    //right motor direction
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    
    //left motor enable
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1);
    //left motor direction
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    
    //Start OC drivers    
    PLIB_OC_Enable(OC_ID_1);
    PLIB_OC_Enable(OC_ID_2);
    
    set_speed(WALK,WALK);
    move_stop();
    
    R_encoder = 0;
    L_encoder = 0;
    
    //PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
    //PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);    
    
    //start timer
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();    
}

// Follower rover
// Clear is forwards, set is backwards
//set the speeds to given inputs
void set_speed(MOTOR_SPEEDS leftSpeed, MOTOR_SPEEDS rightSpeed) {
    //left motor
    if (leftSpeed < 0) {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
        leftSpeed *= -1;
    } else {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    motorsData.leftSpeed = leftSpeed;
    
    //right motor
    if (rightSpeed < 0) {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
        rightSpeed *= -1;
    } else {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    motorsData.rightSpeed = rightSpeed;

    move_start();
}

//get the current speed of one of the motors
MOTOR_SPEEDS get_speed(SIDE side) {
    switch (side) {
        case LEFT:
            return motorsData.leftSpeed;
        case RIGHT:
            return motorsData.rightSpeed;
        default:
            return 0;
    }
}

//Stops all motors immediately
void move_stop() {
    //right motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    //left motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
    
}

//sets motors to drive at current speed and direction
void move_start() {
    //right motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorsData.rightSpeed);
    //left motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorsData.leftSpeed);    
}


// Leader Rover
//turn the rover to the right
void turn_left() {
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK); 
    
    //stop left motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
}

//spin the rover clockwise
void spin_left() {
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK); 
    
    //left motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK);  
}

//turn the rover to the left
void turn_right() {
    //stop right motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    
    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK);  
}

//spin the rover counterclockwise
void spin_right() {
    //right motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK); 
    
    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK);  
}

//function to move forwards
void move_forward() {
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorsData.rightSpeed); 
    
    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorsData.leftSpeed);         
}

//function to move backwards
void move_backward() {
    //right motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorsData.rightSpeed); //placeholder
    
    //left motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorsData.leftSpeed); //placeholder        
}


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

    init_motors();
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
            
                motorsData.state = MOTORS_STATE_SERVICE_TASKS;
            }
            motorsData.moveState = INIT;
            break;
        }

        case MOTORS_STATE_SERVICE_TASKS:
        {
            uint8_t msg[8];
            switch (motorsData.moveState) {
                case INIT:
                    move_stop();
                    R_encoder = L_encoder = 0;
                    if (uxQueueMessagesWaiting(MessageQueueWout)) {
                        xQueueReceive(MessageQueueWin, msg, portMAX_DELAY);
                        motorsData.moveState = MOVE_F;
                        set_speed(CRAWL, CRAWL);

}                    break;
                case MOVE_F:
                    if (R_encoder > 50) {
                        move_stop();
                        R_encoder = L_encoder = 0;
                        if (get_speed(RIGHT) == CRAWL) {
                            spin_right();
                        } else {
                            spin_left();
                        }
                        motorsData.moveState = SPIN;
                    }
                    break;
                case SPIN:
                    if (R_encoder > 98 || L_encoder > 98) {
                        move_stop();
                        R_encoder = L_encoder = 0;
                        if (get_speed(RIGHT) == CRAWL) {
                            set_speed(SPRINT, SPRINT);
                        } else {
                            set_speed(CRAWL, CRAWL);
                        }
                        motorsData.moveState = MOVE_F;
                    }
                    break;
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
