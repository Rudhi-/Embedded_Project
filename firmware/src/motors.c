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
    
    MessageQueueM = xQueueCreate(2, 8*sizeof(char));
    
    motorsData.leftEncoder_Conv = motorsData.rightEncoder_Conv = 8;
    set_dist(60,60); 
    
    set_speed(WALK,WALK);
    move_stop();
    rightEncoder = leftEncoder = 0;
       
    
    //PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
    //PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);    
    
    //start timer
    DRV_TMR1_Start();
    DRV_TMR2_Start();
    DRV_TMR3_Start();    
}


//set the distance for the rover to stop after in cm
void set_dist(int leftDist, int rightDist) {
    motorsData.leftDist = (leftDist * motorsData.leftEncoder_Conv);
    motorsData.rightDist = (rightDist * motorsData.rightEncoder_Conv);
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
    rightEncoder = leftEncoder = 0;
    //left motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, motorsData.leftSpeed + motorsData.leftSpeed_Offset);    
    //right motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, motorsData.rightSpeed + motorsData.rightSpeed_Offset);
}


// Leader Rover
//turn the rover to the right
void turn_left() {
    set_dist(200,200);
    rightEncoder = leftEncoder = 0;
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK + motorsData.rightSpeed_Offset); 
    
    //stop left motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
}

//spin the rover clockwise
void spin_left() {
    set_dist(22,22);
    rightEncoder = leftEncoder = 0;
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK + motorsData.rightSpeed_Offset); 
    
    //left motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK + motorsData.leftSpeed_Offset);  
}

//turn the rover to the left
void turn_right() {
    set_dist(200,200);
    rightEncoder = leftEncoder = 0;
    //stop right motor
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    
    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK + motorsData.leftSpeed_Offset);  
}

//spin the rover counterclockwise
void spin_right() {
    set_dist(22,22);
    rightEncoder = leftEncoder = 0;
    //right motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, WALK + motorsData.rightSpeed_Offset); 
    
    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, WALK + motorsData.leftSpeed_Offset);  
}

//function to move forwards
void move_forward(int leftDist, int rightDist) {
    set_speed(RUN, RUN);
    set_dist (leftDist, rightDist);
    motorsData.moveState = MOVE;
    rightEncoder = leftEncoder = 0;
    //right motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);

    //left motor
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);

    move_start();
}

//function to move backwards
void move_backward() {
    rightEncoder = leftEncoder = 0;
    //right motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    
    //left motor
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);

    move_start();
}

int get_distance(SIDE side) {
    switch (side) {
        case LEFT:
            return leftEncoder * motorsData.leftEncoder_Conv;
        case RIGHT:
            return rightEncoder * motorsData.rightEncoder_Conv;
        default:
            return 0;
    }
}

MOVE_STATE getMoveState() {
    return motorsData.moveState;
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
            motorsData.moveState = WAIT;
            break;
        }

        case MOTORS_STATE_SERVICE_TASKS:
        {
            switch (motorsData.moveState) {
                case INIT:
                    move_stop();
                    set_speed(RUN, RUN);
                    set_dist(50,50);
                    move_start();
                    motorsData.moveState = TEST;
                    motorsData.testState = MOVE1;
                case TEST:
                    switch (motorsData.testState) {
                        case MOVE1:
                            if ((rightEncoder > motorsData.rightDist) && (leftEncoder > motorsData.leftDist)) {
                                move_stop();
                                spin_left();
                                motorsData.testState = SPIN1;
                            }
                            break;
                        case SPIN1:
                            if (rightEncoder > motorsData.rightDist) {
                                move_stop();
                                set_speed(RUN, RUN);
                                set_dist(50,50);
                                move_start();
                                motorsData.testState = MOVE2;
                            }
                            break;
                        case MOVE2:
                            if ((rightEncoder > motorsData.rightDist) && (leftEncoder > motorsData.leftDist)) {
                                move_stop();
                                spin_right();
                                motorsData.testState = SPIN2;
                            }
                            break;
                        case SPIN2:
                            if (leftEncoder > motorsData.leftDist) {
                                move_stop();
                                motorsData.testState = MOVE1;
                                motorsData.moveState = WAIT;
                            }
                            break;
                    }
                    break;                    
                case WAIT:
                    if (uxQueueMessagesWaiting(MessageQueueM)) {
                        move_stop();
                        int temp[4], i; //temps for set and get values, iterator
                        xQueueReceive(MessageQueueM, motorsData.motor_msg, portMAX_DELAY);
                        if ((motorsData.motor_msg[0] & 0x03) != MOTOR_THREAD_ID) {
                            break;
                        }
                        if (motorsData.motor_msg[0] & 0x20){ //if set command
                            for (i = 0; i < 3; i++) {
                                if (motorsData.motor_msg[i+1] & 0x80) {
                                    temp[i] = 0xFFFFFF00 | motorsData.motor_msg[i+1];
                                } else {
                                    temp[i] = 0x00000000 | motorsData.motor_msg[i+1];
                                }
                            }
                            motorsData.leftEncoder_Conv += temp[0];
                            motorsData.rightEncoder_Conv += temp[1];
                            motorsData.leftSpeed_Offset += temp[2];
                            motorsData.rightSpeed_Offset += temp[3];
                            //now test
                            
                            motorsData.moveState = TEST;
                            set_dist(50,50);
                            set_speed(RUN, RUN);
                            move_start();
                            
                        } else if (motorsData.motor_msg[0] & 0x10) {    //else if get command
                            if (motorsData.motor_msg[1] & 0x01) { //check if getting encoders
                                motorsData.motor_msg[1] = (char)(motorsData.leftEncoder_Conv >> 8);
                                motorsData.motor_msg[2] = (char)(motorsData.leftEncoder_Conv);
                                motorsData.motor_msg[3] = (char)(motorsData.rightEncoder_Conv >> 8);
                                motorsData.motor_msg[4] = (char)(motorsData.rightEncoder_Conv);
                            } else if (motorsData.motor_msg[1] & 0x02){ //else sending speeds
                                motorsData.motor_msg[1] = (char)(motorsData.leftSpeed_Offset >> 8);
                                motorsData.motor_msg[2] = (char)(motorsData.leftSpeed_Offset);
                                motorsData.motor_msg[3] = (char)(motorsData.rightSpeed_Offset >> 8);
                                motorsData.motor_msg[4] = (char)(motorsData.rightSpeed_Offset);
                            } else {
                                break;
                            }
                            motorsData.motor_msg[0] = INT_MSG | (0x01 << 4) | (MOTOR_THREAD_ID << 2) | 0x00 ;
                            //info                   int         get           SRC                   DST
                            xQueueSend(MessageQueueWout, motorsData.motor_msg, pdFAIL);
                        }
                    }
                    break;
                case MOVE:
                    if (rightEncoder > motorsData.rightDist && leftEncoder > motorsData.leftDist) {
                        move_stop();
                        rightEncoder = leftEncoder = 0;
                        motorsData.moveState = WAIT;
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
