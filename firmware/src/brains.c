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

// Main algorithm function what will be called in the lead rover's loop
void algorithm() {
	switch (brainsData.algData.states)
	{
        case FINDING_PATH:
        {
            //legB here is the distance traveled from the most recent start point
            brainsData.algData.legB = getCurrentTravelDistance();// = sqrt(sq(brainsData.algData.position.x - brainsData.algData.origin.x) + sq(brainsData.algData.position.y - brainsData.algData.origin.y));

            brainsData.algData.distToGo = getLegC(brainsData.algData.legA, brainsData.algData.legB, brainsData.algData.angToTurn);
            float temp = fmod(asin(brainsData.algData.legA * (sin(brainsData.algData.angC) / brainsData.algData.distToGo)), MIN_ROTATION_INCREMENT);
            brainsData.algData.angToTurn =  temp + asin(brainsData.algData.legA * (sin(brainsData.algData.angC) / brainsData.algData.distToGo));

            // determine when if its faster to rotate right or faster to rotate left 
            if (brainsData.algData.currRotAng < brainsData.algData.angToTurn) {
                    if (fabs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = true;
                    } else {
                        brainsData.algData.turnRight = false;
                    }
                } else {
                    if (fabs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = false;
                    } else {
                        brainsData.algData.turnRight = true;
                    }
                }
            // switch brain state to send message mode in order to tell muscles to rotate and move
            brainsData.state = BRAINS_STATE_TRANSMIT_FINDING_PATH;
            break;
        }
        case OBSTACLE_ENCOUNTERED:
        {
            brainsData.algData.numObjsCollide++;

            if (brainsData.algData.numObjsCollide == 1) {	

                brainsData.algData.legA = brainsData.algData.distToGo - getCurrentTravelDistance();

                turnHandler();
            }
            else if (brainsData.algData.numObjsCollide >= 2) {
                brainsData.algData.legB = getCurrentTravelDistance();
                brainsData.algData.legA = getLegC(brainsData.algData.legA, brainsData.algData.legB, brainsData.algData.angC);
                turnHandler();
            }
            else {
                // This Should never happen 
                break;
            }

            brainsData.algData.currRotAng = fmod(brainsData.algData.currRotAng,360);
            brainsData.algData.angToTurn =	fmod(brainsData.algData.angToTurn,360);
            // which way to rotate?

            int travelDist = getCurrentTravelDistance();

            if (travelDist < 1) { // this is so that if the rover immediately encountering another obstacle it doesn't turn a different direction. 
                brainsData.algData.turnRight = brainsData.algData.turnRight;
            } else {
                if (brainsData.algData.currRotAng < brainsData.algData.angToTurn) {
                    if (fabs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = true;
                    } else {
                        brainsData.algData.turnRight = false;
                    }
                } else {
                    if (fabs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = false;
                    } else {
                        brainsData.algData.turnRight = true;
                    }
                }
            }
            // Send message to motors that they need to start rotating and then move after
            brainsData.state = BRAINS_STATE_TRANSMIT_OBSTACLE_ENCOUNTERED;
            break;
        }
        case FOUND_LINE:
        {
            brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            break;
        }
        case FOUND_ENDPOINT:
        {
            brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            break;
        }
        default:
        {
            break;
        }
	}
}

// TODO implement distance traveled so far function
int getCurrentTravelDistance() {
    return brainsData.algData.currDistTrav;
}
// TODO implement set new local origin method 
/*void setLocalOrigin() {

}*/
// responsible for setting the turn at which rover will go when it encounters an obstacle
void turnHandler() {
   
	int leftObjs = 0; // number of obstacles on left  (-60)
	int rightObjs = 0; // number of obstacles on right (+60)
	
    // LeftAng and rightAng are the angles we should glance in to see which side is better 
    //float leftAng = (brainsData.algData.currRotAng - 90) - fmod((brainsData.algData.currRotAng - 90),MIN_ROTATION_INCREMENT);
	//float rightAng = fmod((brainsData.algData.currRotAng + 90),MIN_ROTATION_INCREMENT) + (brainsData.algData.currRotAng + 90);

	// TODO scan left and right side of rover to determine which has the least amount of obstacles, distances of 40, 60, 80
	
	
	// "weight" adjustment based upon distance of possible path's endpoint to final endpoint
	int leftMag;
	int tempAng = ROTATION_AMOUNT;
	if (brainsData.algData.numObjsCollide >= 2) {
		tempAng = 180 - asin(brainsData.algData.legA * (sin(brainsData.algData.angC) / brainsData.algData.legB)) - ROTATION_AMOUNT;
	}
    
	leftMag = getLegC(brainsData.algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);
	int rightMag;
	tempAng = ROTATION_AMOUNT;
	if (brainsData.algData.numObjsCollide >= 2) {
		tempAng = 180 - asin(brainsData.algData.legB * (sin(brainsData.algData.angC) / brainsData.algData.legA)) + ROTATION_AMOUNT;
	}
    
	tempAng%=360;
	rightMag = getLegC(brainsData.algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);
    
	if (leftMag < rightMag) {
		leftObjs -= 3;
	} else if (rightMag < leftMag) {
		rightObjs -= 3;
	} else {
		// do nothing no weight change
	}

	if (leftObjs > rightObjs) { //turn left
		brainsData.algData.angToTurn = (brainsData.algData.currRotAng - ROTATION_AMOUNT) - fmod((brainsData.algData.currRotAng - ROTATION_AMOUNT), MIN_ROTATION_INCREMENT);
		if (brainsData.algData.angToTurn < 0) {
			brainsData.algData.angToTurn += 360;
		}
		brainsData.algData.angToTurn= fmod(brainsData.algData.angToTurn, 360);
		if (brainsData.algData.numObjsCollide == 1) {
			brainsData.algData.angC = ROTATION_AMOUNT;
		} else if (brainsData.algData.numObjsCollide >= 2) {
			brainsData.algData.angC = 180 - asin(brainsData.algData.legB * (sin(brainsData.algData.angC) / brainsData.algData.legA)) - ROTATION_AMOUNT;
		}

		brainsData.algData.side = 1;
	} else {
		// turn right
		brainsData.algData.angToTurn = fmod((brainsData.algData.currRotAng + ROTATION_AMOUNT), MIN_ROTATION_INCREMENT) + (brainsData.algData.currRotAng + ROTATION_AMOUNT);
		if (brainsData.algData.angToTurn < 0) {
			brainsData.algData.angToTurn += 360;
		}
		brainsData.algData.angToTurn = fmod(brainsData.algData.angToTurn,360);
		if (brainsData.algData.numObjsCollide == 1) {
			brainsData.algData.angC = ROTATION_AMOUNT;
		} else if (brainsData.algData.numObjsCollide >= 2) {
			brainsData.algData.angC = 180 - asin(brainsData.algData.legB * (sin(brainsData.algData.angC) / brainsData.algData.legA)) + ROTATION_AMOUNT;
		}
		brainsData.algData.side = -1;
	}
}
// calculation function to get a third leg (leg C) when doing law of cosines 
int getLegC() {
	return sqrt(pow(brainsData.algData.legA,2) + pow(brainsData.algData.legB,2) - (2 * brainsData.algData.legA * brainsData.algData.legB * cos(brainsData.algData.angC)));
}
// algorithm initializer function 
void algoInit(){
	brainsData.algData.legA = 0;
	brainsData.algData.legB = 0;
	brainsData.algData.angC = 0;
	brainsData.algData.distToGo = 0;
	brainsData.algData.angToTurn = 0;
	brainsData.algData.numObjsCollide = 0;
	brainsData.algData.turnRight = true;
	brainsData.algData.states = FINDING_PATH;
	brainsData.algData.currRotAng = 0;
    brainsData.algData.currDistTrav = 0;
}

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
    brainsData.notifyPicNum = 0;
    algoInit();
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
            
                brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_INIT;
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_INIT:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                if(brainsData.rx_data[0] == 0x83){ // from raspi
#if DEBUG_START == true
                    brainsData.algData.distToGo = DEBUG_START_DIST;
                    brainsData.algData.currRotAng = START_CURRENT_ANG_ROT;
                    brainsData.algData.angToTurn = (brainsData.algData.currRotAng + DEBUG_START_DEGREES_TO_TURN)%360;
#else
                    brainsData.algData.distToGo = (int)brainsData.rx_data[3];
                    brainsData.algData.currRotAng = START_CURRENT_ANG_ROT;
                    brainsData.algData.angToTurn += (brainsData.algData.currRotAng + ((brainsData.rx_data[1]<<8)|brainsData.rx_data[2]))%360;
#endif                    
                    brainsData.state = BRAINS_STATE_TRANSMIT_FINDING_PATH;                       
                }  
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_ROTATE_AND_MOVE:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                // message from sensors that line was encountered 
                if( (brainsData.rx_data[0] == 0x93) && (brainsData.rx_data[1] > 0x00)){
					// end condition reached 
                     brainsData.algData.states = FOUND_LINE;
                    brainsData.state = BRAINS_STATE_WORK_ON_DATA;
                }
                // message from sensors that obstacle was encountered 
                else if((brainsData.rx_data[0] == 0x93) && 
                        ( (brainsData.rx_data[2] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[3] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[4] <= OBJECT_DETECT_DISTANCE) ) ){
                    brainsData.state = BRAINS_STATE_TRANSMIT_MOTOR_STOP;
                }
                // message from motors of the distance traveled and the angle rotated
                else if(brainsData.rx_data[0] == 0x8b){
                    // this means that we have reached the end
                    brainsData.algData.states = FOUND_ENDPOINT;
                    brainsData.state = BRAINS_STATE_WORK_ON_DATA;
                }           
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_ROTATE_AND_MOVE:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                // message from sensors that line was encountered 
                if( (brainsData.rx_data[0] == 0x93) && (brainsData.rx_data[1] > 0x00)){
                     brainsData.algData.states = FOUND_LINE;
                    brainsData.state = BRAINS_STATE_WORK_ON_DATA;
                }
                // message from sensors that obstacle was encountered 
                else if((brainsData.rx_data[0] == 0x93) && 
                        ( (brainsData.rx_data[2] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[3] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[4] <= OBJECT_DETECT_DISTANCE) ) ){
                    brainsData.state = BRAINS_STATE_TRANSMIT_MOTOR_STOP;
                }
                // message from motors that they navigated past the obstacle
                else if(brainsData.rx_data[0] == 0x8b){
                    // this means that we have reached the end
                    brainsData.algData.states = FINDING_PATH;
                    brainsData.state = BRAINS_STATE_WORK_ON_DATA;

                }        
            }
            break;
        }
        // Message from motors of how far they had gotten until they stopped due to obstacle
        case BRAINS_STATE_RECEIVE_MESSAGE_WAIT:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                if(brainsData.rx_data[0] == 0x8b){
                    brainsData.algData.currDistTrav = (int)brainsData.rx_data[3];
                    brainsData.algData.currRotAng += ((brainsData.rx_data[1]<<8)|brainsData.rx_data[2]);
                    
                    // going to algorithm state obstacle encountered 
                    brainsData.algData.states = OBSTACLE_ENCOUNTERED;
                    brainsData.state = BRAINS_STATE_WORK_ON_DATA;
                }
            }
            break;
        }
        case BRAINS_STATE_WORK_ON_DATA:
        {
            algorithm();
            break;
        }
        case BRAINS_STATE_TRANSMIT_FINDING_PATH:
        {
            //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            brainsData.tx_data[0] =  0x99; // sending message to pic 1 muscles
            int16_t degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn);
            if(degreesToTurn > 180){
                degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn + 360);
            }
            degreesToTurn = (brainsData.algData.turnRight)? degreesToTurn : -1 * degreesToTurn;      
            brainsData.tx_data[2] =  0xFF & degreesToTurn; // lower 8 bits for number of degrees rover needs to turn 
            brainsData.tx_data[1] =  0xFF & (degreesToTurn >> 8); // upper 8 bits for number of degrees rover needs to turn
            brainsData.tx_data[3] = (uint8_t)brainsData.algData.distToGo; // distance to travel 
            brainsData.tx_data[4] = 0x00; // unused data 2
            brainsData.tx_data[5] = 0x00; // unused data 2
            brainsData.tx_data[6] = 0x00; // counter 
            brainsData.tx_data[7] = 0x00;
            
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
            dbgOutputVal((uint8_t)brainsData.notifyPicNum);
            //StoreMessage(motorsData.tx_data);
            brainsData.state = BRAINS_STATE_WAIT_ACK_FINDING_PATH;
            break;
        }
        case BRAINS_STATE_TRANSMIT_END_CONDITION:
        {
            switch(brainsData.notifyPicNum)
            {
                case 0:
                {
                    brainsData.tx_data[0] =  0x98; // sending message to rpi
                    break;
                }
                case 1: 
                {
                     brainsData.tx_data[0] =  0x99; // sending to pic 1 muscles 
                    break;
                }
                case 2:
                {
                     brainsData.tx_data[0] =  0x9a; // sending to pic 2 sensors
                    break;
                }
                case 4:
                {
                    brainsData.tx_data[0] =  0x9c; // sending to pic 4 follower
                    break;
                }
                default: 
                {
                    break;
                }
            }
            if(brainsData.notifyPicNum < 4)
            {
                brainsData.tx_data[1] = 0x0F;
                brainsData.tx_data[2] = 0x0F;
                brainsData.tx_data[3] = 0x0F;
                brainsData.tx_data[4] = 0x0F;
                brainsData.tx_data[5] = 0x0F;
                brainsData.tx_data[6] = 0x0F;
                brainsData.tx_data[7] = 0x0F;
            }
            else
            {
                brainsData.tx_data[1] = 0xF0;
                brainsData.tx_data[2] = 0xF0;
                brainsData.tx_data[3] = 0xF0;
                brainsData.tx_data[4] = 0xF0;
                brainsData.tx_data[5] = 0xF0;
                brainsData.tx_data[6] = 0xF0;
                brainsData.tx_data[7] = 0xF0;
            }
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
            dbgOutputVal((uint8_t)brainsData.notifyPicNum);
            //StoreMessage(motorsData.tx_data);
            brainsData.state = BRAINS_STATE_WAIT_ACK_END_CONDITION;
            break;
        }
        case BRAINS_STATE_TRANSMIT_OBSTACLE_ENCOUNTERED:
        {
            brainsData.tx_data[0] =  0x99; // sending message to pic 1 muscles
            int16_t degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn);
            if(degreesToTurn > 180){
                degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn + 360);
            }
            degreesToTurn = (brainsData.algData.turnRight)? degreesToTurn : -1 * degreesToTurn;      
            brainsData.tx_data[2] =  0xFF & degreesToTurn; // lower 8 bits for number of degrees rover needs to turn 
            brainsData.tx_data[1] =  0xFF & (degreesToTurn >> 8); // upper 8 bits for number of degrees rover needs to turn
            brainsData.tx_data[3] = (uint8_t)OBSACLE_AVOIDANCE_DISTANCE; // distance to travel 
            brainsData.tx_data[4] = 0x00; // unused data 2
            brainsData.tx_data[5] = 0x00; // unused data 2
            brainsData.tx_data[6] = 0x00; // counter 
            brainsData.tx_data[7] = 0x00;
            
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
            dbgOutputVal((uint8_t)brainsData.notifyPicNum);
            //StoreMessage(motorsData.tx_data);
            brainsData.state = BRAINS_STATE_WAIT_ACK_OBSTACLE_ENCOUNTERED;
            break;
        }
        case BRAINS_STATE_TRANSMIT_MOTOR_STOP:
        {
            // this means that sensors found an obstacle and we need to send motors a stop
            brainsData.tx_data[0] =  0x99;
            brainsData.tx_data[1] =  0xFF; 
            brainsData.tx_data[2] =  0xFF; 
            brainsData.tx_data[3] =  0xFF; 
            brainsData.tx_data[4] =  0xFF; 
            brainsData.tx_data[5] =  0xFF; 
            brainsData.tx_data[6] =  0xFF; 
            brainsData.tx_data[7] =  0xFF;
			
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
            dbgOutputVal((uint8_t)brainsData.notifyPicNum);
            //StoreMessage(motorsData.tx_data);
            brainsData.state = BRAINS_STATE_WAIT_ACK_MOTOR_STOP;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_FINDING_PATH:
        {
            while(wait_on_ack){
                
            }
            //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_ROTATE_AND_MOVE;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_OBSTACLE_ENCOUNTERED:
        {
            while(wait_on_ack){
                
            }
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_ROTATE_AND_MOVE;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_END_CONDITION:
        {
            while(wait_on_ack){
                PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            }
            //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            if(brainsData.notifyPicNum <= 0){
                brainsData.notifyPicNum = 1;
                brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            }
            else if(brainsData.notifyPicNum == 1){
                brainsData.notifyPicNum = 2;
                brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            }
            else if(brainsData.notifyPicNum == 2){
                brainsData.notifyPicNum = 4;
                brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            }
            else if(brainsData.notifyPicNum >= 4){
                brainsData.state = BRAINS_STATE_DONE;
            }
            else{
               brainsData.state = BRAINS_STATE_DONE;
            }
            break;
        }
        case BRAINS_STATE_WAIT_ACK_MOTOR_STOP:
        {
            while(wait_on_ack){
                
            }
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_WAIT;  
            break;
        }
        case BRAINS_STATE_DONE:
        {
            // DO nothing
            //dbgOutputVal(0xAA);
            //dbgOutputVal((uint8_t)brainsData.notifyPicNum);
            break;
            
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
