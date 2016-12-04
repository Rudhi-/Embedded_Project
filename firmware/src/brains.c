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

// TODO implement distance traveled so far function
int16_t getCurrentTravelDistance() {
    return brainsData.algData.currDistTrav;
}

// responsible for setting the turn at which rover will go when it encounters an obstacle
void turnHandler() {
   
	int16_t leftObjs = 0; // number of obstacles on left  (-60)
	int16_t rightObjs = 0; // number of obstacles on right (+60)
		
	// "weight" adjustment based upon distance of possible path's endpoint to final endpoint
	int16_t leftMag = 0;
	int16_t tempAng = ROTATION_AMOUNT;
	if (brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE){//(brainsData.algData.numObjsCollide >= 2) {
		tempAng = 180 - (180/PI)*asin(brainsData.algData.legA * (sin((PI/180)*brainsData.algData.angC) / brainsData.algData.legB)) - ROTATION_AMOUNT;
        leftMag = getLegC(brainsData.algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);
	}
	int16_t rightMag = 0;
	tempAng = ROTATION_AMOUNT;
	if (brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE){//(brainsData.algData.numObjsCollide >= 2) {
		tempAng = 180 - (180/PI)*asin(brainsData.algData.legB * (sin((PI/180)*brainsData.algData.angC) / brainsData.algData.legA)) + ROTATION_AMOUNT;
        tempAng%=360;
        rightMag = getLegC(brainsData.algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);
	}
    
	if (leftMag < rightMag) {
		leftObjs -= 3;
	} else if (rightMag < leftMag) {
		rightObjs -= 3;
	} else {
		// do nothing no weight change
	}

	if (leftObjs > rightObjs) { //turn left
		brainsData.algData.angToTurn = (brainsData.algData.currRotAng - ROTATION_AMOUNT);
		if (brainsData.algData.angToTurn < 0) {
			brainsData.algData.angToTurn += 360;
		}
		brainsData.algData.angToTurn= (brainsData.algData.angToTurn% 360);
		if (brainsData.algData.numObjsCollide == 1) {
			brainsData.algData.angC = ROTATION_AMOUNT;
		} else if (brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE){//(brainsData.algData.numObjsCollide >= 2) {
			brainsData.algData.angC = 180 - (180/PI)*asin(brainsData.algData.legB * (sin((PI/180)*brainsData.algData.angC) / brainsData.algData.legA)) - ROTATION_AMOUNT;
		}
		brainsData.algData.turnRight = false;
		
	} else {
		// turn right
		brainsData.algData.angToTurn = (brainsData.algData.currRotAng + ROTATION_AMOUNT);
		if (brainsData.algData.angToTurn < 0) {
			brainsData.algData.angToTurn += 360;
		}
		brainsData.algData.angToTurn = (brainsData.algData.angToTurn % 360);
		if (brainsData.algData.numObjsCollide == 1) {
			brainsData.algData.angC = ROTATION_AMOUNT;
		} else if (brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE){//(brainsData.algData.numObjsCollide >= 2) {
			brainsData.algData.angC = 180 - (180/PI)*asin(brainsData.algData.legB * (sin((PI/180)*brainsData.algData.angC) / brainsData.algData.legA)) + ROTATION_AMOUNT;
		}
		brainsData.algData.turnRight = true;
	}
}
// calculation function to get a third leg (leg C) when doing law of cosines 
int16_t getLegC() {
	return sqrt(pow(brainsData.algData.legA,2) + pow(brainsData.algData.legB,2) - (2 * brainsData.algData.legA * brainsData.algData.legB * cos( (PI/180)* brainsData.algData.angC)));
}
// algorithm initializer function 
void algoInit(){
	brainsData.algData.legA = 0;
	brainsData.algData.legB = 0;
	brainsData.algData.angC = 0;
    brainsData.algData.angA = 0;
	brainsData.algData.distToGo = 0;
	brainsData.algData.angToTurn = 0;
	brainsData.algData.numObjsCollide = 0;
	brainsData.algData.turnRight = true;
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
    brainsData.altStateObs = BRAINS_STATE_RECEIVE_MESSAGE_MOVE;
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
                
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                if(brainsData.rx_data[0] == 0x83){ // from raspi
#ifdef DEBUG_START //== true
                    brainsData.algData.distToGo = DEBUG_START_DIST;
                    brainsData.algData.currRotAng = START_CURRENT_ANG_ROT;
                    brainsData.algData.angToTurn = (brainsData.algData.currRotAng + DEBUG_START_DEGREES_TO_TURN)%360;
#else
                    brainsData.algData.distToGo = (int16_t)brainsData.rx_data[3];
                    brainsData.algData.currRotAng = START_CURRENT_ANG_ROT;
                    int16_t temp = ((brainsData.rx_data[1]<<8)|brainsData.rx_data[2]);
                    if (temp < 0){
                        brainsData.algData.turnRight = true;
                    }
                    else{
                        brainsData.algData.turnRight = false;
                    }
                    temp%=360;
                    
                    brainsData.algData.angToTurn = (brainsData.algData.currRotAng + temp);
                    
#endif                    
                    brainsData.state = BRAINS_STATE_TRANSMIT_FINDING_PATH;                       
                }  
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_ROTATE:
        {
            //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                // message from motors that angle was rotated
                if(brainsData.rx_data[0] == 0x8b){
                    brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_MOVE;
                }           
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_MOVE:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                // message from sensors that line was encountered 
                if( (brainsData.rx_data[0] == 0x93) && (brainsData.rx_data[1] > 0x00)){
					// end condition reached 
                     brainsData.state = BRAINS_STATE_ALGORITM_FOUND_LINE;
                }
                // message from sensors that obstacle was encountered 
                else if((brainsData.rx_data[0] == 0x93) && 
                        ( (brainsData.rx_data[2] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[3] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[4] <= OBJECT_DETECT_DISTANCE) ) ){
                    brainsData.state = BRAINS_STATE_TRANSMIT_MOTOR_STOP;
                    brainsData.altStateObs = BRAINS_STATE_RECEIVE_MESSAGE_MOVE;
                }
                // message from motors of the distance traveled and the angle rotated
                else if(brainsData.rx_data[0] == 0x8b){
					//PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                    // this means that we have reached the end
                    //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                    brainsData.state = BRAINS_STATE_ALGORITM_FOUND_ENDPOINT;
                }           
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_ROTATE:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                // message from motors that they finished rotating
                if(brainsData.rx_data[0] == 0x8b){
                    brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE;
                }        
            }
            break;
        }
        case BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE:
        {
            if (uxQueueMessagesWaiting(MessageQueueWin)){
                //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                
                // Receive the message from the receiver thread
                xQueueReceive(MessageQueueWin, brainsData.rx_data, portMAX_DELAY);
                
                // message from sensors that line was encountered 
                if( (brainsData.rx_data[0] == 0x93) && (brainsData.rx_data[1] > 0x00)){
                     brainsData.state = BRAINS_STATE_ALGORITM_FOUND_LINE;
                }
                // message from sensors that obstacle was encountered 
                else if((brainsData.rx_data[0] == 0x93) && 
                        ( (brainsData.rx_data[2] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[3] <= OBJECT_DETECT_DISTANCE) || 
                        (brainsData.rx_data[4] <= OBJECT_DETECT_DISTANCE) ) ){
                    brainsData.state = BRAINS_STATE_TRANSMIT_MOTOR_STOP;
                    brainsData.altStateObs = BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE;
                }
                // message from motors that they navigated past the obstacle
                else if(brainsData.rx_data[0] == 0x8b){
                    // this means that we have reached the end
                    brainsData.algData.currDistTrav = (int16_t)brainsData.rx_data[3];
                    brainsData.algData.currRotAng += -1*((int16_t)((brainsData.rx_data[1]<<8)|brainsData.rx_data[2]));
					brainsData.algData.angC = abs(((int16_t)((brainsData.rx_data[1]<<8)|brainsData.rx_data[2])));
                    brainsData.state = BRAINS_STATE_ALGORITM_FINDING_PATH;//FINDING_PATH;
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
                    brainsData.algData.currDistTrav = (int16_t)brainsData.rx_data[3];
                    brainsData.algData.currRotAng += -1*((int16_t)((brainsData.rx_data[1]<<8)|brainsData.rx_data[2]));
                    
                    // going to algorithm state obstacle encountered 
                    brainsData.state = BRAINS_STATE_ALGORITM_OBSTACLE_ENCOUNTERED;//OBSTACLE_ENCOUNTERED;
                }
            }
            break;
        }
        case BRAINS_STATE_ALGORITM_FINDING_PATH:
        {
            //legB here is the distance traveled from the most recent start point
            brainsData.algData.legB = getCurrentTravelDistance();// = sqrt(sq(brainsData.algData.position.x - brainsData.algData.origin.x) + sq(brainsData.algData.position.y - brainsData.algData.origin.y));

            brainsData.algData.distToGo = getLegC();
            int16_t temp = (180/PI)*asin(brainsData.algData.legA * (sin(brainsData.algData.angC*(PI/180)) / brainsData.algData.distToGo));
            brainsData.algData.angToTurn =  temp;
        
			brainsData.algData.angA =  (180/PI)*acos((pow(brainsData.algData.legB,2) + pow(brainsData.algData.distToGo,2)- pow(brainsData.algData.legA,2))/(2*brainsData.algData.legB*brainsData.algData.distToGo));
            brainsData.algData.angA=abs(brainsData.algData.angA);
            
            // determine when if its faster to rotate right or faster to rotate left 
            /*if (brainsData.algData.currRotAng < brainsData.algData.angToTurn) {
                    if (abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = false;
                    } else {
                        brainsData.algData.turnRight = true;
                    }
                } else {
                    if (abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                        brainsData.algData.turnRight = true;
                    } else {
                        brainsData.algData.turnRight = false;
                    }
                }
             */
               brainsData.algData.turnRight = !brainsData.algData.turnRight;
           
            // switch brain state to send message mode in order to tell muscles to rotate and move
            brainsData.state = BRAINS_STATE_TRANSMIT_FINDING_PATH;
            break;
        }
        case BRAINS_STATE_ALGORITM_OBSTACLE_ENCOUNTERED:
        {
            brainsData.algData.numObjsCollide++;

            if (brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_MOVE){	

                brainsData.algData.legA = brainsData.algData.distToGo - getCurrentTravelDistance();

                turnHandler();
            }
            else if(brainsData.altStateObs == BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE){//if (brainsData.algData.numObjsCollide >= 2) {
                brainsData.algData.legB = getCurrentTravelDistance();
                brainsData.algData.legA = getLegC();
                turnHandler();
            }
            else {
                // This Should never happen 
                break;
            }

            brainsData.algData.currRotAng = (brainsData.algData.currRotAng%360);
            brainsData.algData.angToTurn =	(brainsData.algData.angToTurn%360);
            // which way to rotate?

            if (brainsData.algData.currRotAng < brainsData.algData.angToTurn) {
                if (abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                    brainsData.algData.turnRight = true;
                } else {
                    brainsData.algData.turnRight = false;
                }
            } else {
                if (abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn) < 180) {
                    brainsData.algData.turnRight = false;
                } else {
                    brainsData.algData.turnRight = true;
                }
            }
            
            // Send message to motors that they need to start rotating and then move after
            brainsData.state = BRAINS_STATE_TRANSMIT_OBSTACLE_ENCOUNTERED;
            break;
        }
        case BRAINS_STATE_ALGORITM_FOUND_LINE:
        {
            brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            break;
        }
        case BRAINS_STATE_ALGORITM_FOUND_ENDPOINT:
        {
            brainsData.state = BRAINS_STATE_TRANSMIT_END_CONDITION;
            break;
        }
        case BRAINS_STATE_TRANSMIT_FINDING_PATH:
        {
            brainsData.tx_data[0] =  0x99; // sending message to pic 1 muscles
            int16_t degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn);
            if(degreesToTurn > 180){
                degreesToTurn = abs(brainsData.algData.currRotAng - brainsData.algData.angToTurn + 360);
            }
            
            if(brainsData.algData.numObjsCollide >= 1){
                degreesToTurn = abs(180 - brainsData.algData.angA);
                if(degreesToTurn > 10){
                    // degree offset
                    degreesToTurn= abs(degreesToTurn+15);
                }
            }
            
            degreesToTurn = (!brainsData.algData.turnRight)? degreesToTurn : -1 * degreesToTurn;      
            
            brainsData.tx_data[1] =  0xFF & (degreesToTurn >> 8); // upper 8 bits for number of degrees rover needs to turn
            brainsData.tx_data[2] =  0xFF & degreesToTurn; // lower 8 bits for number of degrees rover needs to turn 
            brainsData.tx_data[3] = (uint8_t)brainsData.algData.distToGo; // distance to travel 
            brainsData.tx_data[4] = 0xFF & (brainsData.algData.angC >> 8); // unused data 2
            brainsData.tx_data[5] = 0xFF & brainsData.algData.angC; // unused data 2
            brainsData.tx_data[6] = 0x00; // counter 
            brainsData.tx_data[7] = 0x00;
            
            SendMessageAck(brainsData.tx_data);
            
            brainsData.state = BRAINS_STATE_WAIT_ACK_FINDING_PATH;
            break;
        }
        case BRAINS_STATE_TRANSMIT_END_CONDITION:
        {
            brainsData.tx_data[0] =  0x98; // sending message to rpi
            // stop command 
            brainsData.tx_data[1] = 0x0F;
            brainsData.tx_data[2] = 0x0F;
            brainsData.tx_data[3] = 0x0F;
            brainsData.tx_data[4] = 0x0F;
            brainsData.tx_data[5] = 0x0F;
            brainsData.tx_data[6] = 0x0F;
            brainsData.tx_data[7] = 0x0F;
            // actually sending message out
            SendMessageAck(brainsData.tx_data);
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
            degreesToTurn = ROTATION_AMOUNT;
            
            degreesToTurn = (!brainsData.algData.turnRight)? degreesToTurn : -1 * degreesToTurn;      
            brainsData.tx_data[2] =  0xFF & degreesToTurn; // lower 8 bits for number of degrees rover needs to turn 
            brainsData.tx_data[1] =  0xFF & (degreesToTurn >> 8); // upper 8 bits for number of degrees rover needs to turn
            brainsData.tx_data[3] = (uint8_t)OBSACLE_AVOIDANCE_DISTANCE; // distance to travel 
            brainsData.tx_data[4] = 0x00; // unused data 2
            brainsData.tx_data[5] = 0x00; // unused data 2
            brainsData.tx_data[6] = 0x00; // counter 
            brainsData.tx_data[7] = 0x00;
            
            //SendMessageAck(brainsData.tx_data);
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
           
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
			
            //SendMessageAck(brainsData.tx_data);
            wait_on_ack = true;
            xQueueSend( MessageQueueWout, brainsData.tx_data, pdFAIL );
            
            brainsData.state = BRAINS_STATE_WAIT_ACK_MOTOR_STOP;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_FINDING_PATH:
        {
            if (!wait_on_ack){
             // wait blocking   
                brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_ROTATE;
            }            
            break;
        }
        case BRAINS_STATE_WAIT_ACK_OBSTACLE_ENCOUNTERED:
        {
            while(wait_on_ack);
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_ROTATE;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_END_CONDITION:
        {
            while(wait_on_ack){
                //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            }
            brainsData.state = BRAINS_STATE_DONE;
            break;
        }
        case BRAINS_STATE_WAIT_ACK_MOTOR_STOP:
        {
            while(wait_on_ack);
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_WAIT;  
            break;
        }
        case BRAINS_STATE_DONE:
        {
            // resetting all data and going back to waiting for more commands from raspi
            algoInit();
            brainsData.state = BRAINS_STATE_RECEIVE_MESSAGE_INIT;
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
