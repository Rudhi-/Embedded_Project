/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    brains.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _BRAINS_H
#define _BRAINS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "headers.h"
#include "debug.h"
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	BRAINS_STATE_INIT=0,
    BRAINS_STATE_RECEIVE_MESSAGE_INIT,
    BRAINS_STATE_RECEIVE_MESSAGE_ROTATE,
    BRAINS_STATE_RECEIVE_MESSAGE_MOVE,
    BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_ROTATE,
    BRAINS_STATE_RECEIVE_MESSAGE_OBSTACLE_ENCOUNTERED_MOVE,
    BRAINS_STATE_RECEIVE_MESSAGE_WAIT,
    BRAINS_STATE_ALGORITM_FINDING_PATH,
    BRAINS_STATE_ALGORITM_OBSTACLE_ENCOUNTERED,
    BRAINS_STATE_ALGORITM_FOUND_LINE,
    BRAINS_STATE_ALGORITM_FOUND_ENDPOINT,
    BRAINS_STATE_TRANSMIT_MOTOR_STOP,
    BRAINS_STATE_TRANSMIT_FINDING_PATH,
    BRAINS_STATE_TRANSMIT_END_CONDITION,
    BRAINS_STATE_TRANSMIT_OBSTACLE_ENCOUNTERED,
    BRAINS_STATE_WAIT_ACK_FINDING_PATH,
    BRAINS_STATE_WAIT_ACK_OBSTACLE_ENCOUNTERED,
    BRAINS_STATE_WAIT_ACK_END_CONDITION,
    BRAINS_STATE_WAIT_ACK_MOTOR_STOP,
    BRAINS_STATE_DONE

	/* TODO: Define states used by the application state machine. */

} BRAINS_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    BRAINS_STATES state; // the state of the brains task 
    uint8_t rx_data [8]; // receive buffer 
    uint8_t tx_data [8]; // transmit buffer 
    ALGORITHM_DATA algData; // struct containing algorithm's internal data 
    
} BRAINS_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void BRAINS_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    BRAINS_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void BRAINS_Initialize ( void );


/*******************************************************************************
  Function:
    void BRAINS_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    BRAINS_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void BRAINS_Tasks( void );


#endif /* _BRAINS_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

