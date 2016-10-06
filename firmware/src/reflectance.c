/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    reflectance.c

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

#include "reflectance.h"

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

REFLECTANCE_DATA reflectanceData;

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
    void REFLECTANCE_Initialize ( void )

  Remarks:
    See prototype in reflectance.h.
 */

void REFLECTANCE_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    reflectanceData.state = REFLECTANCE_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    /* init stuff, configure devices*/
}


/******************************************************************************
  Function:
    void REFLECTANCE_Tasks ( void )

  Remarks:
    See prototype in reflectance.h.
 */

void REFLECTANCE_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( reflectanceData.state )
    {
        /* Application's initial state. */
        case REFLECTANCE_STATE_INIT:
        {
            bool appInitialized = true;
            
            start_LED_ON = 0;
            start_LED_OFF = 0;
            start_LED_INPUT = 0;
        
            if (appInitialized)
            {
            
                reflectanceData.state = REFLECTANCE_STATE_LED_ON;
            }
            break;
        }

        case REFLECTANCE_STATE_LED_ON:
        {   
            if (start_LED_ON) {
                start_LED_ON = 0;
                
                /* set LEDs as outputs
                 * turn LEDs on
                 * start timer id 3, instance 2
                 * change state to REFLECTANCE_STATE_LED_OFF
                 *  */
                
                TRISBbits.TRISB8 = 0;
                TRISBbits.TRISB9 = 0;
                TRISBbits.TRISB10 = 0;
                TRISBbits.TRISB11 = 0;
                TRISBbits.TRISB12 = 0;
                TRISBbits.TRISB13 = 0;
                TRISBbits.TRISB14 = 0;
                TRISBbits.TRISB15 = 0;

                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_8, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_10, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15, 1);
                
                DRV_TMR2_CounterClear();
                DRV_TMR2_Start();
                reflectanceData.state = REFLECTANCE_STATE_LED_OFF;
            }
        }
        
        case REFLECTANCE_STATE_LED_OFF:
        {
            if (start_LED_OFF) {
                start_LED_OFF = 0;
                /* disable timer
                 * turn LEDs off
                 * set LEDs as inputs
                 * start timer id 4, instance 3
                 * change state to REFLECTANCE_STATE_LED_INPUT
                 *  */
                
                DRV_TMR2_Stop();
                
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_8, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_9, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_10, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_11, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14, 0);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_15, 0);
                
                TRISBbits.TRISB8 = 1;
                TRISBbits.TRISB9 = 1;
                TRISBbits.TRISB10 = 1;
                TRISBbits.TRISB11 = 1;
                TRISBbits.TRISB12 = 1;
                TRISBbits.TRISB13 = 1;
                TRISBbits.TRISB14 = 1;
                TRISBbits.TRISB15 = 1;
                
                DRV_TMR3_CounterClear();
                DRV_TMR3_Start();
                reflectanceData.state = REFLECTANCE_STATE_LED_INPUT;
            }
        }
        
        case REFLECTANCE_STATE_LED_INPUT:
        {
            if (start_LED_INPUT) {
                start_LED_INPUT = 0;
                /* disable timer
                 * take reading of LEDs
                 * change state to REFLECTANCE_STATE_LED_ON
                 *  */
                
                // wait a little less for this one
                
                DRV_TMR3_Stop();
                
                char a;
                a = ((PORTBbits.RB15 << 7) +
                     (PORTBbits.RB14 << 6) +
                     (PORTBbits.RB13 << 5) +
                     (PORTBbits.RB12 << 4) +
                     (PORTBbits.RB11 << 3) +
                     (PORTBbits.RB10 << 2) +
                     (PORTBbits.RB9 << 1) +
                     (PORTBbits.RB8));
                
                a = a;
                //dbgOutputVal(a);
                
                reflectanceData.state = REFLECTANCE_STATE_LED_ON;
            }
        }
        
        /*
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 1);
            counter = 0;
            while(counter < 10000) {counter++;}
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1, 0);
            counter = 0;
            while(counter < 10000) {counter++;}
            break;
        */

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
