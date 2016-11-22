/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ultrasonic.c

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

#include "ultrasonic.h"

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

ULTRASONIC_DATA ultrasonicData;

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
    void ULTRASONIC_Initialize ( void )

  Remarks:
    See prototype in ultrasonic.h.
 */

void ULTRASONIC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ultrasonicData.state = ULTRASONIC_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    /* init stuff, configure devices*/
}


/******************************************************************************
  Function:
    void ULTRASONIC_Tasks ( void )

  Remarks:
    See prototype in ultrasonic.h.
 */

void ULTRASONIC_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( ultrasonicData.state )
    {
        /* Application's initial state. */
        case ULTRASONIC_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (appInitialized)
            {
                ultrasonicData.state = ULTRASONIC_STATE_SERVICE_TASKS;
            }
            
            ultrasonicData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE); 
            if (DRV_HANDLE_INVALID == ultrasonicData.i2c_handle)
            {
                PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            }
            
            
            uint8_t range_command = 0x51;
            
            ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x08, &range_command, sizeof(range_command), NULL);
            while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));
            
            ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x10, &range_command, sizeof(range_command), NULL);
            while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));
            
            ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x20, &range_command, sizeof(range_command), NULL);
            while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));
            
            DRV_I2C_Close(ultrasonicData.i2c_handle);
            
            break;
        }

        case ULTRASONIC_STATE_SERVICE_TASKS:
        {
            int16_t j;
            
            if (start_ultrasonic) {
                start_ultrasonic = 0;
                ultrasonic_finished = 0;
                
                ultrasonicData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE); 
                if (DRV_HANDLE_INVALID == ultrasonicData.i2c_handle)
                {
                    PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                }
                
                uint8_t range_command = 0x51;
                uint8_t range_0[2];
                uint8_t range_1[2];
                uint8_t range_2[2];
                
                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }
                ultrasonicData.rxbufferhandle = DRV_I2C_Receive(ultrasonicData.i2c_handle, 0x08, range_0, 2, NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.rxbufferhandle ));
                
                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }
                ultrasonicData.rxbufferhandle = DRV_I2C_Receive(ultrasonicData.i2c_handle, 0x10, range_1, 2, NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.rxbufferhandle ));

                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }
                ultrasonicData.rxbufferhandle = DRV_I2C_Receive(ultrasonicData.i2c_handle, 0x20, range_2, 2, NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.rxbufferhandle ));

                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }                
                ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x08, &range_command, sizeof(range_command), NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));

                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }
                ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x10, &range_command, sizeof(range_command), NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));

                for (j = 0; j < 10000; j++)
                {
                    j = j + 1 - 1;
                }
                ultrasonicData.txbufferhandle = DRV_I2C_Transmit(ultrasonicData.i2c_handle, 0x20, &range_command, sizeof(range_command), NULL);
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( ultrasonicData.i2c_handle, ultrasonicData.txbufferhandle ));
                
                
                //dbgOutputVal(range_0[1]);                
                //dbgOutputVal(range_1[1]); 
                //dbgOutputVal(range_2[1]);
                
                DRV_I2C_Close(ultrasonicData.i2c_handle);
                
                //if(range[0] != 0) range[1] = 254;
                //dbgOutputVal(range[1]);
                
                
                packet_tx_data[2] = range_0[1];
                packet_tx_data[3] = range_1[1];
                packet_tx_data[4] = range_2[1];
                
                ultrasonic_finished = 1;
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
