/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    magnetometer.c

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

#include "magnetometer.h"
#include "header.h"
#include "math.h"

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

MAGNETOMETER_DATA magnetometerData;

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
    void MAGNETOMETER_Initialize ( void )

  Remarks:
    See prototype in magnetometer.h.
 */
uint8_t txBuffer_1[] = "\x0""\x7C";
uint8_t txBuffer_2[] = "\x1""\x20";
uint8_t txBuffer_3[] = "\x2""\x00";

uint8_t txBuffer[] = "\x3";

uint8_t rx_buffer[20];

void GetMagnetometerAq()
{
    magnetometerData.state = MAGNETOMETER_STATE_AQUIRE;
}
void GetMagnetometerData()
{
    magnetometerData.state = MAGNETOMETER_STATE_SERVICE_TASKS;
}

void StopGettingMagData()
{
    magnetometerData.state = MAGNETOMETER_STATE_WAIT;
}

void MAGNETOMETER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    magnetometerData.state = MAGNETOMETER_STATE_INIT;
    magnetometerData.getval = false;
    magnetometerData.rxbufferx = 0;
    magnetometerData.rxbuffery = 0;
    MessageQueueDin = xQueueCreate(2, 4*sizeof(uint8_t));
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}
uint8_t deviceAddressSlaveWrite = 0x3c;
uint8_t deviceAddressSlaveRead = 0x3d;


/******************************************************************************
  Function:
    void MAGNETOMETER_Tasks ( void )

  Remarks:
    See prototype in magnetometer.h.
 */


void MAGNETOMETER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( magnetometerData.state )
    {
        /* Application's initial state. */
        case MAGNETOMETER_STATE_INIT:
        {
            magnetometerData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE); 
            if (DRV_HANDLE_INVALID == magnetometerData.i2c_handle)
            {
                
            }
            
            DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_1[0], (sizeof(txBuffer_1)-1), NULL);
            DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_2[0], (sizeof(txBuffer_2)-1), NULL);
            DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_3[0], (sizeof(txBuffer_3)-1), NULL);
            DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, txBuffer, (sizeof(txBuffer)-1), NULL);
            DRV_I2C_Receive ( magnetometerData.i2c_handle, 0x3d, rx_buffer, 6, NULL);
            
            DRV_I2C_Close( magnetometerData.i2c_handle );
            
            bool appInitialized = true;
            xQueueReset (MessageQueueDin);
            
            //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            
            if (appInitialized)
            {
            
                magnetometerData.state = MAGNETOMETER_STATE_WAIT;
            }
            break;
        }
        
        case MAGNETOMETER_STATE_AQUIRE:
        {
            int i,j;
            
            for (i = 0; i < 10; i++)
            {
                for (j = 0; j < 1000000;j++)
                {
                    j = j + 1 -1;
                }
                magnetometerData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE);             
                //while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( magnetometerData.i2c_handle, magnetometerData.txbufferhandle ));
                //PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                DRV_I2C_Transmit ( magnetometerData.i2c_handle, 0x3c, txBuffer, (sizeof(txBuffer)-1), NULL);
                for (j = 0; j < 1000;j++)
                {
                    j = j + 1 -1;
                }
                //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                DRV_I2C_Receive ( magnetometerData.i2c_handle, 0x3d, rx_buffer, 6, NULL);

                DRV_I2C_Close( magnetometerData.i2c_handle );

                magnetometerData.rxbufferx = (rx_buffer[0] << 8) | rx_buffer[1];
                magnetometerData.rxbuffery = (rx_buffer[4] << 8) | rx_buffer[5];
                
                magnetometerData.rxbufferx = (magnetometerData.rxbufferx + 10) * .92;
                magnetometerData.rxbuffery = (magnetometerData.rxbuffery - 10) * .92;

                magnetometerData.bearing[0] = atan2(magnetometerData.rxbuffery, magnetometerData.rxbufferx) * 360/ (2*M_PI);

                if (magnetometerData.bearing[0] < 0)
                    magnetometerData.bearing[0] = 360 + magnetometerData.bearing[0];
                if (magnetometerData.rxbufferx == 0)
                {
                    i = i - 1;
                    //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                }
            }
            magnetometerData.state = MAGNETOMETER_STATE_SERVICE_TASKS;
            break;
        }

        case MAGNETOMETER_STATE_SERVICE_TASKS:
        {   
            int i;
            int j;
            for (i = 0; i < LOOP_SIZE; i++)
            {
                for (j = 0; j < 1000000;j++)
                {
                    j = j + 1 -1;
                }
                magnetometerData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE); 
                if (DRV_HANDLE_INVALID == magnetometerData.i2c_handle)
                {

                }               
                //while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( magnetometerData.i2c_handle, magnetometerData.txbufferhandle ));
                
                DRV_I2C_Transmit ( magnetometerData.i2c_handle, 0x3c, txBuffer, (sizeof(txBuffer)-1), NULL);
                for (j = 0; j < 1000;j++)
                {
                    j = j + 1 -1;
                }
                //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                DRV_I2C_Receive ( magnetometerData.i2c_handle, 0x3d, rx_buffer, 6, NULL);
                
                DRV_I2C_Close( magnetometerData.i2c_handle );

                magnetometerData.rxbufferx = (rx_buffer[0] << 8) | rx_buffer[1];
                magnetometerData.rxbuffery = (rx_buffer[4] << 8) | rx_buffer[5];
                
                magnetometerData.rxbufferx = (magnetometerData.rxbufferx + 15) * .92;
                magnetometerData.rxbuffery = (magnetometerData.rxbuffery + 85) * .92;

                magnetometerData.bearing[i] = atan2(magnetometerData.rxbuffery, magnetometerData.rxbufferx) * 360/ (2*M_PI);
                
                if (magnetometerData.bearing[i] < 0)
                    magnetometerData.bearing[i] = 360 + magnetometerData.bearing[i];
                if (magnetometerData.rxbufferx == 0)
                {
                    i = i - 1;
                    //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                }
                
            }
            magnetometerData.bearingAvg = magnetometerData.bearing[0];
            for (i = 1; i < LOOP_SIZE;i++)
            {
                magnetometerData.bearingAvg = magnetometerData.bearingAvg + magnetometerData.bearing[i];
            }
            magnetometerData.bearingAvg = magnetometerData.bearingAvg/LOOP_SIZE;
            magnetometerData.dx_data[0] = (magnetometerData.bearingAvg & 0xFF00) >> 8;
            magnetometerData.dx_data[1] = magnetometerData.bearingAvg & 0xFF;  
            dbgOutputVal(magnetometerData.bearingAvg/2);
            
            SendMessage(magnetometerData.bearingAvg/2,0);
            
            xQueueSend( MessageQueueDin, magnetometerData.dx_data, pdFAIL );
            
            magnetometerData.state = MAGNETOMETER_STATE_WAIT;
            break;
        }
        
        case MAGNETOMETER_STATE_WAIT:
        {
            
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
