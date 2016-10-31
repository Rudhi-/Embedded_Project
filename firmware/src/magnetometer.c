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

void GetMagnetometerData()
{
    magnetometerData.state = MAGNETOMETER_STATE_SERVICE_TASKS;
}
static const double tanLUT[90] = 
{ 
    0.00000, 0.01746, 0.03492, 0.05241, 0.06993,
    0.08749, 0.10510, 0.12278, 0.14054, 0.15838,
    0.17633, 0.19438, 0.21256, 0.21256, 0.24933,
    0.26795, 0.28675, 0.30573, 0.32492, 0.34433,
    0.36397, 0.38386, 0.40403, 0.42447, 0.44523,
    0.46631, 0.48773, 0.50953, 0.53171, 0.55431,
    0.57735, 0.60086, 0.62487, 0.64941, 0.67451, 
    0.70021, 0.72654, 0.75355, 0.78129, 0.80978,
    0.83910, 0.86929, 0.90040, 0.93252, 0.96569,
    1.03553, 1.07237, 1.11061, 1.15037, 1.19175,
    1.23490, 1.27994, 1.32704, 1.37638, 1.42815,
    1.48256, 1.53986, 1.60033, 1.66428, 1.73205,
    1.80405, 1.88073, 1.96261, 2.05030, 2.14451,
    2.24604, 2.35585, 2.47509, 2.60509, 2.74748,
    2.90421, 3.07768, 3.27085, 3.48741, 3.73205,
    4.01078, 4.33148, 4.70463, 5.14455, 5.67128,
    6.31375, 7.11537, 8.14435, 9.51436, 11.43005,
    14.30067, 19.08114, 28.63625, 57.28996  
};


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

void sendDataI2c(uint8_t address, uint8_t data)
{
    //Start i2c
    DRV_I2C_MasterStart(I2C_ID_1);
    
    //Write address write
    DRV_I2C_ByteWrite(I2C_ID_1, deviceAddressSlaveWrite);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Write register
    DRV_I2C_ByteWrite(I2C_ID_1, address);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Write data
    DRV_I2C_ByteWrite(I2C_ID_1, data);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Stop i2c
    DRV_I2C_MasterStop(I2C_ID_1); 
}

uint8_t readDataByteI2c(uint8_t address)
{
    uint8_t value;
    
    //Start i2c
    DRV_I2C_MasterStart(I2C_ID_1);
    
    //Write address write
    DRV_I2C_ByteWrite(I2C_ID_1, deviceAddressSlaveWrite);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Write register
    DRV_I2C_ByteWrite(I2C_ID_1, address);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Restart i2c
    DRV_I2C_MasterRestart(I2C_ID_1);
    
    //Write address read
    DRV_I2C_ByteWrite(I2C_ID_1, deviceAddressSlaveRead);
    DRV_I2C_WaitForByteWriteToComplete(I2C_ID_1);
    
    //Read the byte
    while (!DRV_I2C_WaitForReadByteAvailable(I2C_ID_1));
    value = DRV_I2C_ByteRead(I2C_ID_1);
    while(!PLIB_I2C_MasterReceiverReadyToAcknowledge(I2C_ID_1));
    PLIB_I2C_ReceivedByteAcknowledge (I2C_ID_1, true);

    //Stop i2c
    DRV_I2C_MasterStop(I2C_ID_1);

    return value;
    
}

uint16_t readWordByteI2c(uint8_t address)
{
    uint16_t value;
    
    uint8_t low = readDataByteI2c(address);
    uint8_t high = readDataByteI2c(address + 1);
    
    value = ((high << 8) | low);
    
    return value;
}

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
                PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            }
            magnetometerData.txbufferhandle = DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_1[0], (sizeof(txBuffer_1)-1), NULL);
            magnetometerData.txbufferhandle = DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_2[0], (sizeof(txBuffer_2)-1), NULL);
            magnetometerData.txbufferhandle = DRV_I2C_Transmit( magnetometerData.i2c_handle, 0x3c, &txBuffer_3[0], (sizeof(txBuffer_3)-1), NULL);
            
            DRV_I2C_Close( magnetometerData.i2c_handle );
            
            bool appInitialized = true;
            
       
        
            if (appInitialized)
            {
            
                magnetometerData.state = MAGNETOMETER_STATE_WAIT;
            }
            break;
        }

        case MAGNETOMETER_STATE_SERVICE_TASKS:
        {   
            int i;
            magnetometerData.i2c_handle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE); 
            if (DRV_HANDLE_INVALID == magnetometerData.i2c_handle)
            {
                
            }
            for (i = 0; i < 100; i++)
            {
                while(!DRV_I2C_BUFFER_EVENT_COMPLETE == DRV_I2C_TransferStatusGet ( magnetometerData.i2c_handle, magnetometerData.txbufferhandle ));
                DRV_I2C_TransmitThenReceive ( magnetometerData.i2c_handle, 0x3c, txBuffer, (sizeof(txBuffer)-1), rx_buffer, 6, NULL);

                DRV_I2C_Close( magnetometerData.i2c_handle );

                magnetometerData.rxbufferx = (rx_buffer[0] << 8) | rx_buffer[1];
                magnetometerData.rxbuffery = (rx_buffer[4] << 8) | rx_buffer[5];

                magnetometerData.rxbufferx = (rx_buffer[0] << 8) | rx_buffer[1];
                magnetometerData.rxbuffery = (rx_buffer[4] << 8) | rx_buffer[5];

                if (magnetometerData.rxbufferx == 0)
                    magnetometerData.rxbufferx = 1;
                magnetometerData.float_val = magnetometerData.rxbuffery/(double)magnetometerData.rxbufferx;
                
                magnetometerData.bearing[i] = atan(magnetometerData.float_val) * 360/ (2*3.14);
            }
            magnetometerData.bearingAvg = magnetometerData.bearing[0];
            for (i = 1; i < 100; i++)
            {
                magnetometerData.bearingAvg = magnetometerData.bearingAvg + magnetometerData.bearing[i];
            }
            magnetometerData.bearingAvg = magnetometerData.bearingAvg/20;
            if (magnetometerData.rxbufferx < 0 && magnetometerData.rxbuffery > 0)
                magnetometerData.bearingAvg = 180 + magnetometerData.bearingAvg;
            else if (magnetometerData.rxbufferx < 0 && magnetometerData.rxbuffery < 0)
                magnetometerData.bearingAvg = 180 +  magnetometerData.bearingAvg;
            else if (magnetometerData.rxbufferx > 0 && magnetometerData.rxbuffery < 0)
                magnetometerData.bearingAvg = 360 + magnetometerData.bearingAvg;

            dbgOutputVal(magnetometerData.bearingAvg);
            magnetometerData.dx_data[0] = (magnetometerData.bearingAvg & 0xFF00) >> 8;
            magnetometerData.dx_data[1] = magnetometerData.bearingAvg & 0xFF;            
            
            xQueueSend( MessageQueueDin, magnetometerData.dx_data, pdFAIL );
            
            //magnetometerData.float_val = magnetometerData.rxbuffery/magnetometerData.rxbufferx;
            
            
            
            magnetometerData.state = MAGNETOMETER_STATE_WAIT;
            break;
        }
        
        case MAGNETOMETER_STATE_WAIT:
        {
            if (magnetometerData.getval && magflag)
                magnetometerData.state = MAGNETOMETER_STATE_SERVICE_TASKS;
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
