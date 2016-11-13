#include "my_timers.h" 

void callback_LED_ON( TimerHandle_t xTimer ){
    start_LED_ON = 1;
    start_ultrasonic = 1;
    start_senddata = 1;
    
    reflectance_finished = 0;
                
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

    xTimerReset(timer_LED_OFF, 0);
    
    //dbgOutputVal(0x01);
    //dbgOutputVal(0x00);
}
void callback_LED_OFF( TimerHandle_t xTimer ){
    start_LED_OFF = 1;
    
    xTimerStop(timer_LED_OFF, 0);
                
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

    xTimerReset(timer_LED_INPUT, 0);
    
    //dbgOutputVal(0x02);
    //dbgOutputVal(0x00);
}
void callback_LED_INPUT( TimerHandle_t xTimer ){
    start_LED_INPUT = 1;
    
    xTimerStop(timer_LED_INPUT, 0);

    reflectance_output = ((PORTBbits.RB15 << 7) +
         (PORTBbits.RB14 << 6) +
         (PORTBbits.RB13 << 5) +
         (PORTBbits.RB12 << 4) +
         (PORTBbits.RB11 << 3) +
         (PORTBbits.RB10 << 2) +
         (PORTBbits.RB9 << 1) +
         (PORTBbits.RB8));

    //reflectance_output = reflectance_output ^ 0xFF;
    packet_tx_data[1] = reflectance_output;

    reflectance_finished = 1;
    
    //dbgOutputVal(0x04);
    //dbgOutputVal(0x00);
}