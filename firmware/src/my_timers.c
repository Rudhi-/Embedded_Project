#include "my_timers.h" 

void callback_100ms( TimerHandle_t xTimer )
{
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
}

void callback_4ms( TimerHandle_t xTimer )
{
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
}