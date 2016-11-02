#include "my_timers.h" 

void callback_LED_ON( TimerHandle_t xTimer ){
    start_LED_ON = 1;
    start_ultrasonic = 1;
    start_senddata = 1;
}
void callback_LED_OFF( TimerHandle_t xTimer ){
    start_LED_OFF = 1;
}
void callback_LED_INPUT( TimerHandle_t xTimer ){
    start_LED_INPUT = 1;
}