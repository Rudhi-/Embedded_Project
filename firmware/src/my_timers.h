/* 
 * File:   my_timers.h
 * Author: Thomas
 *
 * Created on November 1, 2016, 6:49 PM
 */

#ifndef MY_TIMERS_H
#define	MY_TIMERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "main_task.h"
#include "header.h"
    
void callback_LED_ON( TimerHandle_t xTimer );
void callback_LED_OFF( TimerHandle_t xTimer );
void callback_LED_INPUT( TimerHandle_t xTimer );


#ifdef	__cplusplus
}
#endif

#endif	/* MY_TIMERS_H */

