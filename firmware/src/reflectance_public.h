/* 
 * File:   reflectance_public.h
 * Author: Thomas
 *
 * Created on November 1, 2016, 10:17 PM
 */

#ifndef REFLECTANCE_PUBLIC_H
#define	REFLECTANCE_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    

#include "header.h"

TimerHandle_t timer_LED_ON;
TimerHandle_t timer_LED_OFF;
TimerHandle_t timer_LED_INPUT;

void startReflectance();

#ifdef	__cplusplus
}
#endif

#endif	/* REFLECTANCE_PUBLIC_H */

