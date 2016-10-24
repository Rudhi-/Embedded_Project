/* 
 * File:   magnetometer_public.h
 * Author: Takondwa Kakusa
 *
 * Created on October 20, 2016, 5:47 PM
 */

#ifndef MAGNETOMETER_PUBLIC_H
#define	MAGNETOMETER_PUBLIC_H


int magcounter;
bool magflag;
void GetMagnetometerData();
QueueHandle_t MessageQueueDin;


#endif	/* MAGNETOMETER_PUBLIC_H */

