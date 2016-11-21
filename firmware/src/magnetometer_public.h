/* 
 * File:   magnetometer_public.h
 * Author: Takondwa Kakusa
 *
 * Created on October 20, 2016, 5:47 PM
 */

#ifndef MAGNETOMETER_PUBLIC_H
#define	MAGNETOMETER_PUBLIC_H

#define LOOP_SIZE 1
 
int magcounter;
bool magflag;
void GetMagnetometerAq();
void GetMagnetometerData();
void StopGettingMagData();
void StartMagDebug();
void GetOffset();
QueueHandle_t MessageQueueDin;


#endif	/* MAGNETOMETER_PUBLIC_H */

