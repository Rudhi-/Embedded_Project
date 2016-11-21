/* 
 * File:   uartrx_public1.h
 * Author: Ryan
 *
 * Created on September 27, 2016, 9:30 AM
 */

#ifndef UARTRX_PUBLIC1_H
#define	UARTRX_PUBLIC1_H

#include "crc.h"

enum RECEIVE_STATES {WAIT_ON_MESSAGE = 0, WAIT_ON_ACK} receiveState; 
QueueHandle_t MessageQueueWin;

bool stopAllProcesses();
void clearStop();
void SendToTheQueue();
void Transmit2();

#endif	/* UARTRX_PUBLIC1_H */

