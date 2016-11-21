/* 
 * File:   uarttx_public.h
 * Author: Ryan
 *
 * Created on September 27, 2016, 9:38 AM
 */

#ifndef UARTTX_PUBLIC_H
#define	UARTTX_PUBLIC_H

QueueHandle_t MessageQueueWout;
bool wait_on_ack;

void SendMessageOnce(uint8_t *msg);
void SendMessageAck(uint8_t *msg);
void ReSendMessage();
void TransmitTheMessage();

#endif	/* UARTTX_PUBLIC_H */

