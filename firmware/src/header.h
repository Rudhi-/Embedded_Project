/* 
 * File:   headers.h
 * Author: Ryan
 *
 * Created on October 5, 2016, 3:02 PM
 */

#ifndef HEADERS_H
#define	HEADERS_H

#define PIC_ID 1 //set to your pic
#define DBG_MSG         0x00
#define CMD_MSG         0x80 
#define ACK_MSG         0x40
#define INT_MSG         0xC0
#define CONTROL_THREAD_ID 0x00
#define INTERNALMASK      0xC0
#define MESSAGEMASK       0x80
#define ACKMASK           0x40
#define DEBUGMASK         0x00
#define SETMASK           0x20
#define GETMASK           0x00
#define GETRESPONSEMASK   0x10
#define SOURCEWRITEMASK   0x00
#define CONTROLTHREADMASK 0x00
#define MOTORTHREADMASK   0x01
#define REFLECTTHREADMASK 0x02
#define MAGTHREADMASK     0x03
#define MUSCLEPIC         0x01
#define SENSORPIC         0x02
#define BRAINPIC          0x03
#define FOLLOWERPIC       0x04

#include "uartrx_public.h"
#include "uarttx_public.h"
#include "motors_public.h"
#include "motors_public.h"
#include "magnetometer_public.h"
#include "debug.h"
#include "queue.h"



uint8_t rx_counter;
uint8_t tx_counter;

#endif	/* HEADERS_H */
