/* 
 * File:   headers.h
 * Author: Ryan
 *
 * Created on October 5, 2016, 3:02 PM
 */

#ifndef HEADERS_H
#define	HEADERS_H

#include "uartrx_public.h"
#include "uarttx_public.h"

#include "debug.h"
#include "queue.h"

#include "crc.h"

#include "timers.h"
#include "my_timers.h"

#include "reflectance_public.h"

#define PIC_ID 2 //set to your pic
#define DBG_MSG         0x00
#define CMD_MSG         0x80 
#define ACK_MSG         0x40
#define INT_MSG    0xC0

uint8_t rx_counter;
uint8_t tx_counter;


#endif	/* HEADERS_H */