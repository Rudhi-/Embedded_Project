/* 
 * File:   debug.h
 * Author: Takondwa Kakusa
 *
 * Created on September 14, 2016, 10:21 PM
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#ifndef DEBUG_H
#define	DEBUG_H

#define ENTERING_TASK '1'
#define BEFORE_WHILE_LOOP '2'
#define SENDING_TO_QUEUE '3'
#define RECEIVING_FROM_QUEUE '4'
#define SENT_TO_QUEUE '5'
#define RECEIVED_FROM_QUEUE '6'
#define ENTERING_ISR '7'
#define LEAVING_ISR '8'
#define SENDING_TO_ISR_QUEUE '9'
#define RECEIVING_FROM_ISR_QUEUE 'A'
#define SENT_TO_ISR_QUEUE 'B'
#define LEAVING_ISR_QUEUE 'C'
#define ERROR_HAS_OCCURED 'D'

void dbgOutputVal(unsigned char outVal);
void dbgOutputLocReset();
void dbgOutputLoc(unsigned char outVal);


#endif	/* DEBUG_H */

