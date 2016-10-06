/* 
 * File:   crc.h
 * Author: Anirudha Simha
 *
 * Created on October 1, 2016, 8:54 PM
 */

#ifndef CRC_H
#define	CRC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#ifdef	__cplusplus
extern "C" {
#endif


	/**
		This is an initialization function
	*/
	void makeCRCTable(void);

	/**
		message[] is an array of 8bit values where it should be basically message[0:7] 
			is the actual message and message[8] is the checksum value
		n is the number of bytes to process in the crc calculation, so almost always for
			our implementation this will be 7
	*/
	uint8_t checksumCreator(uint8_t message[], int n);

	/**
		message[] is an array of 8bit values where it should be basically message[0:7] 
			is the actual message and message[8] is the checksum value
		n is the number of bytes to process in the crc calculation, so almost always for
			our implementation this will be 7
	*/
	bool crcMatches(uint8_t message[], int n);


#ifdef	__cplusplus
}
#endif

#endif	/* CRC_H */

