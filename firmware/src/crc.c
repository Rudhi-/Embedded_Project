#include "crc.h"


#define WIDTH		(8*sizeof(uint8_t))
#define UPPERBIT		(1 << (WIDTH -1))
#define POLYNOMIAL	0xEA
#define DEBUG		true

static uint8_t table[256];

void makeCRCTable(void) {
	uint8_t remainder; 
	int i;
	// cycling through number of possible bytes
	for (i = 0; i < 256; i++) {
		remainder = i << (WIDTH - 8);
		uint8_t j;
		for (j = 8; j > 0; j--) {
			if (remainder&UPPERBIT) {
				remainder = (remainder << 1) ^ POLYNOMIAL;
			}
			else{
				remainder = (remainder << 1);
			}
		}
		table[i] = remainder;
	}
}

uint8_t checksumCreator(uint8_t message[], int n) {
	int  i;
	uint8_t checksum = 0;
	for (i = 0; i < n; i++) {
		uint8_t x = message[i] ^ (checksum >> (WIDTH - 8));
		checksum = table[x] ^ (checksum << 8);
	}
	#if DEBUG
	printf("%x\n", checksum);
	#endif
	return (checksum);
}

bool crcMatches(uint8_t message[], int n) {
	if ((uint8_t)message[n] == checksumCreator(message, n)) {
		return true;
	}
	else {
		return false;
	}
}
