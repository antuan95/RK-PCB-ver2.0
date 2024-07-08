#include "crc.h"

uint8_t CRC8(const uint8_t *data, int length)
{
	uint8_t crc = 0x00;
	uint8_t extract;
	uint8_t sum;
	for(int i = 0; i < length; i++)
	{
		extract = *data;
		for(uint8_t tempI = 8; tempI; tempI--)
		{
			sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if(sum)
				crc ^= 0x8C;
			extract >>= 1;
		}
		data++;
	}
	return crc;
}
