#include "Kangaroo.h"
static uint8_t Packet_number =1;

size_t bitpackNumber(uint8_t* buffer, int32_t number)
{
 size_t i = 0;

 if (number < 0) { number = -number; number <<= 1; number |= 1; }
 else { number <<= 1; }

 while (i < 5)
 {
 buffer[i ++] = (number & 0x3f) | (number >= 0x40 ? 0x40 : 0x00);
 number >>= 6; if (!number) { break; }
 }

 return i;
}

uint16_t crc14(const uint8_t* data, size_t length)
{
 uint16_t crc = 0x3fff; size_t i, bit;
 for (i = 0; i < length; i ++)
 {
 crc ^= data[i] & 0x7f;
 for (bit = 0; bit < 7; bit ++)
 {
 if (crc & 1) { crc >>= 1; crc ^= 0x22f0; }
 else { crc >>= 1; }
 }
 }
 return crc ^ 0x3fff;
}


HAL_StatusTypeDef Kangaroo_Send(Sabertooth *saber, uint8_t adress,
		uint8_t command, uint8_t data[], uint16_t data_length) {
	uint8_t Packet[64] = {0};
	uint16_t crc;
	Packet[0] = adress;
	Packet[1]= command;
	Packet[2] = data_length;
	for (int i =0;i<data_length;i++) {
		Packet[3+i]=data[i];

	}
	crc = crc14(Packet, 3 + data_length);
	//uint32_t out = HAL_CRC_Calculate(saber->crc, Packet, 3+data_length);
	Packet[3+data_length]= crc & 0x7f;
	Packet[4+data_length]= (crc >> 7) & 0x7f;
	HAL_UART_Transmit(saber->handle,Packet,5+data_length,10);

	return HAL_OK;
}
;
HAL_StatusTypeDef Kangaroo_Init(Sabertooth *saber, UART_HandleTypeDef *handle,
		uint8_t motor[4], uint8_t adress[4]) {
	//sabertooth_ptr = saber;
	for (int i = 0; i < 4; i++) {
		saber->adress[i] = adress[i];
		saber->motor[i] = motor[i];
	}
	saber->handle = handle;
	/*saber->Queue.FirstPacket = 0;
	 saber->Queue.NumberOfPacket = 0;*/
	for (int i =0;i<4;i++) {
		uint8_t data[3] = {saber->motor[i]?'1':'2',64,Packet_number};
		Packet_number++;
		Kangaroo_Send(saber, adress[i], 32, data, 3); //32 --> Start packet

	}


	return HAL_OK;




}
;



HAL_StatusTypeDef Kangaroo_drive(Sabertooth *saber, float command[4]) {
	for (int i =0;i<4;i++) {
	uint8_t data[14]; size_t length = 0;
	 data[length ++] = saber->motor[i]?'1':'2';
	 data[length ++] = 0; // move flags

	 data[length ++] = 2; // Speed
	 float speedf = (command[i]*SPEED_CONSTANT);
	 int32_t speed = (int32_t) speedf;
	 length += bitpackNumber(&data[length], speed);
	 Sabertooth_Send(saber, saber->adress[i], 36, data, length);



/*
	 if (speedLimit >= 0)
	 {
	 data[length ++] = 2; // Speed (Limit if combined with Position)
	 length += bitpackNumber(&data[length], speedLimit);
	 }
*/
	}


	/*for (int i = 0; i < 4; i++) {
			if (command[i] >= 0) {
				uint8_t data = floor(command[i] * 127);
				Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i], &data,
						1);

			} else {
				uint8_t data = floor(-command[i] * 127);
				Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i] + 1,
						&data, 1);
			}
		}*/

		return HAL_OK;
}
;


