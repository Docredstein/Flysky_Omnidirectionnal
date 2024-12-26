#include "Kangaroo.h"
static uint8_t Packet_number =1;
uint16_t CRC14(uint8_t data[], uint16_t lenght) {
	uint16_t polynome = 0x21E8<<1 +1;



}
HAL_StatusTypeDef Kangaroo_Send(Sabertooth *saber, uint8_t adress,
		uint8_t command, uint8_t data[], uint16_t data_length) {
	uint8_t Packet[64] = {0};
	Packet[0] = adress;
	Packet[1]= command;
	Packet[2] = data_length;
	for (int i =0;i<data_length;i++) {
		Packet[3+i]=data[i];

	}
	uint32_t out = HAL_CRC_Calculate(saber->crc, Packet, 3+data_length);
	Packet[3+data_length]=out&&0x1111111;
	Packet[4+data_length]=(out>>8)&&0x1111111;
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

	for (int i = 0; i < 4; i++) {
			if (command[i] >= 0) {
				uint8_t data = floor(command[i] * 127);
				Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i], &data,
						1);

			} else {
				uint8_t data = floor(-command[i] * 127);
				Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i] + 1,
						&data, 1);
			}
		}

		return HAL_OK;
}
;


