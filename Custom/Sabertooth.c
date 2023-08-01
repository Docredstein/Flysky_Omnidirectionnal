/*s
 * Sabertooth.cpp
 *
 *  Created on: Jul 7, 2023
 *      Author: franc
 */

#include "Sabertooth.h"

HAL_StatusTypeDef Sabertooth_Init(Sabertooth *saber, UART_HandleTypeDef *handle,
		uint8_t motor[4], uint8_t adress[4]) {
	for (int i = 0; i < 4; i++) {
		saber->adress[i] = adress[i];
		saber->motor[i] = motor[i];
	}
	saber->handle = handle;

	return HAL_OK;
}
HAL_StatusTypeDef Sabertooth_Send(Sabertooth *saber, uint8_t address,
		uint8_t command, uint8_t data[], uint16_t data_length) {
	uint8_t Packet[3 + data_length];
	Packet[0] = address;
	Packet[1] = command;
	for (int i = 0; i < data_length; i++) {
		Packet[i + 2] = data[i];
	}
	uint8_t Checksum = 0;
	for (int i = 0; i < 2 + data_length; i++) {
		Checksum += Packet[i];
	}
	Checksum = Checksum & 0x7F;
	Packet[2 + data_length] = Checksum;
	return HAL_UART_Transmit_DMA(saber->handle, Packet, 3 + data_length);

}
HAL_StatusTypeDef Sabertooth_Drive(Sabertooth *saber, float command[4]) {
	for (int i = 0; i < 4; i++) {
		if (command[i] >= 0) {
			uint8_t data = floor(command[i] * 127);
			Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i], &data,
					1);

		} else {
			uint8_t data = floor(-command[i] * 127);
			Sabertooth_Send(saber, saber->adress[i], 4 * saber->motor[i]+1, &data,
					1);
		}
	}

	return HAL_OK;
}
