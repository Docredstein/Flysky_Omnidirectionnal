/*s
 * Sabertooth.cpp
 *
 *  Created on: Jul 7, 2023
 *      Author: franc
 */

#include "Sabertooth.h"
static Sabertooth *sabertooth_ptr;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart != sabertooth_ptr->handle) {
		return;
	}
	if (sabertooth_ptr->Queue.NumberOfPacket <= 0) {
		sabertooth_ptr->Queue.CurrentlySending = 0;
	} else {
		HAL_UART_Transmit_DMA(huart, sabertooth_ptr->Queue.FirstPacket->Packet,
				sabertooth_ptr->Queue.FirstPacket->length);
		sabertooth_ptr->Queue.FirstPacket =
				sabertooth_ptr->Queue.FirstPacket->Next_Packet;
		sabertooth_ptr->Queue.NumberOfPacket -= 1;
		sabertooth_ptr->Queue.CurrentlySending = 1;
	}
}
HAL_StatusTypeDef Sabertooth_Init(Sabertooth *saber, UART_HandleTypeDef *handle,
		uint8_t motor[4], uint8_t adress[4]) {
	sabertooth_ptr = saber;
	for (int i = 0; i < 4; i++) {
		saber->adress[i] = adress[i];
		saber->motor[i] = motor[i];
	}
	saber->handle = handle;

	/*saber->Queue.FirstPacket = 0;
	 saber->Queue.NumberOfPacket = 0;*/
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
	//return HAL_UART_Transmit_DMA(saber->handle, Packet, 3 + data_length);
	return HAL_UART_Transmit(saber->handle, Packet, 3+data_length,10);
	/*static Packet_el PacketToSend;
	PacketToSend.Next_Packet = 0;
	PacketToSend.Packet = Packet;
	PacketToSend.length = 3 + data_length;
	if (!saber->Queue.CurrentlySending) {
		HAL_UART_Transmit_DMA(saber->handle, Packet, PacketToSend.length);
		saber->Queue.CurrentlySending = 1;
	} else if (saber->Queue.NumberOfPacket > 0) {

		saber->Queue.lastPacket->Next_Packet = &PacketToSend;
		saber->Queue.lastPacket = &PacketToSend;
		saber->Queue.NumberOfPacket += 1;
	} else {
		saber->Queue.FirstPacket = &PacketToSend;
		saber->Queue.lastPacket = &PacketToSend;
		saber->Queue.NumberOfPacket = 1;
	}*/
	return HAL_OK;
}

HAL_StatusTypeDef Sabertooth_Drive(Sabertooth *saber, float command[4]) {
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

