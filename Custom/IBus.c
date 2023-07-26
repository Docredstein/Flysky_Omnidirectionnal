/*
 * IBus.cpp
 *
 *  Created on: Jul 7, 2023
 *      Author: franc
 */

#include "IBus.h"

uint16_t IBUS_Checksum(uint8_t value[14]) {
	uint16_t checksum = 0xFFFF;
	for (int i = 0; i < 14; i++) {
		checksum -= value[i];
	}
	return checksum;
}

HAL_StatusTypeDef IBUS_Init(IBus_struct *Ibus, UART_HandleTypeDef *handle) {
	memset(Ibus->channel,1500,6*sizeof(uint16_t));
	Ibus->handle = handle;
	memset(Ibus->UARTBuffer,0,128);
	return HAL_UART_Receive_DMA(Ibus->handle, Ibus->UARTBuffer, 128);

}

HAL_StatusTypeDef IBUS_Update(IBus_struct *Ibus) {
	HAL_UART_Receive_DMA(Ibus->handle, Ibus->UARTBuffer, 128);
	uint8_t Packet[64] = {0};
	uint16_t packetStart = 0;
	for (uint16_t i = 0;i<127;i++) {
		if (Ibus->UARTBuffer[i] == 0x20 && Ibus->UARTBuffer[i+1]==0x40) {
			packetStart = i;
			break;
		}
	}
	if (packetStart >=64) {
		return HAL_OK;
	}

	uint16_t ChecksumReceived = (((uint16_t)Packet[29])<<8) + (uint16_t)Packet[28];

	for (uint16_t i=0;i<6;i++) {
		Ibus->channel[i] = (((uint16_t)Packet[2*i+1])<<8) + (uint16_t)Packet[2*i];

	}
	return HAL_OK;
}
uint16_t* IBUS_GetChannels(IBus_struct *Ibus) {
	return Ibus->channel;
}
