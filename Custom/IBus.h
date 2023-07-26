/*
 * IBus.h
 *
 *  Created on: Jul 7, 2023
 *      Author: franc
 */

#ifndef IBUS_H_
#define IBUS_H_
#include "stm32l4xx_hal.h"
#include "stdio.h"
typedef struct {
	UART_HandleTypeDef *handle;
	uint16_t channel[10];
	uint8_t UARTBuffer[128];

} IBus_struct;

HAL_StatusTypeDef IBUS_Init(IBus_struct *Ibus, UART_HandleTypeDef *handle);
HAL_StatusTypeDef IBUS_Update(IBus_struct *Ibus);
uint16_t* IBUS_GetChannels(IBus_struct *Ibus);
uint16_t IBUS_Checksum(uint8_t value[14]);
#endif /* IBUS_H_ */
