/*
 * Sabertooth.h
 *
 *  Created on: Jul 7, 2023
 *      Author: franc
 */

#ifndef SABERTOOTH_H_
#define SABERTOOTH_H_
#include "main.h"

typedef struct {
	UART_HandleTypeDef *handle;
	uint8_t adress[4];
	//motor : 0 or 1
	uint8_t motor[4];

} Sabertooth;

HAL_StatusTypeDef Sabertooth_Drive(Sabertooth *saber,float command[4]);

HAL_StatusTypeDef Sabertooth_Init(Sabertooth *saber,UART_HandleTypeDef *handle, uint8_t motor[4],
		uint8_t adress[4]);
HAL_StatusTypeDef Sabertooth_Send(Sabertooth *saber,uint8_t adress, uint8_t command, uint8_t data[], uint16_t data_length
		);
#endif /* SABERTOOTH_H_ */
