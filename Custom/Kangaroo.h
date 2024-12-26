#pragma once
#include "main.h"
#include "sabertooth.h"

uint16_t CRC14(uint8_t data[], uint16_t lenght);

HAL_StatusTypeDef Kangaroo_drive(Sabertooth *saber, float command[4]);
HAL_StatusTypeDef Kangaroo_Init(Sabertooth *saber, UART_HandleTypeDef *handle,uint8_t motor[4], uint8_t adress[4]);
HAL_StatusTypeDef Kangaroo_Send(Sabertooth *saber, uint8_t adress,uint8_t command, uint8_t data[], uint16_t data_length);
