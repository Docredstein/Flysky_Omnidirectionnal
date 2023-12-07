/*
 * Stepper.h
 *
 *  Created on: Nov 25, 2023
 *      Author: franc
 */

#ifndef STEPPER_H_
#define STEPPER_H_


#include "main.h"

typedef struct {
	uint32_t Pos;
	uint32_t TargetPos;
	uint8_t moving;
	uint8_t UpDir;
	GPIO_TypeDef DirBank;
	uint16_t DirPin;
	TIM_HandleTypeDef Timer_PWM;
	TIM_HandleTypeDef Timer_Gate;
} Stepper_t;
HAL_StatusTypeDef Stepper_Init(Stepper_t *stepper, GPIO_TypeDef DirBank,
		uint16_t DirPin,TIM_HandleTypeDef Timer_PWM,TIM_HandleTypeDef Timer_Gate,uint8_t UpDir );
HAL_StatusTypeDef Stepper_InitPos(Stepper_t *stepper);
HAL_StatusTypeDef Stepper_SetPos(Stepper_t *stepper, uint32_t TargetPos);
#endif /* STEPPER_H_ */
