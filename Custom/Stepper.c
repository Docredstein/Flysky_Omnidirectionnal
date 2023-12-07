/*
 * Stepper.cpp
 *
 *  Created on: Nov 25, 2023
 *      Author: franc
 */

#include "Stepper.h"

HAL_StatusTypeDef Stepper_Init(Stepper_t *stepper, GPIO_TypeDef DirBank,
		uint16_t DirPin, TIM_HandleTypeDef Timer_PWM,
		TIM_HandleTypeDef Timer_Gate, uint8_t UpDir) {
	stepper->DirBank = DirBank;
	stepper->DirPin = DirPin;
	stepper->Pos = 0;
	stepper->TargetPos = 0;
	stepper->Timer_Gate = Timer_Gate;
	stepper->Timer_PWM = Timer_PWM;
	stepper->moving = 0;
	stepper->UpDir = UpDir;
	Stepper_InitPos(stepper);

	return HAL_OK;
}

HAL_StatusTypeDef Stepper_SetPos(Stepper_t *stepper, uint32_t TargetPos) {

}

HAL_StatusTypeDef Stepper_InitPos(Stepper_t *stepper) {
	TIM_HandleTypeDef gate = stepper->Timer_Gate;
	TIM_HandleTypeDef PWM = stepper->Timer_PWM;
	HAL_GPIO_WritePin(stepper->DirBank, stepper->DirPin, stepper->UpDir);


}
