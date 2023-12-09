/*
 * Stepper.cpp
 *
 *  Created on: Nov 25, 2023
 *      Author: franc
 */

#include "Stepper.h"

HAL_StatusTypeDef Stepper_Init(Stepper_t *stepper, GPIO_TypeDef * DirBank,
		uint16_t DirPin, TIM_HandleTypeDef *Timer_PWM,
		TIM_HandleTypeDef* Timer_Gate, uint8_t UpDir) {
	stepper->DirBank = DirBank;
	stepper->DirPin = DirPin;
	stepper->Pos = 0;
	stepper->TargetPos = 0;
	stepper->Timer_Gate = Timer_Gate;
	stepper->Timer_PWM = Timer_PWM;
	stepper->moving = 0;
	stepper->UpDir = UpDir;
	stepper->init_state=0;
	Stepper_InitPos(stepper);

	return HAL_OK;
}

HAL_StatusTypeDef Stepper_SetPos(Stepper_t *stepper, uint32_t TargetPos) {
if (TargetPos != stepper->TargetPos) {
	uint32_t oldTarget = stepper->TargetPos;
	stepper->TargetPos = TargetPos;
	uint32_t RemainingStep = 0;
	if (stepper->moving) {
		RemainingStep = stepper->Timer_Gate->Instance->CNT;
	}
	HAL_TIM_OC_Stop_IT(stepper->Timer_Gate, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(stepper->Timer_PWM, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop_IT(stepper->Timer_Gate);
	HAL_TIM_Base_Stop(stepper->Timer_PWM);
	uint32_t Step_to_target = 0;
	if (stepper->moving) {
		if (((oldTarget-stepper->Pos)>0) == ((TargetPos-stepper->Pos)>0)) { //même direction
			Step_to_target = RemainingStep + (TargetPos-oldTarget)*STEP_PER_POS;
		}
		else {
			Step_to_target = abs(TargetPos-stepper->Pos)*STEP_PER_POS + STEP_PER_POS-RemainingStep;
		}
	}
	else {
		Step_to_target = abs(TargetPos-stepper->Pos)*STEP_PER_POS;
	}
	HAL_GPIO_WritePin(stepper->DirBank, stepper->DirPin,(GPIO_PinState)( (TargetPos- stepper->Pos)>0? stepper->UpDir : !stepper->UpDir));
	stepper->Timer_Gate->Instance->CNT = Step_to_target;
	HAL_TIM_OC_Start_IT(stepper->Timer_Gate, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(stepper->Timer_PWM, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(stepper->Timer_Gate);
	HAL_TIM_Base_Start(stepper->Timer_PWM);
	stepper->moving=1;




}
return HAL_OK;
}

HAL_StatusTypeDef Stepper_InitPos(Stepper_t *stepper) {
	TIM_HandleTypeDef * gate = stepper->Timer_Gate;
	TIM_HandleTypeDef * PWM = stepper->Timer_PWM;
	HAL_GPIO_WritePin(stepper->DirBank, stepper->DirPin, stepper->UpDir);
	stepper->init_state = 1;
	gate->Instance->CNT = 3000;
	HAL_TIM_OC_Start(PWM, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(gate, TIM_CHANNEL_1);
	return HAL_OK;


}
