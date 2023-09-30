#ifndef OMNIDIRECTION_H
#define OMNIDIRECTION_H
#include "main.h"
#include "math.h"


HAL_StatusTypeDef Transform_Omni(uint16_t Channel[3], float output[4]);
void CurveInput(float input[3]);
#endif
