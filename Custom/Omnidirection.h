#ifndef OMNIDIRECTION_H
#define OMNIDIRECTION_H
#include "main.h"


HAL_StatusTypeDef Transform(uint8_t Channel[3], float output[4]) {
	float input[3] = {0};
	uint8_t forward[4] = {1,1,-1,-1};
	uint8_t right[4] = {1,-1,-1,1};
	uint8_t rotate_c = {-1,1,-1,1};
	for (int i =0; i<3;i++) {
		input[i] = (Channel[i]-1500)/500;

	}
	for (int i=0;i<4;i++) {
		output[i] = min(max(-1,input[0] * forward[i] + input[1]*right[i] + input[2]*rotate_c[i]),1) ;
	}

}

#endif