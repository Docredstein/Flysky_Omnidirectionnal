#include "Omnidirection.h"
float exponent[3] = {2,2,2};


void CurveInput(float input[3]) {
	for (int i =0;i<3;i++) {
		input[i] =expf( fabsf(input[i]) * exponent[i] )*input[i]/expf(exponent[i]);
	}
}
HAL_StatusTypeDef Transform_Omni(uint16_t Channel[3], float output[4]) {
	float input[3] = { 0 };
	int8_t forward[4] = { 1, 1, 1, 1 };
	int8_t right[4] = { 1, -1, -1, 1 };
	int8_t rotate_c[4] = { 1, 1, -1, -1 };
	for (int i = 0; i < 3; i++) {
		input[i] = ((float) Channel[i] - 1500) / 500;
		if (Channel[i] <500 || Channel[i] >2000) {
			input[i]=0;
		}

	}
	CurveInput(input);
	for (int i = 0; i < 4; i++) {
				output[i] = input[0] * forward[i] + input[1] * right[i]
						+ input[2] * rotate_c[i];
				if (output[i]>1) {
					output[i]=1;
				}
				else if (output[i]<-1) {
					output[i]=-1;
				}
			}
	/*float norme = sqrtf(
			output[0] * output[0] + output[1] * output[1] + output[2] * output[2]+output[3] * output[3]);*/


	return HAL_OK;
}
