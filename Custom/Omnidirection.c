#include "Omnidirection.h"
HAL_StatusTypeDef Transform_Omni(uint16_t Channel[3], float output[4]) {
	float input[3] = { 0 };
	uint8_t forward[4] = { 1, 1, -1, -1 };
	uint8_t right[4] = { 1, -1, -1, 1 };
	uint8_t rotate_c[4] = { -1, 1, -1, 1 };
	for (int i = 0; i < 3; i++) {
		input[i] = ((float) Channel[i] - 1500) / 500;

	}
	float norme = sqrtf(
			input[0] * input[0] + input[1] * input[1] + input[2] * input[2]);
	if (norme >= 1) {
		for (int i = 0; i < 4; i++) {
			output[i] = (input[0] * forward[i] + input[1] * right[i]
					+ input[2] * rotate_c[i]) / norme;
		}
	} else {
		for (int i = 0; i < 4; i++) {
			output[i] = input[0] * forward[i] + input[1] * right[i]
					+ input[2] * rotate_c[i];
		}
	}
	return HAL_OK;
}
