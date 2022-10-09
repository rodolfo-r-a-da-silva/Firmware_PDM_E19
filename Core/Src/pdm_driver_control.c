/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

//Process input pin levels
void PDM_Input_Process()
{
	inputLevels = 0x0000;

	inputLevels  =  HAL_GPIO_ReadPin(INPUT1_GPIO_Port, INPUT1_Pin);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT2_GPIO_Port, INPUT2_Pin) << 1);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin) << 2);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT4_GPIO_Port, INPUT4_Pin) << 3);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT5_GPIO_Port, INPUT5_Pin) << 4);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT6_GPIO_Port, INPUT6_Pin) << 5);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT7_GPIO_Port, INPUT7_Pin) << 6);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT8_GPIO_Port, INPUT8_Pin) << 7);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT9_GPIO_Port, INPUT9_Pin) << 8);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT10_GPIO_Port, INPUT10_Pin) << 9);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT11_GPIO_Port, INPUT11_Pin) << 10);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT12_GPIO_Port, INPUT12_Pin) << 11);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT13_GPIO_Port, INPUT13_Pin) << 12);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT14_GPIO_Port, INPUT14_Pin) << 13);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT15_GPIO_Port, INPUT15_Pin) << 14);

	inputLevels |= (HAL_GPIO_ReadPin(INPUT16_GPIO_Port, INPUT16_Pin) << 15);

	return;
}

//Process output pin levels
void PDM_Output_Process()
{
	GPIO_PinState output_levels[NBR_OF_OUTPUTS];

	//Process input conditions and safety flags for each output
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		if((((flagDriverSafety >> i) & 0x01) == 0)
				&& (__PDM_INPUT_CONDITION_COMPARE(outputStruct[i].inputEnable[0], outputStruct[i].inputLevels[0], outputStruct[i].outEnable[0])
				||  __PDM_INPUT_CONDITION_COMPARE(outputStruct[i].inputEnable[1], outputStruct[i].inputLevels[1], outputStruct[i].outEnable[1])))
			output_levels[i] = GPIO_PIN_SET;
		else
			output_levels[i] = GPIO_PIN_RESET;
	}

	HAL_GPIO_WritePin(OUTPUT5_GPIO_Port, OUTPUT5_Pin, output_levels[4]);
	HAL_GPIO_WritePin(OUTPUT6_GPIO_Port, OUTPUT6_Pin, output_levels[5]);
	HAL_GPIO_WritePin(OUTPUT7_GPIO_Port, OUTPUT7_Pin, output_levels[6]);
	HAL_GPIO_WritePin(OUTPUT8_GPIO_Port, OUTPUT8_Pin, output_levels[7]);
	HAL_GPIO_WritePin(OUTPUT9_GPIO_Port, OUTPUT9_Pin, output_levels[8]);
	HAL_GPIO_WritePin(OUTPUT10_GPIO_Port, OUTPUT10_Pin, output_levels[9]);
	HAL_GPIO_WritePin(OUTPUT11_GPIO_Port, OUTPUT11_Pin, output_levels[10]);
	HAL_GPIO_WritePin(OUTPUT12_GPIO_Port, OUTPUT12_Pin, output_levels[11]);
	HAL_GPIO_WritePin(OUTPUT13_GPIO_Port, OUTPUT13_Pin, output_levels[12]);
	HAL_GPIO_WritePin(OUTPUT14_GPIO_Port, OUTPUT14_Pin, output_levels[13]);
	HAL_GPIO_WritePin(OUTPUT15_GPIO_Port, OUTPUT15_Pin, output_levels[14]);
	HAL_GPIO_WritePin(OUTPUT16_GPIO_Port, OUTPUT16_Pin, output_levels[15]);
	PDM_PWM_Output_Process(&pwmOutStruct[0], 0, output_levels[0]);
	PDM_PWM_Output_Process(&pwmOutStruct[1], 1, output_levels[1]);
	PDM_PWM_Output_Process(&pwmOutStruct[2], 2, output_levels[2]);
	PDM_PWM_Output_Process(&pwmOutStruct[3], 3, output_levels[3]);

	return;
}
