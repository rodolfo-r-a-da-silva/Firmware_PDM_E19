/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

#ifdef LQFP64
static HAL_StatusTypeDef PDM_Output_Expander_Set(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef retVal = HAL_OK;
	uint8_t level_buffer = 0;

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[8].Enabled_Inputs[0], Output_Pin[8].Input_Levels[0],
									 Output_Pin[8].Enabled_Inputs[1], Output_Pin[8].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 8) & 0x01) == 1)))
		level_buffer |= 1 << 2;
	else
		level_buffer &= ~(1 << 2);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[9].Enabled_Inputs[0], Output_Pin[9].Input_Levels[0],
			 	 	 	 	 	 	 Output_Pin[9].Enabled_Inputs[1], Output_Pin[9].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 9) & 0x01) == 1)))
		level_buffer |= 1 << 3;
	else
		level_buffer &= ~(1 << 3);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[10].Enabled_Inputs[0], Output_Pin[10].Input_Levels[0],
			 	 	 	 	 	 	 Output_Pin[10].Enabled_Inputs[1], Output_Pin[10].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 10) & 0x01) == 1)))
		level_buffer |= 1;
	else
		level_buffer &= ~1;

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[11].Enabled_Inputs[0], Output_Pin[11].Input_Levels[0],
									 Output_Pin[11].Enabled_Inputs[1], Output_Pin[11].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 11) & 0x01) == 1)))
		level_buffer |= 1 << 1;
	else
		level_buffer &= ~(1 << 1);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[12].Enabled_Inputs[0], Output_Pin[12].Input_Levels[0],
	 	 	 	 	 	 	 	 	 Output_Pin[12].Enabled_Inputs[1], Output_Pin[12].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 12) & 0x01) == 1)))
		level_buffer |= 1 << 6;
	else
		level_buffer &= ~(1 << 6);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[13].Enabled_Inputs[0], Output_Pin[13].Input_Levels[0],
	 	 	 	 	 	 	 	 	 Output_Pin[13].Enabled_Inputs[1], Output_Pin[13].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 13) & 0x01) == 1)))
		level_buffer |= 1 << 7;
	else
		level_buffer &= ~(1 << 7);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[14].Enabled_Inputs[0], Output_Pin[14].Input_Levels[0],
	 	 	 	 	 	 	 	 	 Output_Pin[14].Enabled_Inputs[1], Output_Pin[14].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 14) & 0x01) == 1)))
		level_buffer |= 1 << 4;
	else
		level_buffer &= ~(1 << 4);

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[15].Enabled_Inputs[0], Output_Pin[15].Input_Levels[0],
									 Output_Pin[15].Enabled_Inputs[1], Output_Pin[15].Input_Levels[1])
									 && (((Driver_Safety_Flag >> 15) & 0x01) == 1)))
		level_buffer |= 1 << 5;
	else
		level_buffer &= ~(1 << 5);

	retVal = PCF8574A_Write_Pins(hi2c, level_buffer, 0);

	if(retVal == HAL_OK)
		return retVal;

	return PCF8574_Write_Pins(hi2c, level_buffer, 0);
}
#endif

static void PDM_Output_Set(uint8_t output_pin, uint8_t output_level)
{
	switch(output_pin)
	{
	case 4:
		HAL_GPIO_WritePin(OUTPUT5_GPIO_Port, OUTPUT5_Pin, output_level);
		break;

	case 5:
		HAL_GPIO_WritePin(OUTPUT6_GPIO_Port, OUTPUT6_Pin, output_level);
		break;

	case 6:
		HAL_GPIO_WritePin(OUTPUT7_GPIO_Port, OUTPUT7_Pin, output_level);
		break;

	case 7:
		HAL_GPIO_WritePin(OUTPUT8_GPIO_Port, OUTPUT8_Pin, output_level);
		break;

#ifndef LQFP64
	case 8:
		HAL_GPIO_WritePin(OUTPUT9_GPIO_Port, OUTPUT9_Pin, output_level);
		break;

	case 9:
		HAL_GPIO_WritePin(OUTPUT10_GPIO_Port, OUTPUT10_Pin, output_level);
		break;

	case 10:
		HAL_GPIO_WritePin(OUTPUT11_GPIO_Port, OUTPUT11_Pin, output_level);
		break;

	case 11:
		HAL_GPIO_WritePin(OUTPUT12_GPIO_Port, OUTPUT12_Pin, output_level);
		break;

	case 12:
		HAL_GPIO_WritePin(OUTPUT13_GPIO_Port, OUTPUT13_Pin, output_level);
		break;

	case 13:
		HAL_GPIO_WritePin(OUTPUT14_GPIO_Port, OUTPUT14_Pin, output_level);
		break;

	case 14:
		HAL_GPIO_WritePin(OUTPUT15_GPIO_Port, OUTPUT15_Pin, output_level);
		break;

	case 15:
		HAL_GPIO_WritePin(OUTPUT16_GPIO_Port, OUTPUT16_Pin, output_level);
		break;
#endif
	}

	return;
}

void PDM_Input_Process()
{
	Input_Pin_Levels = 0x0000;

	Input_Pin_Levels  = HAL_GPIO_ReadPin(INPUT1_GPIO_Port, INPUT1_Pin);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT2_GPIO_Port, INPUT2_Pin) << 1);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin) << 2);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT4_GPIO_Port, INPUT4_Pin) << 3);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT5_GPIO_Port, INPUT5_Pin) << 4);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT6_GPIO_Port, INPUT6_Pin) << 5);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT7_GPIO_Port, INPUT7_Pin) << 6);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT8_GPIO_Port, INPUT8_Pin) << 7);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT9_GPIO_Port, INPUT9_Pin) << 8);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT10_GPIO_Port, INPUT10_Pin) << 9);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT11_GPIO_Port, INPUT11_Pin) << 10);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT12_GPIO_Port, INPUT12_Pin) << 11);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT13_GPIO_Port, INPUT13_Pin) << 12);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT14_GPIO_Port, INPUT14_Pin) << 13);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT15_GPIO_Port, INPUT15_Pin) << 14);

	Input_Pin_Levels |= (HAL_GPIO_ReadPin(INPUT16_GPIO_Port, INPUT16_Pin) << 15);

	return;
}

void PDM_Output_Process()
{
#ifndef LQFP64
	for(uint8_t i = 4; i < NBR_OF_OUTPUTS; i++)
#else
	for(uint8_t i = 4; i < (NBR_OF_OUTPUTS - 8); i++)
#endif
	{
		if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[i].Enabled_Inputs[0], Output_Pin[i].Input_Levels[0],
										 Output_Pin[i].Enabled_Inputs[1], Output_Pin[i].Input_Levels[1]))
										 && (((Driver_Safety_Flag >> i) & 0x01) == 1))
		{
			PDM_Output_Set(i, GPIO_PIN_SET);
		}else{
			PDM_Output_Set(i, GPIO_PIN_RESET);
		}
	}

#ifdef LQFP64
	PDM_Output_Expander_Set(&hi2c1);
#endif

	PWM_Pin_Status &= 0xF0;

	for(uint8_t i = 0; i < 4; i++)
	{
		PDM_PWM_Output_Process(&PWM_Pins[i], i);
	}

	return;
}

void PDM_Output_Fuse()
{
	uint8_t fuse_flag = 0;

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		if(Data_Buffer[i] > Current_Thresholds[i])
		{
			Accumulator_Output_Fuse[i] += Accumulator_Output_Check;

			if(Accumulator_Output_Fuse[i] >= (Timeout_Output_Fuse[i] * 10))
			{
				Driver_Safety_Flag |= (1 << i);
				Accumulator_Output_Fuse[i] = 0;
				fuse_flag = 1;
			}

		}else{
			Driver_Overcurrent_Flag &= ~(1 << i);
			Accumulator_Output_Fuse[i] = 0;
		}
	}

	Accumulator_Output_Check -= OUTPUT_FUSE_FREQ;

	if(fuse_flag == 1)
		PDM_Output_Process();

	return;
}
