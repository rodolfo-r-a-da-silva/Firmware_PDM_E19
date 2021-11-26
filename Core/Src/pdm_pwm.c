/*
 * pdm_pwm.c
 *
 *  Created on: 20 de out de 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static int16_t Linear_Interpolation(int16_t x, int16_t x0, int16_t x1, int16_t y0, int16_t y1)
{
	return (((y1 - y0) * (x - x0)) / (x1 - x0)) + y0;
}

static int16_t Bilinear_Interpolation(int16_t x, int16_t y, int16_t x0, int16_t x1, int16_t y0, int16_t y1, int16_t z00, int16_t z01, int16_t z10, int16_t z11)
{
	int16_t z[2];

	z[0] = Linear_Interpolation(x, x0, x1, z00, z01);
	z[1] = Linear_Interpolation(x, x0, x1, z10, z11);

	return Linear_Interpolation(y, y0, y1, z[0], z[1]);
}

static void PDM_PWM_Duty_Cycle_Set(PWM_Control_Struct* pwm_struct)
{
	if((pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
		&& (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0]))
	{
		pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Map[0][0];
		return;
	}
	else if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Lim[0][1])
			 && (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0]))
	{
		pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Map[0][pwm_struct->Map_Lengths[0]];
		return;
	}
	else if((pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
			 && (pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Lim[1][1]))
	{
		pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][0];
		return;
	}
	else if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Lim[0][1])
			 && (pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Lim[1][1]))
	{
		pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][pwm_struct->Map_Lengths[0]];
		return;
	}

	if((pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
		|| (pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Lim[0][1]))
	{
		for(uint8_t y = 0; y < pwm_struct->Map_Lengths[1]; y++)
		{
			if((pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Step[1][y])
				&& (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Step[1][y + 1]))
			{
				if(pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
				{
					pwm_struct->Duty_Cycle = Linear_Interpolation(pwm_struct->Command_Var[1],
																  pwm_struct->Command_Var_Step[1][y],
																  pwm_struct->Command_Var_Step[1][y + 1],
																  pwm_struct->Duty_Cycle_Map[y][0],
																  pwm_struct->Duty_Cycle_Map[y + 1][0]);
				}else{
					pwm_struct->Duty_Cycle = Linear_Interpolation(pwm_struct->Command_Var[1],
																  pwm_struct->Command_Var_Step[1][y],
																  pwm_struct->Command_Var_Step[1][y + 1],
																  pwm_struct->Duty_Cycle_Map[y][pwm_struct->Map_Lengths[0]],
																  pwm_struct->Duty_Cycle_Map[y + 1][pwm_struct->Map_Lengths[0]]);
				}
				return;
			}
		}
	}

	if((pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0])
		|| (pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Lim[1][1]))
	{
		for(uint8_t x = 0; x < pwm_struct->Map_Lengths[0]; x++)
		{
			if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Step[0][x])
				&& (pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Step[0][x + 1]))
			{
				if(pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0])
				{
					pwm_struct->Duty_Cycle = Linear_Interpolation(pwm_struct->Command_Var[0],
																  pwm_struct->Command_Var_Step[0][x],
																  pwm_struct->Command_Var_Step[0][x + 1],
																  pwm_struct->Duty_Cycle_Map[0][x],
																  pwm_struct->Duty_Cycle_Map[0][x + 1]);
				}else{
					pwm_struct->Duty_Cycle = Linear_Interpolation(pwm_struct->Command_Var[0],
																  pwm_struct->Command_Var_Step[0][x],
																  pwm_struct->Command_Var_Step[0][x + 1],
																  pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][x],
																  pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][x + 1]);
				}
				return;
			}
		}
	}

	for(uint8_t x = 0; x < pwm_struct->Map_Lengths[0]; x++)
	{
		if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Step[0][x])
			&& (pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Step[0][x + 1]))
		{
			for(uint8_t y = 0; y < pwm_struct->Map_Lengths[1]; y++)
			{
				if((pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Step[1][y])
					&& (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Step[1][y + 1]))
				{
					pwm_struct->Duty_Cycle = Bilinear_Interpolation(pwm_struct->Command_Var[0],
																	pwm_struct->Command_Var[1],
																	pwm_struct->Command_Var_Step[0][x],
																	pwm_struct->Command_Var_Step[0][x + 1],
																	pwm_struct->Command_Var_Step[1][y],
																	pwm_struct->Command_Var_Step[1][y + 1],
																	pwm_struct->Duty_Cycle_Map[y][x],
																	pwm_struct->Duty_Cycle_Map[y][x + 1],
																	pwm_struct->Duty_Cycle_Map[y + 1][x],
																	pwm_struct->Duty_Cycle_Map[y + 1][x + 1]);
					return;
				}
			}
		}
	}
	return;
}

void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number)
{
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;

#ifdef LQFP64
	switch(pwm_out_number)
	{
	case 0:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_4);
		break;
	case 1:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_3);
		break;
	case 2:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_3);
		break;
	case 3:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_4);
		break;
	default:
		return;
	}
#else
	switch(pwm_out_number)
	{
	case 0:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_4);
		break;
	case 1:
		__PDM_PWM_SELECT_TIM(&htim8, TIM_CHANNEL_2);
		break;
	case 2:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_3);
		break;
	case 3:
		__PDM_PWM_SELECT_TIM(&htim1, TIM_CHANNEL_4);
		break;
	default:
		return;
	}
#endif

	for(uint8_t i = 0; i <= pwm_struct->Map_Lengths[0]; i++)
	{
		pwm_struct->Command_Var_Step[0][i] = (i * ((pwm_struct->Command_Var_Lim[0][1] - pwm_struct->Command_Var_Lim[0][0]) / pwm_struct->Map_Lengths[0])) + pwm_struct->Command_Var_Lim[0][0];
	}

	for(uint8_t j = 0; j <= pwm_struct->Map_Lengths[1]; j++)
	{
		pwm_struct->Command_Var_Step[1][j] = (j * ((pwm_struct->Command_Var_Lim[1][1] - pwm_struct->Command_Var_Lim[1][0]) / pwm_struct->Map_Lengths[1])) + pwm_struct->Command_Var_Lim[1][0];
	}

	HAL_TIM_PWM_Start(htim, tim_channel);

	__HAL_TIM_SET_AUTORELOAD(htim, pwm_struct->PWM_Frequency);

	Data_Verify_Buffer[26 + pwm_out_number] = (PWM_Pin_Status >> pwm_out_number) & 0x01;

	if(Data_Verify_Buffer[26 + pwm_out_number] == OUTPUT_PWM_ENABLE)
		PDM_PWM_CAN_Filter_Config(hcan, pwm_struct, pwm_out_number);

	PDM_PWM_Output_Process(pwm_struct, pwm_out_number);
}

void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number)
{
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;

#ifdef LQFP64
	switch(pwm_out_number)
	{
	case 0:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_4);
		break;
	case 1:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_3);
		break;
	case 2:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_3);
		break;
	case 3:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_4)
		break;
	default:
		return;
	}
#else
	switch(pwm_out_number)
	{
	case 0:
		__PDM_PWM_SELECT_TIM(&htim3, TIM_CHANNEL_4);
		break;
	case 1:
		__PDM_PWM_SELECT_TIM(&htim8, TIM_CHANNEL_2);
		break;
	case 2:
		__PDM_PWM_SELECT_TIM(&htim2, TIM_CHANNEL_3);
		break;
	case 3:
		__PDM_PWM_SELECT_TIM(&htim1, TIM_CHANNEL_4);
		break;
	default:
		return;
	}
#endif

	if((__PDM_INPUT_CONDITION_COMPARE(Output_Pin[pwm_out_number].Enabled_Inputs[0], Output_Pin[pwm_out_number].Input_Levels[0],
									 Output_Pin[pwm_out_number].Enabled_Inputs[1], Output_Pin[pwm_out_number].Input_Levels[1]))
									 && (((Driver_Safety_Flag >> pwm_out_number) & 0x01) == 1))
	{
		if(Data_Verify_Buffer[26 + pwm_out_number] == OUTPUT_PWM_ENABLE)
		{
			if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct[pwm_out_number].Input_DC_Preset_Enable[0], pwm_struct->Input_DC_Preset[0],
											 pwm_struct[pwm_out_number].Input_DC_Preset_Enable[1], pwm_struct->Input_DC_Preset[1]))
			{
				pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Preset[0];
			}
			else if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct[pwm_out_number].Input_DC_Preset_Enable[2], pwm_struct->Input_DC_Preset[2],
					 	 	 	 	 	 	 	  pwm_struct[pwm_out_number].Input_DC_Preset_Enable[3], pwm_struct->Input_DC_Preset[3]))
			{
				pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Preset[1];
			}else{
				PDM_PWM_Duty_Cycle_Set(pwm_struct);
			}
		}else{
			pwm_struct->Duty_Cycle = 1010;
		}
	}else{
		pwm_struct->Duty_Cycle = 0;
	}

	__HAL_TIM_SET_COMPARE(htim, tim_channel, (htim->Init.Period * pwm_struct->Duty_Cycle) / 1000);

	Data_Buffer[26 + pwm_out_number] = pwm_struct->Duty_Cycle;

	return;
}
