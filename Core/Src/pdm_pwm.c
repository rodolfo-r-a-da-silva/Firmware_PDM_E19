/*
 * pdm_pwm.c
 *
 *  Created on: 20 de out de 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

//static int16_t Linear_Interpolation(int16_t x, int16_t x0, int16_t x1, int16_t y0, int16_t y1)
//{
//	return (((y1 - y0) * (x - x0)) / (x1 - x0)) + y0;
//}

//static int16_t Bilinear_Interpolation(int16_t x, int16_t y, int16_t x0, int16_t x1, int16_t y0, int16_t y1, int16_t z00, int16_t z01, int16_t z10, int16_t z11)
//{
//	int16_t z[2];
//
//	z[0] = __PDM_LINEAR_INTERPOLATION(x, x0, x1, z00, z01);
//	z[1] = __PDM_LINEAR_INTERPOLATION(x, x0, x1, z10, z11);
//
//	return __PDM_LINEAR_INTERPOLATION(y, y0, y1, z[0], z[1]);
//}

//Sets PWM output duty cycle using its command variables
static void PDM_PWM_Duty_Cycle_Set(PWM_Control_Struct* pwm_struct)
{
	//Checks if both command variables are above the collum and line limits and attributes the map's closest corner value
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

	//Check if the command variable point is outside the collums (x limits) of the 3D map
	if((pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
		|| (pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Lim[0][1]))
	{
		for(uint8_t y = 0; y < pwm_struct->Map_Lengths[1]; y++)
		{
			//Checks if the command variable point is inside the y, y + 1 line
			if((pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Step[1][y])
				&& (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Step[1][y + 1]))
			{
				//Checks if the command variable point is to the left or to the right of the 3D map then sets duty cycle via linear interpolation
				if(pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Lim[0][0])
				{
					pwm_struct->Duty_Cycle = __PDM_LINEAR_INTERPOLATION(pwm_struct->Command_Var[1],
																  	    pwm_struct->Command_Var_Step[1][y],
																		pwm_struct->Command_Var_Step[1][y + 1],
																		pwm_struct->Duty_Cycle_Map[y][0],
																		pwm_struct->Duty_Cycle_Map[y + 1][0]);
				}else{
					pwm_struct->Duty_Cycle = __PDM_LINEAR_INTERPOLATION(pwm_struct->Command_Var[1],
																  	    pwm_struct->Command_Var_Step[1][y],
																		pwm_struct->Command_Var_Step[1][y + 1],
																		pwm_struct->Duty_Cycle_Map[y][pwm_struct->Map_Lengths[0]],
																		pwm_struct->Duty_Cycle_Map[y + 1][pwm_struct->Map_Lengths[0]]);
				}
				return;
			}
		}
	}

	//Check if the command variable point is outside the lines (y limits) of the 3D map
	if((pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0])
		|| (pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Lim[1][1]))
	{
		for(uint8_t x = 0; x < pwm_struct->Map_Lengths[0]; x++)
		{
			//Checks if the command variable point is inside the x, x + 1 collum
			if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Step[0][x])
				&& (pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Step[0][x + 1]))
			{
				//Checks if the command variable point is above or below the lines (y limits) of the 3D map then sets duty cycle via linear interpolation
				if(pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Lim[1][0])
				{
					pwm_struct->Duty_Cycle = __PDM_LINEAR_INTERPOLATION(pwm_struct->Command_Var[0],
																  	    pwm_struct->Command_Var_Step[0][x],
																		pwm_struct->Command_Var_Step[0][x + 1],
																		pwm_struct->Duty_Cycle_Map[0][x],
																		pwm_struct->Duty_Cycle_Map[0][x + 1]);
				}else{
					pwm_struct->Duty_Cycle = __PDM_LINEAR_INTERPOLATION(pwm_struct->Command_Var[0],
																  	    pwm_struct->Command_Var_Step[0][x],
																		pwm_struct->Command_Var_Step[0][x + 1],
																		pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][x],
																		pwm_struct->Duty_Cycle_Map[pwm_struct->Map_Lengths[1]][x + 1]);
				}
				return;
			}
		}
	}

	//Since the command variable point is inside the map's boundary, sets duty cycle via bilinear interpolation
	for(uint8_t x = 0; x < pwm_struct->Map_Lengths[0]; x++)
	{
		//Checks if the command variable point is inside the x, x + 1 collum
		if((pwm_struct->Command_Var[0] >= pwm_struct->Command_Var_Step[0][x])
			&& (pwm_struct->Command_Var[0] <= pwm_struct->Command_Var_Step[0][x + 1]))
		{
			for(uint8_t y = 0; y < pwm_struct->Map_Lengths[1]; y++)
			{
				//Checks if the command variable point is inside the y, y + 1 line
				if((pwm_struct->Command_Var[1] >= pwm_struct->Command_Var_Step[1][y])
					&& (pwm_struct->Command_Var[1] <= pwm_struct->Command_Var_Step[1][y + 1]))
				{
					pwm_struct->Duty_Cycle = __PDM_BILINEAR_INTERPOLATION(pwm_struct->Command_Var[0],
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

//Initializes PWM output and sets its CAN bus filter
void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number)
{
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;

	//Sets the verify bit of the PWM output to sign if PWM is enabled
	Data_ID_Buffer[NBR_OF_DATA_CHANNELS - NBR_OF_PWM_OUTPUTS + pwm_out_number] |= (PWM_Pin_Status >> pwm_out_number) & 0x01;

	//Select TIM_HandleTypeDef and channel based on pwm_out_number
	__PDM_PWM_SELECT_TIM(pwm_out_number);

	//Sets PWM parameters if PWM is enabled
	if(((PWM_Pin_Status >> pwm_out_number) & 0x01) == OUTPUT_PWM_ENABLE)
	{
		//Sets the PWM frequency
		__HAL_TIM_SET_AUTORELOAD(htim, pwm_struct->PWM_Frequency);

		//Sets CAN filter and duty cycle map steps if PWM CAN is enabled and map lengths are bigger than zero
		if((((PWM_Pin_Status >> pwm_out_number) & 0x10) == OUTPUT_PWM_CAN_ENABLE) && (pwm_struct->Map_Lengths[0] != 0) && (pwm_struct->Map_Lengths[1] != 0))
		{
			PDM_PWM_CAN_Filter_Config(hcan, pwm_struct, pwm_out_number);

			for(uint16_t i = 0; i <= pwm_struct->Map_Lengths[0]; i++)
				pwm_struct->Command_Var_Step[0][i] = ((i * (pwm_struct->Command_Var_Lim[0][1] - pwm_struct->Command_Var_Lim[0][0])) / pwm_struct->Map_Lengths[0]) + pwm_struct->Command_Var_Lim[0][0];

			for(uint16_t j = 0; j <= pwm_struct->Map_Lengths[1]; j++)
				pwm_struct->Command_Var_Step[1][j] = ((j * (pwm_struct->Command_Var_Lim[1][1] - pwm_struct->Command_Var_Lim[1][0])) / pwm_struct->Map_Lengths[1]) + pwm_struct->Command_Var_Lim[1][0];
		}
	}

	//Start the PWM timer
	HAL_TIM_PWM_Start(htim, tim_channel);

	return;
}

//Process input conditions and command variables and sets the PWM output duty cycle
void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number)
{
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;

	//Select TIM_HandleTypeDef and channel based on pwm_out_number
	__PDM_PWM_SELECT_TIM(pwm_out_number);

	//Check if virtual fuse isn't tripped and if the input pins match their enabled states
	if((((Driver_Safety_Flag >> pwm_out_number) & 0x0001) == 0)
			&& (__PDM_INPUT_CONDITION_COMPARE(Output_Pin[pwm_out_number].Enabled_Inputs[0], Output_Pin[pwm_out_number].Input_Levels[0])
			||  __PDM_INPUT_CONDITION_COMPARE(Output_Pin[pwm_out_number].Enabled_Inputs[1], Output_Pin[pwm_out_number].Input_Levels[1])))
	{
		//Checks if the output is enabled as PWM, if i isn't, just sets it at 100%
		if(((PWM_Pin_Status >> pwm_out_number) & 0x01) == OUTPUT_PWM_ENABLE)
		{
			//Checks if the inputs match the first PWM preset
			if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct[pwm_out_number].Input_DC_Preset_Enable[0], pwm_struct->Input_DC_Preset[0]))
				pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Preset[0];

			//Checks if the inputs match the second PWM preset
			else if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct[pwm_out_number].Input_DC_Preset_Enable[1], pwm_struct->Input_DC_Preset[1]))
				pwm_struct->Duty_Cycle = pwm_struct->Duty_Cycle_Preset[1];

			//If no preset is matched, set PWM duty cycle using the 3D map
			else if(((PWM_Pin_Status >> pwm_out_number) & 0x10) == OUTPUT_PWM_CAN_ENABLE)
				PDM_PWM_Duty_Cycle_Set(pwm_struct);

		}else{
			pwm_struct->Duty_Cycle = 1010;
		}
	}else
		pwm_struct->Duty_Cycle = 0;


	__HAL_TIM_SET_COMPARE(htim, tim_channel, (htim->Init.Period * pwm_struct->Duty_Cycle) / 1000);

	//Stores output duty cycle inside the data buffer to be sent via CAN/USB
	Data_Buffer[NBR_OF_DATA_CHANNELS - NBR_OF_PWM_OUTPUTS + pwm_out_number] = pwm_struct->Duty_Cycle;

	return;
}
