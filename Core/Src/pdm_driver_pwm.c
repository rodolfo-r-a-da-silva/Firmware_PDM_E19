/*
 * pdm_pwm.c
 *
 *  Created on: 20 de out de 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "stdlib.h"

static void PWM_DeAlloc(PWM_Control_Struct *pwm_struct);
static HAL_StatusTypeDef PWM_SoftStart(PWM_Control_Struct *pwm_struct);
static uint16_t PWM_Map_Duty_Cycle_Set(PDM_PWM_Map_Struct* pwm_map_struct);

//Initializes PWM output and sets its CAN bus filter
void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number)
{
	TIM_HandleTypeDef* htim;
	uint16_t timChannel = 0;
	uint16_t prescaler = 0;
	uint16_t memAddress = 0;

	//Sets the verify bit of the PWM output to sign if PWM is enabled
	dataIdBuffer[NBR_OF_DATA_CHANNELS - NBR_OF_PWM_OUTPUTS + pwm_out_number] |= (pwmPinStatus >> pwm_out_number) & 0x01;

	//Select TIM_HandleTypeDef and channel based on pwm_out_number
	__PDM_PWM_SELECT_TIM(pwm_out_number);

	if(pwm_struct->pwmFrequency == 0)
		pwm_struct->pwmFrequency = PWM_FREQ_100HZ;

	//Sets the PWM frequency
	if((htim->Instance == TIM1) || (htim->Instance == TIM8))
		prescaler = (pwm_struct->pwmFrequency * 2) + 1;
	else
		prescaler = pwm_struct->pwmFrequency;

	__HAL_TIM_SET_PRESCALER(htim, prescaler);

	//Zero the duty cycle
	pwm_struct->dutyCycle = 0;
	__HAL_TIM_SET_COMPARE(htim, timChannel, 0);

	//Deallocates all pointers
	PWM_DeAlloc(pwm_struct);

	if(pwm_struct->softStart == SoftStart_Enabled)
		PDM_PWM_Load_SoftStart_From_EEPROM(&hi2c1, pwm_struct, pwm_out_number);

	//Configures 3D map or Artificial Neural Network
	if(pwm_struct->outputType == OutType_Map)
		PDM_PWM_Map_Load_From_EEPROM(&hi2c1, pwm_struct, memAddress);

	//Deallocates all pointers if there is any allocation problem
	if(pwm_struct->outputType == OutType_Error)
		PWM_DeAlloc(pwm_struct);

	return;
}

//Process input conditions and command variables and sets the PWM output duty cycle
void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number, GPIO_PinState output_level)
{
	uint8_t softStart = 0;
	uint16_t timChannel;
	TIM_HandleTypeDef* htim;

	//Select TIM_HandleTypeDef and channel based on pwm_out_number
	__PDM_PWM_SELECT_TIM(pwm_out_number);

	if((output_level == GPIO_PIN_SET)
			&& (((flagDriverSafety >> pwm_out_number) & 0x01) == 0)
			&& (pwm_struct->outputType != OutType_Error))
	{
		if((pwm_struct->softStart == SoftStart_Enabled) && (pwm_struct->softStartStruct != NULL) && (pwm_struct->dutyCycle == 0))
			softStart = 1;

		//Sets duty cycle to 100% if the output is set as standard
		if(pwm_struct->outputType == OutType_Standard)
			pwm_struct->dutyCycle = 1000;

		//Checks if the inputs match the first PWM preset
		else if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct->presetEnable[0], pwm_struct->presetInputs[0], Output_Enabled))
			pwm_struct->dutyCycle = pwm_struct->presetDutyCycle[0];

		//Checks if the inputs match the second PWM preset
		else if(__PDM_INPUT_CONDITION_COMPARE(pwm_struct->presetEnable[1], pwm_struct->presetInputs[1], Output_Enabled))
			pwm_struct->dutyCycle = pwm_struct->presetDutyCycle[1];

		//Sets duty cycle based on the 3D map if enabled
		else if((pwm_struct->outputType == OutType_Map) && (pwm_struct->pwmMapStruct != NULL))
			pwm_struct->dutyCycle = PWM_Map_Duty_Cycle_Set(pwm_struct->pwmMapStruct);

		else
			pwm_struct->dutyCycle = 0;
	}

	else
		pwm_struct->dutyCycle = 0;

	if(softStart == 0)
		__HAL_TIM_SET_COMPARE(htim, timChannel, pwm_struct->dutyCycle);

	else
	{
		if(PWM_SoftStart(pwm_struct) == HAL_OK)
		{
			if(pwm_out_number == 1)
			{
				HAL_TIMEx_PWMN_Stop(htim, timChannel);
				HAL_TIMEx_PWMN_Start_DMA(htim, timChannel, (uint32_t*) pwm_struct->softStartStruct->dutyCycleBuffer, pwm_struct->softStartStruct->dutyCycles);
			}

			else
			{
				HAL_TIM_PWM_Stop(htim, timChannel);
				HAL_TIM_PWM_Start_DMA(htim, timChannel, (uint32_t*) pwm_struct->softStartStruct->dutyCycleBuffer, pwm_struct->softStartStruct->dutyCycles);
			}
		}

		else
			__HAL_TIM_SET_COMPARE(htim, timChannel, pwm_struct->dutyCycle);
	}

	//Stores output duty cycle inside the data buffer to be sent via CAN/USB
	dataBuffer[NBR_OF_DATA_CHANNELS - NBR_OF_PWM_OUTPUTS + pwm_out_number] = pwm_struct->dutyCycle;

	return;
}

static void PWM_DeAlloc(PWM_Control_Struct *pwm_struct)
{
	if(pwm_struct->softStartStruct != NULL)
	{

		if(pwm_struct->softStartStruct->dutyCycleBuffer != NULL)
		{
			free(pwm_struct->softStartStruct->dutyCycleBuffer);
			pwm_struct->softStartStruct->dutyCycleBuffer = NULL;
		}

		free(pwm_struct->softStartStruct);
		pwm_struct->softStartStruct = NULL;
		pwm_struct->softStart = SoftStart_Disabled;
	}

	if(pwm_struct->pwmMapStruct != NULL)
	{
		if(pwm_struct->pwmMapStruct->commandVarStep[0] != NULL)
		{
			free(pwm_struct->pwmMapStruct->commandVarStep[0]);
			pwm_struct->pwmMapStruct->commandVarStep[0] = NULL;
		}

		if(pwm_struct->pwmMapStruct->commandVarStep[1] != NULL)
		{
			free(pwm_struct->pwmMapStruct->commandVarStep[1]);
			pwm_struct->pwmMapStruct->commandVarStep[1] = NULL;
		}

		for(uint8_t i = 0; i < pwm_struct->pwmMapStruct->mapLengths[0]; i++)
		{
			if(pwm_struct->pwmMapStruct->dutyCycleMap[i] != NULL)
			{
				free(pwm_struct->pwmMapStruct->dutyCycleMap[i]);
				pwm_struct->pwmMapStruct->dutyCycleMap[i] = NULL;
			}
		}

		if(pwm_struct->pwmMapStruct->dutyCycleMap != NULL)
		{
			free(pwm_struct->pwmMapStruct->dutyCycleMap);
			pwm_struct->pwmMapStruct->dutyCycleMap = NULL;
		}

		free(pwm_struct->pwmMapStruct);
		pwm_struct->pwmMapStruct = NULL;
	}

	if((pwm_struct->presetEnable[0] != 0)
			|| (pwm_struct->presetEnable[1] != 0))
		pwm_struct->outputType = OutType_Preset;

	else
		pwm_struct->outputType = OutType_Standard;

	return;
}

//Sets the PWM soft start buffer
static HAL_StatusTypeDef PWM_SoftStart(PWM_Control_Struct *pwm_struct)
{
	HAL_StatusTypeDef retVal = HAL_OK;

	pwm_struct->softStartStruct->dutyCycles = (pwm_struct->softStartStruct->slope * pwm_struct->dutyCycle) / 1000;

	pwm_struct->softStartStruct->dutyCycleBuffer = malloc(pwm_struct->softStartStruct->dutyCycles * sizeof(uint16_t));

	if(pwm_struct->softStartStruct->dutyCycleBuffer != NULL)
	{
		for(uint16_t i = 0; i < pwm_struct->softStartStruct->dutyCycles; i++)
			pwm_struct->softStartStruct->dutyCycleBuffer[i] = __PDM_LINEAR_INTERPOLATION(i, 0, pwm_struct->softStartStruct->dutyCycles,
																							0, pwm_struct->dutyCycle);

		retVal = HAL_OK;
	}

	else
		retVal = HAL_ERROR;

	return retVal;
}

//Sets PWM output duty cycle using its command variables
static uint16_t PWM_Map_Duty_Cycle_Set(PDM_PWM_Map_Struct* pwm_map_struct)
{
	uint16_t retVal = 0;

	//Checks if both command variables out of the column and line limits and attributes the map's closest corner value
	if((pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][0])
		&& (pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][0]))

		retVal = pwm_map_struct->dutyCycleMap[0][0];

	else if((pwm_map_struct->commandVar[0] >= pwm_map_struct->commandVarStep[0][pwm_map_struct->mapLengths[0] - 1])
			 && (pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][0]))

		retVal = pwm_map_struct->dutyCycleMap[pwm_map_struct->mapLengths[0] - 1][0];

	else if((pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][0])
			 && (pwm_map_struct->commandVar[1] >= pwm_map_struct->commandVarStep[1][pwm_map_struct->mapLengths[1] - 1]))

		retVal = pwm_map_struct->dutyCycleMap[0][pwm_map_struct->mapLengths[1] - 1];

	else if((pwm_map_struct->commandVar[0] >= pwm_map_struct->commandVarStep[0][pwm_map_struct->mapLengths[0] - 1])
			 && (pwm_map_struct->commandVar[1] >= pwm_map_struct->commandVarStep[1][pwm_map_struct->mapLengths[1] - 1]))

		retVal = pwm_map_struct->dutyCycleMap[pwm_map_struct->mapLengths[0] - 1][pwm_map_struct->mapLengths[1] - 1];

	//Check if the command variable point is outside the lines (y limits) of the 3D map or there is only 1 variable input
	else if((pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][0])
			|| (pwm_map_struct->commandVar[1] >= pwm_map_struct->commandVarStep[1][pwm_map_struct->mapLengths[1] - 1])
			|| (pwm_map_struct->mapLengths[1] == 1))
	{
		for(uint8_t x = 0; x < (pwm_map_struct->mapLengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((pwm_map_struct->commandVar[0] >= pwm_map_struct->commandVarStep[0][x])
				&& (pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][x + 1]))
			{
				//Checks if the command variable point is above or below the lines (y limits) of the 3D map then sets duty cycle via linear interpolation
				if((pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][0]) || (pwm_map_struct->mapLengths[1] == 1))
				{
					retVal = __PDM_LINEAR_INTERPOLATION(pwm_map_struct->commandVar[0],
																  	   pwm_map_struct->commandVarStep[0][x],
																	   pwm_map_struct->commandVarStep[0][x + 1],
																	   pwm_map_struct->dutyCycleMap[x][0],
																	   pwm_map_struct->dutyCycleMap[x + 1][0]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(pwm_map_struct->commandVar[0],
																  	   pwm_map_struct->commandVarStep[0][x],
																	   pwm_map_struct->commandVarStep[0][x + 1],
																	   pwm_map_struct->dutyCycleMap[x][pwm_map_struct->mapLengths[1] - 1],
																	   pwm_map_struct->dutyCycleMap[x + 1][pwm_map_struct->mapLengths[1] - 1]);
				}
			}
		}
	}

	//Check if the command variable point is outside the columns (x limits) of the 3D map
	else if((pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][0])
			|| (pwm_map_struct->commandVar[0] >= pwm_map_struct->commandVarStep[0][pwm_map_struct->mapLengths[0] - 1]))
	{
		for(uint8_t y = 0; y < (pwm_map_struct->mapLengths[1] - 1); y++)
		{
			//Checks if the command variable point is inside the y, y + 1 line
			if((pwm_map_struct->commandVar[1] >= pwm_map_struct->commandVarStep[1][y])
				&& (pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][y + 1]))
			{
				//Checks if the command variable point is to the left or to the right of the 3D map then sets duty cycle via linear interpolation
				if(pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][0])
				{
					retVal = __PDM_LINEAR_INTERPOLATION(pwm_map_struct->commandVar[1],
																  	   pwm_map_struct->commandVarStep[1][y],
																	   pwm_map_struct->commandVarStep[1][y + 1],
																	   pwm_map_struct->dutyCycleMap[0][y],
																	   pwm_map_struct->dutyCycleMap[0][y + 1]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(pwm_map_struct->commandVar[1],
																  	   pwm_map_struct->commandVarStep[1][y],
																	   pwm_map_struct->commandVarStep[1][y + 1],
																	   pwm_map_struct->dutyCycleMap[pwm_map_struct->mapLengths[0] - 1][y],
																	   pwm_map_struct->dutyCycleMap[pwm_map_struct->mapLengths[0] - 1][y + 1]);
				}
			}
		}
	}

	//Since the command variable point is inside the map's boundary, sets duty cycle via bilinear interpolation
	else
	{
		for(uint8_t x = 0; x < (pwm_map_struct->mapLengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((pwm_map_struct->commandVar[0] >= pwm_map_struct->commandVarStep[0][x])
				&& (pwm_map_struct->commandVar[0] <= pwm_map_struct->commandVarStep[0][x + 1]))
			{
				for(uint8_t y = 0; y < (pwm_map_struct->mapLengths[1] - 1); y++)
				{
					//Checks if the command variable point is inside the y, y + 1 line
					if((pwm_map_struct->commandVar[1] >= pwm_map_struct->commandVarStep[1][y])
						&& (pwm_map_struct->commandVar[1] <= pwm_map_struct->commandVarStep[1][y + 1]))
					{
						retVal = __PDM_BILINEAR_INTERPOLATION(pwm_map_struct->commandVar[0],
																		  	 pwm_map_struct->commandVar[1],
																		     pwm_map_struct->commandVarStep[0][x],
																		     pwm_map_struct->commandVarStep[0][x + 1],
																		     pwm_map_struct->commandVarStep[1][y],
																		     pwm_map_struct->commandVarStep[1][y + 1],
																		     pwm_map_struct->dutyCycleMap[x][y],
																		     pwm_map_struct->dutyCycleMap[x + 1][y],
																		     pwm_map_struct->dutyCycleMap[x][y + 1],
																		     pwm_map_struct->dutyCycleMap[x + 1][y + 1]);
					}
				}
			}
		}
	}

	return retVal;
}
