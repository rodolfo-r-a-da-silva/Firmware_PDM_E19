/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

static void Output_Set_State(PDM_Output_Ctrl_Struct* outStruct, uint16_t* dataBuffer);
static void PWM_Set_DutyCycle(PDM_PWM_Ctrl_Struct* pwmStruct);
static void PWM_Set_Soft_Start(PDM_PWM_Ctrl_Struct* pwmStruct, uint16_t dutyCycle);
static uint16_t PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* mapStruct);

/*BEGIN THREADS*/

//Reads input pin levels, checks CAN data and sets output levels
void PDM_Output_Thread(void* threadStruct)
{
	//Struct containing each input and output structure, CAN data Queue handle and semaphore handle
	PDM_OutSet_Thread_Struct* thrdStr = (PDM_OutSet_Thread_Struct*) threadStruct;

	uint32_t inputLevels;	//Stores each input pin logical level

	//CAN reception and storage
	uint8_t canDlc;
	uint8_t canIde;
	uint8_t canRxData[8];
	uint32_t canId;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{


#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREAD*/

/*START STATIC FUNCTIONS*/

static void Output_Set_State(PDM_Output_Ctrl_Struct* outStruct, uint16_t* dataBuffer)
{
	uint8_t j = Data_PWM1;	//Index for PWM data inside Thread safe data buffer

	//Zero output data inside data buffer
	dataBuffer[Data_Output] = 0;

	//Set each output state
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		//Set output level or PWM duty cycle
		if(outStruct->outputHardware == Output_GPIO)
		{
			if((*outStruct[i]->inputFunc == Result_True)
					&& (outStruct[i]->inputFunc != NULL)
					&& (outStruct[i]->fuseStruct->status != Fuse_Open))
				outStruct[i]->outputState = GPIO_PIN_SET;

			else
				outStruct[i]->outputState = GPIO_PIN_RESET;

			__PDM_OUT_SET_LEVEL(outStruct[i]);

			//Place output state inside data buffer
			dataBuffer[Data_Output] |= (outStruct[i]->outputState << i);
		}

		else if((outStruct->outputHardware == Output_PWM) || (outStruct->outputHardware == Output_PWMN))
		{
			//Set output PWM if fuse isn't open, else set it to minimum
			if(outStruct[i]->fuseStruct->status != Fuse_Open)
			{
				if((*outStruct[i]->inputFunc == Result_True)
						&& (outStruct[i]->inputFunc != NULL))
					outStruct[i]->pwmStruct->dutyCycle = PWM_MAX_DUTY_CYCLE;

				else
					PWM_Set_DutyCycle(outStruct[i]->pwmStruct, outStruct[i]->outputState);
			}

			else
				outStruct[i]->pwmStruct->dutyCycle = PWM_MAX_DUTY_CYCLE;

			__PDM_PWM_SET_COMPARE(outStruct[i]);

			//Place PWM Duty Cycle inside data buffer
			dataBuffer[j++] = outStruct[i]->pwmStruct->dutyCycle;

			//Place output state inside data buffer
			if(outStruct[i]->pwmStruct->dutyCycle > PWM_MIN_DUTY_CYCLE)
				dataBuffer[Data_Output] |= (GPIO_PIN_SET << i);
		}
	}

	return;
}

//Process input conditions and command variables and sets the PWM output duty cycle
//PDM_PWM_Ctrl_Struct* pwmStruct - Pointer to struct containing PWM configuration and data
//outLevel - Output pin level
static void PWM_Set_DutyCycle(PDM_PWM_Ctrl_Struct* pwmStruct)
{
	//Auxiliary variable to set duty cycle
	uint16_t dutyCycle = PWM_MIN_DUTY_CYCLE;

	//Check each PWM preset value
	for(uint8_t i = 0; i < PWM_NBR_OF_PRESETS; i++)
	{
		if((*pwmStruct->presetInputs[i] == Result_True)
				&& (pwmStruct->presetInputs[i] != NULL))
		{
			dutyCycle = pwmStruct->presetDutyCycle[i];
			break;
		}
	}

	//Check 3D map and ANN if no preset was activated
	if(dutyCycle == PWM_MIN_DUTY_CYCLE)
	{
		if(pwmStruct->mapStruct != NULL)
			dutyCycle = PWM_Set_Map_DutyCycle(pwmStruct->mapStruct);

		else if(pwmStruct->annStruct != NULL)
			1;
	}

	//Check soft start enabled status and condition
	if(pwmStruct->ssStruct == NULL)
	{
		pwmStruct->dutyCycle = dutyCycle;
		__HAL_TIM_SET_COMPARE(pwmStruct->htim, pwmStruct->timChannel, dutyCycle);
	}

	else
		PWM_Set_Soft_Start(pwmStruct, dutyCycle);

	return;
}

//Set duty cycle buffer and start PWM Timer DMA transfer
//PDM_PWM_Ctrl_Struct* pwmStruct - Pointer to PWM struct
//uint16_t dutyCycle - Final duty cycle value
static void PWM_Set_Soft_Start(PDM_PWM_Ctrl_Struct* pwmStruct, uint16_t dutyCycle)
{
	//Number of cycles until correct duty cycle is reached
	uint16_t cycles = 0;

	if((pwmStruct->ssStruct->thresholds[SoftStart_Current] <= pwmStruct->dutyCycle)
			&& (pwmStruct->ssStruct->thresholds[SoftStart_Next] <= dutyCycle))
	{
		for(; pwmStruct->ssStruct->buffer[cycles] < dutyCycle; cycles++)
		{
			pwmStruct->ssStruct->buffer[cycles] = __PDM_LINEAR_INTERPOLATION(cycles, 0, pwmStruct->ssStruct->nbrOfCycles,
																			 PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);
		}

		pwmStruct->ssStruct->buffer[cycles++] = dutyCycle;

		HAL_TIM_PWM_Start_DMA(pwmStruct->htim, pwmStruct->timChannel, pwmStruct->ssStruct->buffer, cycles);
	}

	else
	{
		pwmStruct->dutyCycle = dutyCycle;
		__HAL_TIM_SET_COMPARE(pwmStruct->htim, pwmStruct->timChannel, dutyCycle);
	}

	return;
}

//Sets PWM output duty cycle using its command variables
//PDM_PWM_Map_Struct* mapStruct - Pointer to struct containing PWM map data
static uint16_t PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* mapStruct)
{
	uint16_t retVal = 0;

	//Checks if both command variables out of the column and line limits and attributes the map's closest corner value
	if((mapStruct->inputs[0] <= mapStruct->axis[0][0])
		&& (mapStruct->inputs[1] <= mapStruct->axis[1][0]))

		retVal = mapStruct->map[0][0];

	else if((mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1])
			 && (mapStruct->inputs[1] <= mapStruct->axis[1][0]))

		retVal = mapStruct->map[mapStruct->lengths[0] - 1][0];

	else if((mapStruct->inputs[0] <= mapStruct->axis[0][0])
			 && (mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1]))

		retVal = mapStruct->map[0][mapStruct->lengths[1] - 1];

	else if((mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1])
			 && (mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1]))

		retVal = mapStruct->map[mapStruct->lengths[0] - 1][mapStruct->lengths[1] - 1];

	//Check if the command variable point is outside the lines (y limits) of the 3D map or there is only 1 variable input
	else if((mapStruct->inputs[1] <= mapStruct->axis[1][0])
			|| (mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1])
			|| (mapStruct->lengths[1] == 1))
	{
		for(uint8_t x = 0; x < (mapStruct->lengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((mapStruct->inputs[0] >= mapStruct->axis[0][x])
				&& (mapStruct->inputs[0] <= mapStruct->axis[0][x + 1]))
			{
				//Checks if the command variable point is above or below the lines (y limits) of the 3D map
				//hen sets duty cycle via linear interpolation
				if((mapStruct->inputs[1] <= mapStruct->axis[1][0]) || (mapStruct->lengths[1] == 1))
				{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->inputs[0],
																  	   mapStruct->axis[0][x],
																	   mapStruct->axis[0][x + 1],
																	   mapStruct->map[x][0],
																	   mapStruct->map[x + 1][0]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->inputs[0],
																  	   mapStruct->axis[0][x],
																	   mapStruct->axis[0][x + 1],
																	   mapStruct->map[x][mapStruct->lengths[1] - 1],
																	   mapStruct->map[x + 1][mapStruct->lengths[1] - 1]);
				}
			}
		}
	}

	//Check if the command variable point is outside the columns (x limits) of the 3D map
	else if((mapStruct->inputs[0] <= mapStruct->axis[0][0])
			|| (mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1]))
	{
		for(uint8_t y = 0; y < (mapStruct->lengths[1] - 1); y++)
		{
			//Checks if the command variable point is inside the y, y + 1 line
			if((mapStruct->inputs[1] >= mapStruct->axis[1][y])
				&& (mapStruct->inputs[1] <= mapStruct->axis[1][y + 1]))
			{
				//Checks if the command variable point is to the left or to the right of the 3D map
				//Then sets duty cycle via linear interpolation
				if(mapStruct->inputs[0] <= mapStruct->axis[0][0])
				{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->inputs[1],
																  	   mapStruct->axis[1][y],
																	   mapStruct->axis[1][y + 1],
																	   mapStruct->map[0][y],
																	   mapStruct->map[0][y + 1]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->inputs[1],
																  	   mapStruct->axis[1][y],
																	   mapStruct->axis[1][y + 1],
																	   mapStruct->map[mapStruct->lengths[0] - 1][y],
																	   mapStruct->map[mapStruct->lengths[0] - 1][y + 1]);
				}
			}
		}
	}

	//Since the command variable point is inside the map's boundary, sets duty cycle via bilinear interpolation
	else
	{
		for(uint8_t x = 0; x < (mapStruct->lengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((mapStruct->inputs[0] >= mapStruct->axis[0][x])
				&& (mapStruct->inputs[0] <= mapStruct->axis[0][x + 1]))
			{
				for(uint8_t y = 0; y < (mapStruct->lengths[1] - 1); y++)
				{
					//Checks if the command variable point is inside the y, y + 1 line
					if((mapStruct->inputs[1] >= mapStruct->axis[1][y])
						&& (mapStruct->inputs[1] <= mapStruct->axis[1][y + 1]))
					{
						retVal = __PDM_BILINEAR_INTERPOLATION(mapStruct->inputs[0],
																		  	 mapStruct->inputs[1],
																		     mapStruct->axis[0][x],
																		     mapStruct->axis[0][x + 1],
																		     mapStruct->axis[1][y],
																		     mapStruct->axis[1][y + 1],
																		     mapStruct->map[x][y],
																		     mapStruct->map[x + 1][y],
																		     mapStruct->map[x][y + 1],
																		     mapStruct->map[x + 1][y + 1]);
					}
				}
			}
		}
	}

	return retVal;
}

/*END STATIC FUNCTIONS*/
