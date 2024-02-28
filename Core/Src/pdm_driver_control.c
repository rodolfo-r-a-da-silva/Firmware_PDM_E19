/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

static void Output_Set_Level(PDM_Output_Ctrl_Struct* outStruct);
static void Output_Set_PWM(PDM_PWM_Ctrl_Struct* pwmStruct);
static uint8_t PWM_Set_Soft_Start(PDM_PWM_Ctrl_Struct* pwmStruct, uint16_t dutyCycle);
static uint16_t PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* mapStruct);

/*BEGIN THREADS*/

//Reads input pin levels, checks CAN data and sets output levels
void PDM_Output_Thread(void* threadStruct)
{
	//Struct containing each input and output structure, CAN data Queue handle and semaphore handle
	PDM_OutSet_Thread_Struct* thrdStr = (PDM_OutSet_Thread_Struct*) threadStruct;

	//Indicate origin from interrupt
	uint8_t processFlag = 0;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{

		if(osMessageQueueGet(thrdStr->outQueueHandle, (void*) &processFlag, 0, osWaitForever) == osOK)
		{
			if((processFlag & PROCESS_OUTPUT) == PROCESS_OUTPUT)
				Output_Set_Level(thrdStr->outStruct);

			if((processFlag & PROCESS_PWM) == PROCESS_PWM)
				Output_Set_PWM(thrdStr->pwmStruct);

	//		else if((processFlag & PROCESS_PWM_SS) == PROCESS_PWM_SS)
		}


#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREAD*/

/*START STATIC FUNCTIONS*/

//Set output level or PWM duty cycle
//PDM_Output_Ctrl_Struct* outStruct - Pointer to array of output structs
//int16_t* dataBuffer - Pointer to array of data transmitted via CAN bus
static void Output_Set_Level(PDM_Output_Ctrl_Struct* outStruct)
{
	//Set each output state
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		//Set output state variable
		if((*outStruct[i].inputFunc == Result_True)
				&& (outStruct[i].inputFunc != NULL)
				&& (*outStruct[i].fuseStatus != Fuse_Open))
			outStruct[i].outputState = GPIO_PIN_SET;

		else
			outStruct[i].outputState = GPIO_PIN_RESET;

		//Set level for standard GPIO output
		if(outStruct[i].outputHardware == Output_GPIO)
			__PDM_OUT_SET_LEVEL(outStruct[i]);
	}

	return;
}

static void Output_Set_PWM(PDM_PWM_Ctrl_Struct* pwmStruct)
{
	uint16_t dutyCycle;

	for(uint8_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		//Set storage variable to minimum for comparison purposes
		dutyCycle = PWM_MIN_DUTY_CYCLE;

		if(*pwmStruct[i].fuseStatus != Fuse_Open)
		{
			//Set maximum value if standard output function is true
			if(**pwmStruct[i].stdOutput == Result_True)
				dutyCycle = PWM_MAX_DUTY_CYCLE;

			//Set value based on preset
			else
			{
				for(uint8_t j = 0; j < PWM_NBR_OF_PRESETS; j++)
				{
					if((*pwmStruct[i].presetInputs[j] == Result_True)
							&& (pwmStruct[i].presetInputs[j] != NULL))
					{
						dutyCycle = pwmStruct[i].presetDutyCycle[j];
						break;
					}
				}
			}

			//Check if there was no activation yet
			if(dutyCycle == PWM_MIN_DUTY_CYCLE)
			{
				//Activate via 3D map
				if(pwmStruct[i].mapStruct != NULL)
					dutyCycle = PWM_Set_Map_DutyCycle(pwmStruct[i].mapStruct);
			}

			//Begin soft start cycle or set duty cycle based on previous conditions
			if(PWM_Set_Soft_Start(&pwmStruct[i], dutyCycle) == 0)
			{
				pwmStruct[i].dutyCycle = dutyCycle;

				//Set output duty cycle value inside Timer register
				__PDM_PWM_SET_COMPARE(pwmStruct[i]);
			}
		}

		//Set PWM duty cycle to minimum if fuse is open
		else
		{
			//Stop soft start DMA transfer
			HAL_DMA_Abort_IT(pwmStruct[i].hdma);
			pwmStruct[i].dutyCycle = PWM_MIN_DUTY_CYCLE;

			//Set output duty cycle value inside Timer register
			__PDM_PWM_SET_COMPARE(pwmStruct[i]);
		}

		//Set corresponding output struct output state
		if(pwmStruct[i].dutyCycle != PWM_MIN_DUTY_CYCLE)
			*pwmStruct[i].outState = GPIO_PIN_SET;

		else
			*pwmStruct[i].outState = GPIO_PIN_RESET;
	}

	return;
}

//Set duty cycle buffer and start PWM Timer DMA transfer
//PDM_PWM_Ctrl_Struct* pwmStruct - Pointer to PWM struct
//uint16_t dutyCycle - Final duty cycle value
static uint8_t PWM_Set_Soft_Start(PDM_PWM_Ctrl_Struct* pwmStruct, uint16_t dutyCycle)
{
	uint8_t retVal = 0;	//Tell if soft start was used
	uint8_t start = 0;  //Starting position of duty cycle buffer
	uint8_t finish = 0; //Finishing position of duty cycle buffer

	//Current PWM duty cycle
	uint16_t currentDutyCycle = __HAL_TIM_GET_COMPARE(pwmStruct->htim, pwmStruct->timChannel);

	//Execute if there is a valid soft start struct
	if((pwmStruct->ssStruct != NULL)
			&& (pwmStruct->ssStruct->thresholds[SoftStart_Current] <= currentDutyCycle)
			&& (pwmStruct->ssStruct->thresholds[SoftStart_Next] <= dutyCycle))
	{
		//Stop current DMA transfer
		HAL_DMA_Abort_IT(pwmStruct->hdma);

		//Find first value inside buffer
		for(start = 0; (start <= PWM_SS_MAX_CYCLES) && (pwmStruct->ssStruct->buffer[start] < dutyCycle); start++);

		//Find last index inside buffer
		for(finish = start; (finish < PWM_SS_MAX_CYCLES) && (pwmStruct->ssStruct->buffer[finish] < dutyCycle); finish++);

		//Start DMA transfer based on output type
		if(*pwmStruct->ssStruct->outHardware == Output_PWM)
			HAL_TIM_PWM_Start_DMA(pwmStruct->htim, pwmStruct->timChannel, (uint32_t*) &pwmStruct->ssStruct->buffer[start], (finish-start));

		else
			HAL_TIMEx_PWMN_Start_DMA(pwmStruct->htim, pwmStruct->timChannel, (uint32_t*) &pwmStruct->ssStruct->buffer[start], (finish-start));

		retVal = 1;
	}

	return retVal;
}

//Sets PWM output duty cycle using its command variables
//PDM_PWM_Map_Struct* mapStruct - Pointer to struct containing PWM map data
static uint16_t PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* mapStruct)
{
	uint16_t retVal = 0;

	//Checks if both command variables out of the column and line limits and attributes the map's closest corner value
	if((*mapStruct->inputs[0] <= mapStruct->axis[0][0])
		&& (*mapStruct->inputs[1] <= mapStruct->axis[1][0]))

		retVal = mapStruct->map[0][0];

	else if((*mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1])
			 && (*mapStruct->inputs[1] <= mapStruct->axis[1][0]))

		retVal = mapStruct->map[mapStruct->lengths[0] - 1][0];

	else if((*mapStruct->inputs[0] <= mapStruct->axis[0][0])
			 && (*mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1]))

		retVal = mapStruct->map[0][mapStruct->lengths[1] - 1];

	else if((*mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1])
			 && (*mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1]))

		retVal = mapStruct->map[mapStruct->lengths[0] - 1][mapStruct->lengths[1] - 1];

	//Check if the command variable point is outside the lines (y limits) of the 3D map or there is only 1 variable input
	else if((*mapStruct->inputs[1] <= mapStruct->axis[1][0])
			|| (*mapStruct->inputs[1] >= mapStruct->axis[1][mapStruct->lengths[1] - 1])
			|| (mapStruct->lengths[1] == 1))
	{
		for(uint8_t x = 0; x < (mapStruct->lengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((*mapStruct->inputs[0] >= mapStruct->axis[0][x])
				&& (*mapStruct->inputs[0] <= mapStruct->axis[0][x + 1]))
			{
				//Checks if the command variable point is above or below the lines (y limits) of the 3D map
				//hen sets duty cycle via linear interpolation
				if((*mapStruct->inputs[1] <= mapStruct->axis[1][0]) || (mapStruct->lengths[1] == 1))
				{
					retVal = __PDM_LINEAR_INTERPOLATION(*mapStruct->inputs[0],
														 mapStruct->axis[0][x],
														 mapStruct->axis[0][x + 1],
														 mapStruct->map[x][0],
														 mapStruct->map[x + 1][0]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(*mapStruct->inputs[0],
														 mapStruct->axis[0][x],
														 mapStruct->axis[0][x + 1],
														 mapStruct->map[x][mapStruct->lengths[1] - 1],
														 mapStruct->map[x + 1][mapStruct->lengths[1] - 1]);
				}
			}
		}
	}

	//Check if the command variable point is outside the columns (x limits) of the 3D map
	else if((*mapStruct->inputs[0] <= mapStruct->axis[0][0])
			|| (*mapStruct->inputs[0] >= mapStruct->axis[0][mapStruct->lengths[0] - 1]))
	{
		for(uint8_t y = 0; y < (mapStruct->lengths[1] - 1); y++)
		{
			//Checks if the command variable point is inside the y, y + 1 line
			if((*mapStruct->inputs[1] >= mapStruct->axis[1][y])
				&& (*mapStruct->inputs[1] <= mapStruct->axis[1][y + 1]))
			{
				//Checks if the command variable point is to the left or to the right of the 3D map
				//Then sets duty cycle via linear interpolation
				if(*mapStruct->inputs[0] <= mapStruct->axis[0][0])
				{
					retVal = __PDM_LINEAR_INTERPOLATION(*mapStruct->inputs[1],
														 mapStruct->axis[1][y],
														 mapStruct->axis[1][y + 1],
														 mapStruct->map[0][y],
														 mapStruct->map[0][y + 1]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(*mapStruct->inputs[1],
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
			if((*mapStruct->inputs[0] >= mapStruct->axis[0][x])
				&& (*mapStruct->inputs[0] <= mapStruct->axis[0][x + 1]))
			{
				for(uint8_t y = 0; y < (mapStruct->lengths[1] - 1); y++)
				{
					//Checks if the command variable point is inside the y, y + 1 line
					if((*mapStruct->inputs[1] >= mapStruct->axis[1][y])
						&& (*mapStruct->inputs[1] <= mapStruct->axis[1][y + 1]))
					{
						retVal = __PDM_BILINEAR_INTERPOLATION(*mapStruct->inputs[0],
															  *mapStruct->inputs[1],
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
