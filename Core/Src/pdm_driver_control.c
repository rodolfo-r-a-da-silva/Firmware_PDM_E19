/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

static void Output_Set_State(PDM_Output_Ctrl_Struct* outStruct, uint16_t* dataBuffer);
static void PWM_Set_DutyCycle(PDM_PWM_Ctrl_Struct* pwmStruct, GPIO_PinState outLevel);
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
		if((*outStruct[i]->inputFunc == Result_True)
				&& (outStruct[i]->fuseStruct->status != Fuse_Open))
			outStruct[i]->outputState = GPIO_PIN_SET;

		else
			outStruct[i]->outputState = GPIO_PIN_RESET;

		//Set output level or PWM duty cycle
		if(outStruct->outputHardware == Output_GPIO)
		{
			__PDM_OUT_SET_LEVEL(outStruct[i]);

			//Place output state inside data buffer
			dataBuffer[Data_Output] |= (outStruct[i]->outputState << i);
		}

		else if((outStruct->outputHardware == Output_PWM) || (outStruct->outputHardware == Output_PWMN))
		{
			PWM_Set_DutyCycle(outStruct[i]->pwmStruct, outStruct[i]->outputState);

			__PDM_PWM_SET_COMPARE(outStruct[i]);

			//Place PWM Duty Cycle inside data buffer
			dataBuffer[j++] = outStruct[i]->pwmStruct->dutyCycle;

			//Place output state inside data buffer
			if(outStruct[i]->pwmStruct->dutyCycle != PWM_MIN_DUTY_CYCLE)
				dataBuffer[Data_Output] |= (GPIO_PIN_SET << i);
		}
	}

	return;
}

//Process input conditions and command variables and sets the PWM output duty cycle
//pwmStruct - Pointer to struct containing PWM configuration and data
//outLevel - Output pin level
static void PWM_Set_DutyCycle(PDM_PWM_Ctrl_Struct* pwmStruct, GPIO_PinState outLevel)
{
	//Auxiliary variable to set duty cycle
	uint16_t dutyCycle = PWM_MIN_DUTY_CYCLE;

	//Check if output level is low and return if true
	if(outLevel == GPIO_PIN_RESET)
		return;

	//Set PWM to maximum Duty Cycle if configured as standard output
	if(pwmStruct->type == OutType_Standard)
		dutyCycle = PWM_MAX_DUTY_CYCLE;

	else if(pwmStruct->type >= OutType_Pwm_Preset)
	{
		//Process preset conditions if enabled
		for(uint8_t i = 0; i < PWM_NBR_OF_PRESETS; i++)
		{
			if((pwmStruct->presetInputs[i] != NULL) && (*pwmStruct->presetInputs[i] == Result_True))
			{
				dutyCycle = pwmStruct->presetDutyCycle[i];
				break;
			}
		}

		if(dutyCycle != PWM_MIN_DUTY_CYCLE)
		{
			//Calculate Duty Cycle based on 3D map
			if(pwmStruct->type == OutType_Pwm_Map)
				dutyCycle = PWM_Set_Map_DutyCycle(pwmStruct->mapStruct);

			else if(pwmStruct->type == OutType_Pwm_Ann)
				1;
		}
	}

	if(pwmStruct->softStart == SoftStart_Disabled)
		pwmStruct->dutyCycle = dutyCycle;

	return;
}

//Sets PWM output duty cycle using its command variables
//mapStruct - Pointer to struct containing PWM map data
static uint16_t PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* mapStruct)
{
	uint16_t retVal = 0;

	//Checks if both command variables out of the column and line limits and attributes the map's closest corner value
	if((mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][0])
		&& (mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][0]))

		retVal = mapStruct->dutyCycleMap[0][0];

	else if((mapStruct->commandVar[0] >= mapStruct->commandVarStep[0][mapStruct->mapLengths[0] - 1])
			 && (mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][0]))

		retVal = mapStruct->dutyCycleMap[mapStruct->mapLengths[0] - 1][0];

	else if((mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][0])
			 && (mapStruct->commandVar[1] >= mapStruct->commandVarStep[1][mapStruct->mapLengths[1] - 1]))

		retVal = mapStruct->dutyCycleMap[0][mapStruct->mapLengths[1] - 1];

	else if((mapStruct->commandVar[0] >= mapStruct->commandVarStep[0][mapStruct->mapLengths[0] - 1])
			 && (mapStruct->commandVar[1] >= mapStruct->commandVarStep[1][mapStruct->mapLengths[1] - 1]))

		retVal = mapStruct->dutyCycleMap[mapStruct->mapLengths[0] - 1][mapStruct->mapLengths[1] - 1];

	//Check if the command variable point is outside the lines (y limits) of the 3D map or there is only 1 variable input
	else if((mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][0])
			|| (mapStruct->commandVar[1] >= mapStruct->commandVarStep[1][mapStruct->mapLengths[1] - 1])
			|| (mapStruct->mapLengths[1] == 1))
	{
		for(uint8_t x = 0; x < (mapStruct->mapLengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((mapStruct->commandVar[0] >= mapStruct->commandVarStep[0][x])
				&& (mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][x + 1]))
			{
				//Checks if the command variable point is above or below the lines (y limits) of the 3D map
				//hen sets duty cycle via linear interpolation
				if((mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][0]) || (mapStruct->mapLengths[1] == 1))
				{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->commandVar[0],
																  	   mapStruct->commandVarStep[0][x],
																	   mapStruct->commandVarStep[0][x + 1],
																	   mapStruct->dutyCycleMap[x][0],
																	   mapStruct->dutyCycleMap[x + 1][0]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->commandVar[0],
																  	   mapStruct->commandVarStep[0][x],
																	   mapStruct->commandVarStep[0][x + 1],
																	   mapStruct->dutyCycleMap[x][mapStruct->mapLengths[1] - 1],
																	   mapStruct->dutyCycleMap[x + 1][mapStruct->mapLengths[1] - 1]);
				}
			}
		}
	}

	//Check if the command variable point is outside the columns (x limits) of the 3D map
	else if((mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][0])
			|| (mapStruct->commandVar[0] >= mapStruct->commandVarStep[0][mapStruct->mapLengths[0] - 1]))
	{
		for(uint8_t y = 0; y < (mapStruct->mapLengths[1] - 1); y++)
		{
			//Checks if the command variable point is inside the y, y + 1 line
			if((mapStruct->commandVar[1] >= mapStruct->commandVarStep[1][y])
				&& (mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][y + 1]))
			{
				//Checks if the command variable point is to the left or to the right of the 3D map
				//Then sets duty cycle via linear interpolation
				if(mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][0])
				{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->commandVar[1],
																  	   mapStruct->commandVarStep[1][y],
																	   mapStruct->commandVarStep[1][y + 1],
																	   mapStruct->dutyCycleMap[0][y],
																	   mapStruct->dutyCycleMap[0][y + 1]);
				}else{
					retVal = __PDM_LINEAR_INTERPOLATION(mapStruct->commandVar[1],
																  	   mapStruct->commandVarStep[1][y],
																	   mapStruct->commandVarStep[1][y + 1],
																	   mapStruct->dutyCycleMap[mapStruct->mapLengths[0] - 1][y],
																	   mapStruct->dutyCycleMap[mapStruct->mapLengths[0] - 1][y + 1]);
				}
			}
		}
	}

	//Since the command variable point is inside the map's boundary, sets duty cycle via bilinear interpolation
	else
	{
		for(uint8_t x = 0; x < (mapStruct->mapLengths[0] - 1); x++)
		{
			//Checks if the command variable point is inside the x, x + 1 column
			if((mapStruct->commandVar[0] >= mapStruct->commandVarStep[0][x])
				&& (mapStruct->commandVar[0] <= mapStruct->commandVarStep[0][x + 1]))
			{
				for(uint8_t y = 0; y < (mapStruct->mapLengths[1] - 1); y++)
				{
					//Checks if the command variable point is inside the y, y + 1 line
					if((mapStruct->commandVar[1] >= mapStruct->commandVarStep[1][y])
						&& (mapStruct->commandVar[1] <= mapStruct->commandVarStep[1][y + 1]))
					{
						retVal = __PDM_BILINEAR_INTERPOLATION(mapStruct->commandVar[0],
																		  	 mapStruct->commandVar[1],
																		     mapStruct->commandVarStep[0][x],
																		     mapStruct->commandVarStep[0][x + 1],
																		     mapStruct->commandVarStep[1][y],
																		     mapStruct->commandVarStep[1][y + 1],
																		     mapStruct->dutyCycleMap[x][y],
																		     mapStruct->dutyCycleMap[x + 1][y],
																		     mapStruct->dutyCycleMap[x][y + 1],
																		     mapStruct->dutyCycleMap[x + 1][y + 1]);
					}
				}
			}
		}
	}

	return retVal;
}

/*END STATIC FUNCTIONS*/
