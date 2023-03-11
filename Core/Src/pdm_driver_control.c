/*
 * pdm_driver_controls.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include <pdm.h>

static void Output_Set_State(PDM_Output_Ctrl_Struct* outputStruct, PDM_CAN_Rx_Struct* canRxStruct, uint32_t inputLevels);
static void Output_CAN_Set_Level(PDM_CAN_Rx_Struct* canRxStruct, PDM_Output_CAN_Struct* canOutStruct);
static void Output_PWM_Set_DutyCycle(PDM_Output_Ctrl_Struct* outputStruct);
static uint16_t Output_PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* pwm_map_struct);

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
	PDM_CAN_Rx_Struct canRxStruct;		//Store data received via CAN bus

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		inputLevels = 0;	//Resets variable that stores input pin levels

		//Read all input pin levels
		for(uint8_t i = 0; i < NBR_OF_INPUTS; i++)
			inputLevels |= HAL_GPIO_ReadPin(thrdStr->inputGPIOs[i], thrdStr->inputPins[i]) << i;

		//Set all output levels and PWM duty cycles
		for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
			Output_Set_State(thrdStr->outStruct[i], &canRxStruct);

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

		//Wait for any interaction that require output level changing
		osSemaphoreAcquire(thrdStr->semaphoreHandle, osWaitForever);

		//Check if any data was received via CAN bus
		if(osMessageQueueGet(thrdStr->canQueueHandle, (void*) canRxStruct, NULL, 0) != osOK)
			canRxStruct->canFlag = CAN_Idle;
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

static void Output_Set_State(PDM_Output_Ctrl_Struct* outputStruct, PDM_CAN_Rx_Struct* canRxStruct, uint32_t inputLevels)
{
	//Set input pin output flags
	if(__PDM_INPUT_CONDITION_COMPARE(outputStruct->inputEnable[0], outputStruct->inputLevels[0], outputStruct->outEnable[0])
			|| __PDM_INPUT_CONDITION_COMPARE(outputStruct->inputEnable[1], outputStruct->inputLevels[1], outputStruct->outEnable[1]))
		outputStruct->outputState[OutState_Inputs] = GPIO_PIN_SET;
	else
		outputStruct->outputState[OutState_Inputs] = GPIO_PIN_RESET;

	//Set delay output flags
	switch(outputStruct->delayStruct.delayType)
	{
		case Delay_Standard:
			break;

		case Delay_Blink:
			break;

		case Delay_Pulse:
			break;

		default:
			break;
	}

	//Set output level or PWM duty cycle
	if(outputStruct->outputHardware == Output_GPIO)
		HAL_GPIO_WritePin(outputStruct->outputState, outputStruct->outputPin, outputStruct->outputState[OutState_Current]);

	else if((outputStruct->outputHardware == Output_PWM) || (outputStruct->outputHardware == Output_PWMN))
		__HAL_TIM_SET_COMPARE(outputStruct->pwmStruct->htim, outputStruct->pwmStruct->timChannel, outputStruct->pwmStruct->dutyCycle);

	return;
}

//Process input conditions and command variables and sets the PWM output duty cycle
static void Output_PWM_Set_DutyCycle(PDM_Output_Ctrl_Struct* outputStruct)
{


//	if((outputStruct->pwmStruct != NULL) && (outputStruct->pwmStruct->htim != NULL))
//	{
//		if(outputStruct->outputState[0] == GPIO_PIN_SET)
//		{
//			//Checks if the inputs match the first PWM preset
//			if(__PDM_INPUT_CONDITION_COMPARE(outputStruct->pwmStruct->presetEnable[0], outputStruct->pwmStruct->presetInputs[0], outputStruct->pwmStruct->outEnable[0]))
//				outputStruct->pwmStruct->dutyCycle = outputStruct->pwmStruct->presetDutyCycle[0];
//
//			//Checks if the inputs match the second PWM preset
//			else if(__PDM_INPUT_CONDITION_COMPARE(outputStruct->pwmStruct->presetEnable[1], outputStruct->pwmStruct->presetInputs[1], outputStruct->pwmStruct->outEnable[1]))
//				outputStruct->pwmStruct->dutyCycle = outputStruct->pwmStruct->presetDutyCycle[1];
//
//			//Sets duty cycle based on the 3D map if enabled
//			else if((outputStruct->pwmStruct->outputType == OutType_Pwm_Map) && (outputStruct->pwmStruct->pwmMapStruct != NULL))
//				outputStruct->pwmStruct->dutyCycle = Output_PWM_Set_Map_DutyCycle(outputStruct->pwmStruct->pwmMapStruct);
//
//			else
//				outputStruct->pwmStruct->dutyCycle = 0;
//		}
//
//		else
//			outputStruct->pwmStruct->dutyCycle = 0;
//
//		__HAL_TIM_SET_COMPARE(outputStruct->pwmStruct->htim, outputStruct->pwmStruct->timChannel, outputStruct->pwmStruct->dutyCycle);
//	}

	return;
}

//Sets PWM output duty cycle using its command variables
static uint16_t Output_PWM_Set_Map_DutyCycle(PDM_PWM_Map_Struct* pwm_map_struct)
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

//Set output level, Delay timer, Blink timer or Blink timer
static void Output_Delay_Callback(PDM_Output_Ctrl_Struct* outputStruct)
{
	return;
}
