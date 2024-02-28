/*
 * pdm_channels.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Rodolfo
 */

#include "pdm.h"

static uint8_t Function_Custom_Process(PDM_Function_Struct* funcStruct);
static uint8_t Function_Result_Process(PDM_Function_Struct* funcStruct);
static uint8_t Input_Detect_Edge(PDM_Function_Struct* fncStruct, PDM_Function_InputChange inNbr);

/*BEGIN FUNCTIONS*/

//Cast read data to int32_t inside Channel struct
//PDM_Data_Channel_Struct* dataStruct - Pointer to Channel struct
//uint8_t* buffer - Pointer to buffer where data is stored
void PDM_Data_Cast(PDM_Channel_CAN_Struct* dataStruct, uint8_t* buffer)
{
	uint8_t aux = 0;

	//Cast data read from CAN bus to accommodate correct negative and positive values
	//Use data mask in case of unsigned data and byte swap in case of 16 bit data
	switch(dataStruct->cast)
	{
		case Cast_Uint8:
			dataStruct->data[Data_Current] = (int32_t) (buffer[0] & dataStruct->mask);
			break;

		case Cast_Int8:
			dataStruct->data[Data_Current] = (int32_t) ((int8_t) buffer[0]);
			break;

		case Cast_Uint16:

			if(dataStruct->alignment == Alignment_Swap)
			{
				aux = buffer[0];
				buffer[0] = buffer[1];
				buffer[1] = aux;
			}

			dataStruct->data[Data_Current] = (int32_t) ((uint16_t) ((buffer[0] << 8) | buffer[1]) & dataStruct->mask);

			break;

		case Cast_Int16:

			if(dataStruct->alignment == Alignment_Swap)
			{
				aux = buffer[0];
				buffer[0] = buffer[1];
				buffer[1] = aux;
			}

			dataStruct->data[Data_Current] = (int32_t) ((int16_t) ((buffer[0] << 8) | buffer[1]));

			break;
	}

	return;
}

/*END FUNCTIONS*/

/*BEGIN THREAD FUNCTIONS*/

//Handle data channels and conditional functions used for output level setting
//Receive CAN bus data via Message Queue from PDM_CAN_Thread_Receive_Data
//Receive Local data via Message Queue from PDM_Readings_Thread
//Process
void PDM_Process_Thread(void* threadStruct)
{
	PDM_Process_Thread_Struct* thrdStr = (PDM_Process_Thread_Struct*) threadStruct;

	uint8_t processFlag;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		if(osSemaphoreAcquire(thrdStr->semHandle, osWaitForever) == osOK)
		{
			//Process functions if any of their inputs is updated
			processFlag = Function_Custom_Process(thrdStr->functions);

			//Process outputs if any of their inputs is updated
			if(((processFlag & PROCESS_OUTPUT) == PROCESS_OUTPUT)
					|| ((processFlag & PROCESS_PWM) == PROCESS_PWM))
				osMessageQueuePut(thrdStr->outQueueHandle, &processFlag, 0, 0);
		}

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit();	//Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREAD FUNCTIONS*/

/*BEGIN STATIC FUNCTIONS*/

//Process each custom Function set by user
//PDM_Function_Struct* funcStruct - Pointer to array of Functions
static uint8_t Function_Custom_Process(PDM_Function_Struct* funcStruct)
{
	uint8_t retVal = PROCESS_NONE;

	for(uint16_t i = 0; i < FNC_CUSTOM_FINISH; i++)
	{
		//Set previous result
		funcStruct[i].result[Result_Previous] = funcStruct[i].result[Result_Current];

		//Continue "for loop" if Function is unused and there are no input changes
		if((funcStruct[i].inUse == IN_USE_NONE)
				|| (Input_Detect_Edge(&funcStruct[i], FuncIn_Current1)
						&& Input_Detect_Edge(&funcStruct[i], FuncIn_Current2)
						&& Input_Detect_Edge(&funcStruct[i], FuncIn_Current3)
						&& Input_Detect_Edge(&funcStruct[i], FuncIn_Current4)))
			continue;

		//Process each result based on Function Type
		switch(funcStruct[i].type)
		{
			case Function_NOT:
				funcStruct[i].result[Result_Next] = !(*funcStruct[i].inputs[FuncIn_Current1]);

				break;

			case Function_AND:
				//Set result to allow true results and set flag
				funcStruct[i].result[Result_Next] = Result_True;


				//AND operation with each Function input
				for(uint8_t k = 0; k < funcStruct[i].nbrOfInputs; k++)
					funcStruct[i].result[Result_Next] = (funcStruct[i].result[Result_Next] && *funcStruct[i].inputs[k*2]);

				break;

			case Function_OR:
				//Reset result to allow false results and set flag
				funcStruct[i].result[Result_Next] = Result_False;

				//OR operation with each Function input
				for(uint8_t k = 0; k < funcStruct[i].nbrOfInputs; k++)
					funcStruct[i].result[Result_Next] = (funcStruct[i].result[Result_Next] || *funcStruct[i].inputs[k*2]);

				break;

			case Function_XOR:
				funcStruct[i].result[Result_Next] = ((*funcStruct[i].inputs[FuncIn_Current1] || *funcStruct[i].inputs[FuncIn_Current2])
												&& !(*funcStruct[i].inputs[FuncIn_Current1] && *funcStruct[i].inputs[FuncIn_Current2]));

				break;

			case Function_BitAND:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] & *funcStruct[i].inputs[FuncIn_Current2]);

				break;

			case Function_Equals:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] == *funcStruct[i].inputs[FuncIn_Current2]);

				break;

			case Function_Less:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] < *funcStruct[i].inputs[FuncIn_Current2]);

				break;

			case Function_More:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] > *funcStruct[i].inputs[FuncIn_Current2]);

				break;

			case Function_Minus:
				funcStruct[i].result[Result_Next] = *funcStruct[i].inputs[FuncIn_Current1] - *funcStruct[i].inputs[FuncIn_Current2];

				break;

			case Function_Plus:
				funcStruct[i].result[Result_Next] = *funcStruct[i].inputs[FuncIn_Current1] + *funcStruct[i].inputs[FuncIn_Current2];

				break;

			case Function_Hysteresis:
				//Set flag if there is any result change
				//Check low value
				if(*funcStruct[i].inputs[FuncIn_Current1] < *funcStruct[i].inputs[FuncIn_Current2])
					funcStruct[i].result[Result_Next] = Result_False;

				//Check high value
				else if(*funcStruct[i].inputs[FuncIn_Current1] > *funcStruct[i].inputs[FuncIn_Current3])
					funcStruct[i].result[Result_Next] = Result_True;

				break;

			case Function_Blink:
				//Set flag and set next equal to input result, Function output will blink while next result is true
				funcStruct[i].result[Result_Next] = *funcStruct[i].inputs[FuncIn_Current1];

				break;

			case Function_Pulse:
				//Set flag if a pulse should start or stop
				//Check for pulse stop condition
				if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current2))
					funcStruct[i].result[Result_Next] = Result_False;

				//Check for change in input state and and correct input edge
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current1))
					funcStruct[i].result[Result_Next] = Result_True;

				break;

			case Function_SetReset:
				//Set flag if there is any result change
				//Reset result if input state 1 has changed and has correct edge
				if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current1))
					funcStruct[i].result[Result_Next] = Result_False;

				//Set result if input state 2 has changed and has correct edge
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current2))
					funcStruct[i].result[Result_Next] = Result_True;

				break;

			case Function_Toggle:
				//Set flag if there is any result change
				//Override for false output
				if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current2))
					funcStruct[i].result[Result_Next] = Result_False;

				//Override for true output
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current3))
					funcStruct[i].result[Result_Next] = Result_True;

				//Perform not operation on next result if input state changed and has correct edge
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current1))
					funcStruct[i].result[Result_Next] = !funcStruct[i].result[Result_Next];

				break;

			case Function_Counter:
				//Update retVal if there is any change in value
				//First override
				if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current3))
					funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Ovrr1];

				//Second override
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current4))
					funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Ovrr2];

				//Decrement if input state 1 has changed and has correct edge
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current1))
				{
					//Check if channel isn't already at minimum count
					funcStruct[i].result[Result_Next] -= funcStruct[i].consts[FuncConst_Dec];

					//Check if count is below minimum count
					if(funcStruct[i].result[Result_Next] < funcStruct[i].consts[FuncConst_Low])
					{
						//Counter wrap to highest count
						if((funcStruct[i].countWrap & COUNTER_WRAP_TO_HIGH) == COUNTER_WRAP_TO_HIGH)
							funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Ovrr1];

						//Set to maximum count if wrap is disabled
						else
							funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Low];
					}
				}

				//Increment if input state 2 has changed and has correct edge
				else if(Input_Detect_Edge(&funcStruct[i], FuncIn_Current2))
				{
					funcStruct[i].result[Result_Next] += funcStruct[i].consts[FuncConst_Inc];

					//Check if count is above maximum count
					if(funcStruct[i].result[Result_Next] > funcStruct[i].consts[FuncConst_High])
					{
						//Counter wrap to lowest count if enabled
						if((funcStruct[i].countWrap & COUNTER_WRAP_TO_LOW) == COUNTER_WRAP_TO_LOW)
							funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Ovrr2];

						//Set to maximum count if wrap is disabled
						else
							funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_High];
					}
				}

				break;

			default:
				continue;
		}

		//Process result inversions and delays
		retVal |= Function_Result_Process(&funcStruct[i]);
	}

	return retVal;
}

//Set current result or start delay timer according to next result state
//PDM_Function_Struct* funcStruct = Pointer to Function struct
static uint8_t Function_Result_Process(PDM_Function_Struct* funcStruct)
{
	//Store return flags
	uint8_t retVal = PROCESS_NONE;

	//Store next Result to process according to correct Delay
	uint8_t aux;

	//Process according to Function type
	//Timed Functions receive their specific process
	if((funcStruct->type == Function_Blink) || (funcStruct->type == Function_Pulse))
	{
		//Invert current result if enabled
		if(funcStruct->invert == 1)
			funcStruct->result[Result_Current] = !funcStruct->result[Result_Next];

		else
			funcStruct->result[Result_Current] = funcStruct->result[Result_Next];

		//Set aux variable to Process correct result Timer condition
		 aux = funcStruct->result[Result_Current];

		//Start or stop timer according to current result
		if(funcStruct->result[Result_Next] == Result_True)
			osTimerStart(funcStruct->funcTimerHandle, funcStruct->funcDelay[aux]);

		else if(osTimerIsRunning(funcStruct->funcTimerHandle) == 1)
			osTimerStop(funcStruct->funcTimerHandle);

		retVal = funcStruct->inUse;
	}

	else
	{
		//Invert result if configured
		if(funcStruct->invert == 1)
			funcStruct->result[Result_Next] = !funcStruct->result[Result_Next];

		//Set aux variable to Process correct result Timer condition
		aux = funcStruct->result[Result_Next];

		//Set current Result and retVal if there is no Delay
		if(funcStruct->funcDelay[aux] == 0)
		{
			funcStruct->result[Result_Current] = funcStruct->result[Result_Next];

			retVal = funcStruct->inUse;
		}

		//Start Timer to wait Delay
		else
			osTimerStart(funcStruct->funcTimerHandle, funcStruct->funcDelay[aux]);
	}

	return retVal;
}

//Detect if there is change in level of function inputs
//PDM_Function_Struct* funcStruct - Pointer to Function struct
//PDM_Function_InputChange inNbr - Number of corresponding input
static uint8_t Input_Detect_Edge(PDM_Function_Struct* fncStruct, PDM_Function_InputChange inNbr)
{
	//Return 0 if inputs are equal or edges don't match
	return ((*fncStruct->inputs[inNbr] == *fncStruct->inputs[inNbr+NBR_OF_FUNC_INPUTS])
			|| ((*fncStruct->inputs[inNbr] < *fncStruct->inputs[inNbr+NBR_OF_FUNC_INPUTS]) && (fncStruct->inEdge[inNbr] == Edge_Rising))
			|| ((*fncStruct->inputs[inNbr] > *fncStruct->inputs[inNbr+NBR_OF_FUNC_INPUTS]) && (fncStruct->inEdge[inNbr] == Edge_Falling)));
}

/*END STATIC FUNCTIONS*/
