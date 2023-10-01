/*
 * pdm_channels.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Rodolfo
 */

#include "pdm.h"

static uint8_t Channel_CAN_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Queue_Struct* rxStruct, uint8_t nbrOfChannels);
static uint8_t Channel_Local_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Type dataRead, int16_t* dataBuffer);
static uint8_t Function_Input_Process(PDM_Function_Struct* dataStruct, PDM_Input_Struct* inStruct, int16_t* dataBuffer);
static uint8_t Function_Fuse_Process(PDM_Function_Struct* funcStruct, PDM_Output_Ctrl_Struct* outStruct, int16_t* dataBuffer);
static uint8_t Function_Custom_Process(PDM_Function_Struct* funcStruct, uint8_t nbrOfFunctions);
static uint8_t Function_Result_Process(PDM_Function_Struct* funcStruct);
static void Read_Analog_Inputs(TIM_HandleTypeDef* htim, PDM_Data_Type* readingFlag, uint16_t* adcBuffer, int16_t* dataBuffer);
static void Fuse_Check_Status(PDM_Output_Fuse_Struct* fuseStruct, PDM_Data_Type dataRead, int16_t* dataBuffer);

/*BEGIN FUNCTIONS*/

//Cast read data to int32_t inside Channel struct
//PDM_Data_Channel_Struct* dataStruct - Pointer to Channel struct
//uint8_t* buffer - Pointer to buffer where data is stored
void PDM_Data_Cast(PDM_Data_Channel_Struct* dataStruct, uint8_t* buffer)
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

	uint8_t processFlag = 0;
	PDM_Data_Queue_Struct rxStruct;

	int16_t dataBuffer[NBR_OF_DATA_CHANNELS];	//Thread safe buffer to store local data
	PDM_Data_Type readingFlag[2];				//Flag for data to be read from HSDs, starts by setting reading to current

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Set for first readings
	readingFlag[0] = Data_Type_Init;
	readingFlag[1] = Data_Type_Init;

	for(;;)
	{
		if(osMessageQueueGet(thrdStr->intQueueHandle, (void*) &rxStruct, 0, osWaitForever) == osOK)
		{
			switch(rxStruct.source)
			{
				case Interrupt_CAN:
					//Try to convert data for each CAN data channel
					processFlag = Channel_CAN_Process(thrdStr->channels, &rxStruct, thrdStr->nbrOfChannels);

					break;

				case Interrupt_Function:
					break;

				case Interrupt_Fuse:
					//Process fuse Functions and place states in Thread safe data buffer
					processFlag = Function_Fuse_Process(thrdStr->functions, thrdStr->outStruct, dataBuffer);

					break;

				case Interrupt_Gpio:
					//Read all input pins, if any pin is used, set flag
					processFlag = Function_Input_Process(thrdStr->functions, thrdStr->inStruct, dataBuffer);

					break;

				case Interrupt_Timer:
					//Read analog inputs and place values in thread safe buffer
					Read_Analog_Inputs(thrdStr->htim, readingFlag, thrdStr->adcBuffer, dataBuffer);

					//Check currents and fuse status for each output
					Fuse_Check_Status(thrdStr->fuseStruct, readingFlag[1], dataBuffer);

					//Check if any channel uses local data and if yes, change channel value and update flag
					processFlag = Channel_Local_Process(thrdStr->channels, readingFlag[1], dataBuffer);

					if(osMutexAcquire(thrdStr->dataMutexHandle, 0) == osOK)	//Check if data buffer is safe to access
					{
						osMutexRelease(thrdStr->dataMutexHandle);	//Release data buffer Mutex
					}

					break;
			}
		}

		if((processFlag & PROCESS_FUNCTION) == PROCESS_FUNCTION)
			processFlag |= Function_Custom_Process(thrdStr->functions, thrdStr->nbrOfFunctions);

		if(((processFlag & PROCESS_OUTPUT) == PROCESS_OUTPUT)
				|| ((processFlag & PROCESS_PWM) == PROCESS_PWM))
			osMessageQueuePut(thrdStr->outQueueHandle, &processFlag, 0, 0);

		processFlag = 0;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit();	//Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREAD FUNCTIONS*/

/*BEGIN STATIC FUNCTIONS*/

//Try to convert data received from CAN bus
//dataStruct - Pointer to array of data Channel structs
//rxStruct - Pointer to struct containing data from CAN bus
//nbrOfChannels - Total number of data Channels
static uint8_t Channel_CAN_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Queue_Struct* rxStruct, uint8_t nbrOfChannels)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t aux[2];


	for(uint8_t i = CHANNEL_CAN_OFFSET; i < (CHANNEL_CAN_OFFSET+nbrOfChannels); i++)
	{
		//Continue "for loop" if CAN bus frame id doesn't match channel filter or channel isn't used for any conditions
		if((dataStruct[i].dataFilter != rxStruct->id)
				|| (dataStruct[i].inUse == IN_USE_NONE))
			continue;

		//Get data from Queue according to data source
		//Return if channel source isn't from CAN bus
		switch(dataStruct[i].source)
		{
			//For fixed position data in specific frame
			//Will continue "for loop" if cast type is invalid
			case Data_CAN_Fixed:
			//{
				if((dataStruct[i].cast == Cast_Uint8) || (dataStruct[i].cast == Cast_Int8))
					aux[0] = rxStruct->data[dataStruct[i].position];

				else if((dataStruct[i].cast == Cast_Uint16) || (dataStruct[i].cast == Cast_Int16))
				{
					aux[0] = rxStruct->data[dataStruct[i].position];
					aux[1] = rxStruct->data[dataStruct[i].position + 1];
				}

				else
					continue;

				break;
			//}

			//For data identification inside the frame's data field
			//Will continue "for loop" if frame length is invalid or data isn't available
			case Data_CAN_Channel:
			//{
				if((rxStruct->length != 4) && (rxStruct->length != 8))
				{
					if(dataStruct[i].position == ((rxStruct->data[1] << 8) | rxStruct->data[0]))
					{
						aux[0] = rxStruct->data[2];
						aux[1] = rxStruct->data[3];
					}

					else if((dataStruct[i].position == ((rxStruct->data[4] << 8) | rxStruct->data[5])) && (rxStruct->length == 8))
					{
						aux[0] = rxStruct->data[6];
						aux[1] = rxStruct->data[7];
					}

					else
						continue;
				}

				else
					continue;

				break;
			//}

			default:
				continue;
		}

		//Reset timer if data was received and timer was created correctly
		//Set return flag to indicate received value
		if(osTimerStart(dataStruct[i].timer, pdMS_TO_TICKS(dataStruct[i].timeout)) == osOK)
			dataStruct[i].timeoutFlag = CAN_Data_Refresh;

		//Reset value and set flag to timeout if timer cannot be started
		else
		{
			PDM_Data_Timeout_Callback((void*) dataStruct);
			continue;
		}

		//Change previous state of data
		dataStruct[i].data[Data_Previous] = dataStruct[i].data[Data_Current];

		//Cast received data to correct format
		PDM_Data_Cast(dataStruct, aux);

		//Set retVal if value has changed and CAN data is used in Functions or PWM
		if(dataStruct[i].data[Data_Previous] != dataStruct[i].data[Data_Current])
			retVal |= dataStruct[i].inUse;
	}

	return retVal;
}

//Convert data read from analog MCU inputs
//dataStruct - Pointer to array of data Channel structs
//dataRead - Indication of last type of data read
//int16_t* dataBuffer - Pointer to Thread safe array containing all data read locally by the module
static uint8_t Channel_Local_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Type dataRead, int16_t* dataBuffer)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t i, j, k;	//dataStruct and dataBuffer indexes and maximum index for dataBuffer respectively

	//Set indexes based on last data read
	switch(dataRead)
	{
		case Data_Type_Current0:
			j = Data_Curr1;
			k = Data_Curr15;
			break;

		case Data_Type_Current1:
			j = Data_Curr2;
			k = Data_Curr16;
			break;

		case Data_Type_Temperature:
			j = Data_Temp1;
			k = Data_Temp8;
			break;

		case Data_Type_Voltage:
			j = Data_Volt;
			k = Data_Volt;
			break;

		default:
			return retVal;
	}

	//Update MCU temperature Channel
	dataStruct[CHANNEL_TEMP_MCU].data[Data_Previous] = dataStruct[CHANNEL_TEMP_MCU].data[Data_Current];
	dataStruct[CHANNEL_TEMP_MCU].data[Data_Current] = (int32_t) dataBuffer[Data_TempMCU];

	//Update retVal if data has changed
	if(dataStruct[CHANNEL_TEMP_MCU].data[Data_Previous] != dataStruct[CHANNEL_TEMP_MCU].data[Data_Current])
		retVal |= dataStruct[CHANNEL_TEMP_MCU].inUse;

	//Place each read data into respective Channel
	for(i = 0; j <= k; i++, j++)
	{
		dataStruct[j].data[Data_Previous] = dataStruct[j].data[Data_Current];
		dataStruct[j].data[Data_Current] = (int32_t) dataBuffer[i];

		//Update retVal if data has changed
		if(dataStruct[j].data[Data_Previous] != dataStruct[j].data[Data_Current])
			retVal |= dataStruct[j].inUse;
	}

	return retVal;
}

//Read Input Pin levels, process each Function and place Levels at the Thread safe data buffer
//PDM_Function_Struct* funcStruct - Pointer to Functions array
//PDM_Input_Struct* inStruct - Pointer to struct containing all Input pins information
//int16_t* dataBuffer - Pointer to Thread safe array containing all data read locally by the module
static uint8_t Function_Input_Process(PDM_Function_Struct* funcStruct, PDM_Input_Struct* inStruct, int16_t* dataBuffer)
{
	//Store value to return
	uint8_t retVal = PROCESS_NONE;

	//Reset input value in Thread safe buffer
	dataBuffer[Data_Input] = 0;

	for(uint8_t i = FUNCTION_INPUT_OFFSET, j = 0; i < FUNCTION_INPUT_FINISH; i++, j++)
	{
		//Read input pin level and set data value
		funcStruct[i].result[Result_Next] = HAL_GPIO_ReadPin(&inStruct->gpio[j], inStruct->pin[j]);

		//Change value in Thread safe buffer
		dataBuffer[Data_Input] |= (funcStruct[i].result[Result_Next] << j);

		//Skip if Input Pin is unused or next and current Results are equal
		if((funcStruct[i].inUse == IN_USE_NONE)
				|| (funcStruct[i].result[Result_Next] == funcStruct[i].result[Result_Current]))
			continue;

		retVal |= Function_Result_Process(&funcStruct[i]);
	}

	return retVal;
}

//Read output fuses state, update each function and place states into Thread safe data buffer
//Closed circuit is represented by "0" and open circuit by "1"
//PDM_Function_Struct* funcStruct - Pointer to Functions array
//PDM_Output_Ctrl_Struct* outStruct - Pointer to output structs array containing each fuse struct
//int16_t* dataBuffer - Pointer to Thread safe array containing all data read locally by the module
static uint8_t Function_Fuse_Process(PDM_Function_Struct* funcStruct, PDM_Output_Ctrl_Struct* outStruct, int16_t* dataBuffer)
{
	//Store value to return
	uint8_t retVal = PROCESS_NONE;

	//Reset input value in Thread safe buffer
	dataBuffer[Data_Fuse] = 0;

	for(uint8_t i = FUNCTION_FUSE_OFFSET, j = 0; i < FUNCTION_FUSE_FINISH; i++ ,j++)
	{
		//Place state inside result
		if(*outStruct[j].fuseSatus == Fuse_Closed)
			funcStruct[i].result[Result_Next] = 0;

		else
			funcStruct[i].result[Result_Next] = 1;

		//Change value in Thread safe buffer
		dataBuffer[Data_Fuse] |= (funcStruct[i].result[Result_Next] << j);

		//Skip if Input Pin is unused or next and current Results are equal
		if((funcStruct[i].inUse == IN_USE_NONE)
				|| (funcStruct[i].result[Result_Next] == funcStruct[i].result[Result_Current]))
			continue;

		retVal |= Function_Result_Process(&funcStruct[i]);
	}

	return retVal;
}

//Process each custom Function set by user
//PDM_Function_Struct* funcStruct - Pointer to array of Functions
//uint8_t nbrOfFunctions - Amount of custom functions defined by user
static uint8_t Function_Custom_Process(PDM_Function_Struct* funcStruct, uint8_t nbrOfFunctions)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t updateFlag = 0;

	for(uint8_t i = FUNCTION_CUSTOM_OFFSET, j = 0; j < nbrOfFunctions; i++, j++)
	{
		//Continue "for loop" if Function is unused
		if((funcStruct[i].inUse == IN_USE_NONE)
				|| ((funcStruct[i].result[FuncIn_Current1] == funcStruct[i].result[FuncIn_Previous1])
						&& (funcStruct[i].result[FuncIn_Current2] == funcStruct[i].result[FuncIn_Previous2])
						&& (funcStruct[i].result[FuncIn_Current3] == funcStruct[i].result[FuncIn_Previous3])
						&& (funcStruct[i].result[FuncIn_Current4] == funcStruct[i].result[FuncIn_Previous4])))
			continue;

		//Reset update flag, set only if there are state changes
		updateFlag = 0;

		//Process each result based on Function Type
		switch(funcStruct[i].type)
		{
			case Function_NOT:
				funcStruct[i].result[Result_Next] = !(*funcStruct[i].inputs[FuncIn_Current1]);
				updateFlag = 1;
				break;

			case Function_AND:
				//Set result to allow true results and set flag
				funcStruct[i].result[Result_Next] = Result_True;
				updateFlag = 1;

				//AND operation with each Function input
				for(uint8_t k = 0; k < funcStruct[i].nbrOfInputs; k++)
					funcStruct[i].result[Result_Next] = (funcStruct[i].result[Result_Next] && *funcStruct[i].inputs[k*2]);

				break;

			case Function_OR:
				//Reset result to allow false results and set flag
				funcStruct[i].result[Result_Next] = Result_False;
				updateFlag = 1;

				//OR operation with each Function input
				for(uint8_t k = 0; k < funcStruct[i].nbrOfInputs; k++)
					funcStruct[i].result[Result_Next] = (funcStruct[i].result[Result_Next] || *funcStruct[i].inputs[k*2]);

				break;

			case Function_XOR:
				funcStruct[i].result[Result_Next] = ((*funcStruct[i].inputs[FuncIn_Current1] || *funcStruct[i].inputs[FuncIn_Current2])
												&& !(*funcStruct[i].inputs[FuncIn_Current1] && *funcStruct[i].inputs[FuncIn_Current2]));
				updateFlag = 1;
				break;

			case Function_BitAND:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] & *funcStruct[i].inputs[FuncIn_Current2]);
				updateFlag = 1;
				break;

			case Function_Equals:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] == *funcStruct[i].inputs[FuncIn_Current2]);
				updateFlag = 1;
				break;

			case Function_Less:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] < *funcStruct[i].inputs[FuncIn_Current2]);
				updateFlag = 1;
				break;

			case Function_More:
				funcStruct[i].result[Result_Next] = (*funcStruct[i].inputs[FuncIn_Current1] > *funcStruct[i].inputs[FuncIn_Current2]);
				updateFlag = 1;
				break;

			case Function_Hysteresis:
				//Set flag if there is any result change
				//Check low value
				if(*funcStruct[i].inputs[FuncIn_Current1] < *funcStruct[i].inputs[FuncIn_Current2])
				{
					funcStruct[i].result[Result_Next] = Result_False;
					updateFlag = 1;
				}

				//Check high value
				else if(*funcStruct[i].inputs[FuncIn_Current1] > *funcStruct[i].inputs[FuncIn_Current3])
				{
					funcStruct[i].result[Result_Next] = Result_True;
					updateFlag = 1;
				}

				break;

			case Function_Blink:
				//Set flag and set next equal to input result, Function output will blink while next result is true
				funcStruct[i].result[Result_Next] = *funcStruct[i].inputs[FuncIn_Current1];
				updateFlag = 1;

				break;

			case Function_Pulse:
				//Set flag if a pulse should start or stop
				//Check for pulse stop condition
				if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current2))
				{
					funcStruct[i].result[Result_Next] = Result_False;
					updateFlag = 1;
				}

				//Check for change in input state and and correct input edge
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current1))
				{
					funcStruct[i].result[Result_Next] = Result_True;
					updateFlag = 1;
				}

				break;

			case Function_SetReset:
				//Set flag if there is any result change
				//Reset result if input state 1 has changed and has correct edge
				if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current1))
				{
					funcStruct[i].result[Result_Next] = Result_False;
					updateFlag = 1;
				}

				//Set result if input state 2 has changed and has correct edge
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current2))
				{
					funcStruct[i].result[Result_Next] = Result_True;
					updateFlag = 1;
				}

				break;

			case Function_Toggle:
				//Set flag if there is any result change
				//Override for false output
				if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current3))
				{
					funcStruct[i].result[Result_Next] = Result_False;
					updateFlag = 1;
				}

				//Override for true output
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current2))
				{
					funcStruct[i].result[Result_Next] = Result_True;
					updateFlag = 1;
				}

				//Perform not operation on next result if input state changed and has correct edge
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current1))
				{
					funcStruct[i].result[Result_Next] = !funcStruct[i].result[Result_Next];
					updateFlag = 1;
				}

				break;

			case Function_Counter:
				//Update retVal if there is any change in value
				//First override
				if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current3))
					funcStruct[i].result[Result_Current] = funcStruct[i].consts[FuncConst_Ovrr1];

				//Second override
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current4))
					funcStruct[i].result[Result_Next] = funcStruct[i].consts[FuncConst_Ovrr2];

				//Decrement if input state 1 has changed and has correct edge
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current1))
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
				else if(__PDM_FUNCTION_EDGE_CONDITION(funcStruct[i], FuncIn_Current2))
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

				//Set current count and retVal if it has changed
				if(funcStruct[i].result[Result_Current] != funcStruct[i].result[Result_Next])
				{
					funcStruct[i].result[Result_Current] = funcStruct[i].result[Result_Next];
					retVal |= funcStruct[i].inUse;
				}


				break;

			default:
				continue;
		}

		//Process result inversion and delay if there was an update
		if(updateFlag == 1)
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
		//Start or stop timer according to next result (equal to input condition in this case)
		if(funcStruct->result[Result_Next] == Result_True)
			osTimerStart(funcStruct->funcTimer, pdMS_TO_TICKS(funcStruct->funcDelay[Time_True]*10));

		else if(osTimerIsRunning(funcStruct->funcTimer) == 1)
			osTimerStop(funcStruct->funcTimer);

		funcStruct->result[Result_Current] = funcStruct->result[Result_Next];
		retVal = funcStruct->inUse;
	}

	else
	{
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
			osTimerStart(funcStruct->funcTimer, pdMS_TO_TICKS(funcStruct->funcDelay[aux]*10));
	}

	//Invert result if configured
	if(funcStruct->invert == 1)
		funcStruct->result[Result_Current] = !funcStruct->result[Result_Current];

	return retVal;
}

//Read the module's analog inputs (currents, temperature and voltage)
//Set wait time for next reading and start hardware Timer
//TIM_HandleTypeDef* htim - Pointer to hardware Timer peripheral handle
//PDM_Data_Type* readingFlag - Pointer to array containing flag of data to read now and flag of last processed data
//uint16_t* adcBuffer - Pointer to buffer used by ADC to store read inputs
//uint16_t* dataBuffer - Pointer to Thread safe data buffer
static void Read_Analog_Inputs(TIM_HandleTypeDef* htim, PDM_Data_Type* readingFlag, uint16_t* adcBuffer, int16_t* dataBuffer)
{
	//Convert ADC value based on selected reading and sets delay for next reading
	switch(readingFlag[0])
	{
		//Convert ADC to output current and set next reading to second bank of outputs
		case Data_Type_Current0:

			dataBuffer[Data_Curr1] = 0;
			dataBuffer[Data_Curr3] = 0;
			dataBuffer[Data_Curr5] = 0;
			dataBuffer[Data_Curr7] = 0;
			dataBuffer[Data_Curr9] = 0;
			dataBuffer[Data_Curr11] = 0;
			dataBuffer[Data_Curr13] = 0;
			dataBuffer[Data_Curr15] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			readingFlag[0] = Data_Type_Current1;	//Set flag for next type of analog reading
			readingFlag[1] = Data_Type_Current0;	//Set flag for last type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;

		//Convert ADC to output current and set next reading to HSD temperature
		case Data_Type_Current1:

			dataBuffer[Data_Curr2] = 0;
			dataBuffer[Data_Curr4] = 0;
			dataBuffer[Data_Curr6] = 0;
			dataBuffer[Data_Curr8] = 0;
			dataBuffer[Data_Curr10] = 0;
			dataBuffer[Data_Curr12] = 0;
			dataBuffer[Data_Curr14] = 0;
			dataBuffer[Data_Curr16] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			readingFlag[0] = Data_Type_Voltage;	//Set flag for next type of analog reading
			readingFlag[1] = Data_Type_Current0;	//Set flag for last type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_VOLT);	//set timer period to next type of analog reading

			break;

		//Convert ADC to input voltage and set next reading to first bank of output currents
		case Data_Type_Voltage:

			dataBuffer[Data_Volt] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag[0] = Data_Type_Temperature;	//Set flag for next type of analog reading
			readingFlag[1] = Data_Type_Current0;	//Set flag for last type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_TEMP);	//set timer period to next type of analog reading

			break;

		//Convert ADC to HSD temperature and set next reading to input voltage
		case Data_Type_Temperature:

			dataBuffer[Data_Temp1] = 0;
			dataBuffer[Data_Temp2] = 0;
			dataBuffer[Data_Temp3] = 0;
			dataBuffer[Data_Temp4] = 0;
			dataBuffer[Data_Temp5] = 0;
			dataBuffer[Data_Temp6] = 0;
			dataBuffer[Data_Temp7] = 0;
			dataBuffer[Data_Temp8] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag[0] = Data_Type_Current0;	//Set flag for next type of analog reading
			readingFlag[1] = Data_Type_Current0;	//Set flag for last type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;

		//If readingsFlag is set to initialize or is unspecified, set next reading to output current
		default:
			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag[0] = Data_Type_Current0;	//Set flag for next type of analog reading
			readingFlag[1] = Data_Type_Current0;	//Set flag for last type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;
	}

	//Convert ADC into MCU temperature
	dataBuffer[Data_TempMCU] = 0;

	__HAL_TIM_SET_COUNTER(htim, 0);	//Reset timer counter
	HAL_TIM_Base_Start_IT(htim);	//Start readings timer

	return;
}

//Check if any output current is above threshold
//Start Timer for outputs above Threshold or already in open circuit
//PDM_Output_Ctrl_Struct* outStruct - pointer to array of output structs
//PDM_Data_Type dataRead - Flag indicating which bank of output Fuses to check
//uint16_t* dataBuffer - Pointer to Thread safe data buffer
static void Fuse_Check_Status(PDM_Output_Fuse_Struct* fuseStruct, PDM_Data_Type dataRead, int16_t* dataBuffer)
{
	uint8_t i, j;	//fuseStruct buffer and dataBuffer minimum and maximum index

	//Select first output to be checked base on last reading
	//Will return if no current was read
	switch(dataRead)
	{
		case Data_Type_Current0:
			i = Data_Curr1;
			j = Data_Curr15;
			break;

		case Data_Type_Current1:
			i = Data_Curr2;
			j = Data_Curr16;
			break;

		default:
			return;
	}

	//Check fuse status for each even or odd number output
	for(; i <= j; i++)
	{
		//Check if fuse is enabled
		//Skip loop execution if max current is zero
		if(fuseStruct[i].maxCurrent != 0)
			continue;

		//Enters if current is below allowed and previous fuse status was waiting for circuit opening
		if((dataBuffer[i] <= fuseStruct[i].maxCurrent) && (fuseStruct[i].status == Fuse_Wait))
		{
			fuseStruct[i].status = Fuse_Closed;	//Set fuse status to closed circuit
			fuseStruct[i].retryCount = 0;		//Reset number of re-closing retry count

			if(osTimerIsRunning(fuseStruct[i].osTimer) == 1)
				osTimerStop(fuseStruct[i].osTimer);	//Stop timeout timer
		}

		//Check if timer isn't already running
		else if(osTimerIsRunning(fuseStruct[i].osTimer) == 0)
		{
			//Enter if current is above allowed and fuse status indicates closed circuit
			if((dataBuffer[i] > fuseStruct[i].maxCurrent) && (fuseStruct[i].status != Fuse_Open))
			{
				fuseStruct[i].status = Fuse_Wait;

				//Set waiting time for disarming for the first time
				if(fuseStruct[i].retryCount == 0)
					osTimerStart(fuseStruct[i].osTimer,
							pdMS_TO_TICKS(fuseStruct[i].timeout[Fuse_Time_First]));

				//Set waiting time for disarming after the first attempt
				else
					osTimerStart(fuseStruct[i].osTimer,
							pdMS_TO_TICKS(fuseStruct[i].timeout[Fuse_Time_Open]));
			}

			//Enter if there are re-closing retry attempts available and fuse status is open
			else if(((fuseStruct[i].retryCount < fuseStruct[i].retry)
					|| (fuseStruct[i].retry == FUSE_RETRY_INF))
					&& (fuseStruct[i].status == Fuse_Open))
				osTimerStart(fuseStruct[i].osTimer, pdMS_TO_TICKS(fuseStruct[i].timeout[Fuse_Time_Close]));
		}
	}

	return;
}

/*END STATIC FUNCTIONS*/
