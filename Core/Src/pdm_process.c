/*
 * pdm_channels.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Rodolfo
 */

#include "pdm.h"

static uint8_t Channel_CAN_Process(PDM_Data_Queue_Struct* rxStruct, PDM_Data_Channel_Struct* dataStruct, uint8_t nbrOfChannels);
static uint8_t Channel_Local_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Type dataRead, int16_t* dataBuffer);
static uint8_t Function_Input_Process(PDM_Function_Struct* dataStruct, PDM_Input_Struct* inStruct, int16_t* dataBuffer);
static uint8_t Function_Result_Process(PDM_Function_Struct* funcStruct);
static void Data_Cast(PDM_Data_Channel_Struct* dataStruct, uint8_t* buffer);
static void Data_Timeout_Callback(void* dataStruct);
static void Read_Analog_Inputs(TIM_HandleTypeDef* htim, PDM_Data_Type* readingFlag, uint16_t* adcBuffer, uint16_t* dataBuffer);
static void Fuse_Check_Status(PDM_Output_Ctrl_Struct* outStruct, PDM_Data_Type dataRead, uint16_t* dataBuffer);
static void Fuse_Timer_Callback(void* funcStruct);

/*BEGIN FUNCTIONS*/

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
		if(osMessageQueueGet(thrdStr->intQueueHandle, (void*) rxStruct, 0, osWaitForever) == osOK)
		{
			switch(rxStruct.source)
			{
				case Interrupt_CAN:
					//Try to convert data for each CAN data channel
					updateFlag = Channel_CAN_Process(&rxStruct, thrdStr->channels, thrdStr->nbrOfChannels);

					break;

				case Interrupt_Gpio:
					//Read all input pins, if any pin is used, set flag
					updateFlag = Function_Input_Process(thrdStr->functions, thrdStr->inStruct, dataBuffer);

					break;

				case Interrupt_Timer:
					//Read analog inputs and place values in thread safe buffer
					Read_Analog_Inputs(thrdStr->htim, readingFlag, thrdStr->adcBuffer, dataBuffer);

					//Check currents and fuse status for each output
					Fuse_Check_Status(thrdStr->outStruct->fuseStruct, readingFlag[1], dataBuffer);

					//Check if any channel uses local data and if yes, change channel value and update flag
					updateFlag = Channel_Local_Process(thrdStr->channels, readingFlag[1], dataBuffer);

					if(osMutexAcquire(thrdStr->dataMutexHandle, 0) == osOK)	//Check if data buffer is safe to access
					{
						osMutexRelease(thrdStr->dataMutexHandle);	//Release data buffer Mutex
					}

					break;
			}
		}

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

static uint8_t Channel_CAN_Process(PDM_Data_Queue_Struct* rxStruct, PDM_Data_Channel_Struct* dataStruct, uint8_t nbrOfChannels)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t aux[2];


	for(uint8_t i = CHANNEL_CAN_OFFSET; i < (CHANNEL_CAN_OFFSET+nbrOfChannels); i++)
	{
		//Continue "for loop" if CAN bus frame id doesn't match channel filter or channel isn't used for any conditions
		if((dataStruct[i]->dataFilter != rxStruct->id)
				|| (dataStruct[i]->inUse == IN_USE_NONE))
			continue;

		//Get data from Queue according to data source
		//Return if channel source isn't from CAN bus
		switch(dataStruct[i]->source)
		{
			//For fixed position data in specific frame
			//Will continue "for loop" if cast type is invalid
			case Data_CAN_Fixed:
			//{
				if((dataStruct[i]->cast == Cast_Uint8) || (dataStruct[i]->cast == Cast_Int8))
					aux[0] = rxStruct->data[dataStruct[i]->position];

				else if((dataStruct[i]->cast == Cast_Uint16) || (dataStruct[i]->cast == Cast_Int16))
				{
					aux[0] = rxStruct->data[dataStruct[i]->position];
					aux[1] = rxStruct->data[dataStruct[i]->position + 1];
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
					if(dataStruct[i]->position == ((rxStruct->data[1] << 8) | rxStruct->data[0]))
					{
						aux[0] = rxStruct->data[2];
						aux[1] = rxStruct->data[3];
					}

					else if((dataStruct[i]->position == ((rxStruct->data[4] << 8) | rxStruct->data[5])) && (rxStruct->length == 8))
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
		if(osTimerStart(dataStruct[i]->timer, pdMS_TO_TICKS(dataStruct[i]->timeout)) == osOK)
			dataStruct[i]->timeoutFlag = CAN_Data_Refresh;

		//Reset value and set flag to timeout if timer cannot be started
		else
		{
			Data_Timeout_Callback((void*) dataStruct);
			continue;
		}

		//Cast received data to correct format
		Data_Cast(dataStruct, aux);

		//Set retVal if CAN data is used in Functions or PWM
		retVal |= dataStruct[i]->inUse;
	}

	return retVal;
}

static uint8_t Channel_Local_Process(PDM_Data_Channel_Struct* dataStruct, PDM_Data_Type dataRead, int16_t* dataBuffer)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t index[2];	//Minimum and maximum index for dataBuffer

	for(uint8_t i = CHANNEL_ANALOG_OFFSET; i < CHANNEL_ANALOG_FINISH; i++)
	{
		//Return if channel source isn't Local or channel isn't used in any conditions
		if((dataStruct[i]->source != Data_ADC)
				|| (dataStruct[i]->inUse == IN_USE_NONE))
			continue;

		//Check if channel uses MCU Temperature
		//This check exists because the temperature is read at every ADC processing cycle
		if(dataStruct[i]->position == Data_TempMCU)
			dataStruct[i]->data = (int32_t) dataBuffer[Data_TempMCU];

		else
		{
			//Select channels to update based on last data read
			switch(dataRead)
			{
				case Data_Type_Current0:
					index[0] = Data_Curr1;
					index[1] = Data_Curr15;
					break;

				case Data_Type_Current1:
					index[0] = Data_Curr2;
					index[1] = Data_Curr16;
					break;

				case Data_Type_Temperature:
					index[0] = Data_Temp1;
					index[1] = Data_Temp8;
					break;

				case Data_Type_Voltage:
					index[0] = Data_Volt;
					index[1] = Data_Volt;
					break;

				default:
					continue;
			}
		}

		//Check for matching data ID
		for(; index[0] <= index[1]; index[0]++)
		{
			if(dataStruct[i]->position == index[0])
			{
				dataStruct->data = (int32_t) dataBuffer[index[0]];
				retVal |= dataStruct[i]->inUse;
			}
		}
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

	for(uint8_t i = FUNCTION_INPUT_OFFSET; i < FUNCTION_INPUT_FINISH; i++)
	{
		//Read input pin level and set data value
		funcStruct[i]->result[Result_Next] = HAL_GPIO_ReadPin(inStruct->gpio[i], inStruct->pin[i]);

		//Change value in Thread safe buffer
		dataBuffer[Data_Input] |= (funcStruct[i]->result[Result_Next] << i);

		//Skip if Input Pin is unused or next and current Results are equal
		if((funcStruct[i]->inUse == IN_USE_NONE)
				|| (funcStruct[i]->result[Result_Next] == funcStruct[i]->result[Result_Current]))
			continue;

		retVal |= Function_Result_Process(&funcStruct[i]);
	}

	return retVal;
}

static uint8_t Function_Result_Process(PDM_Function_Struct* funcStruct)
{
	//Store return flags
	uint8_t retVal = PROCESS_NONE;

	//Store next Result to process according to correct Delay
	uint8_t aux;

	//Set aux variable to Process correct result Timer condition
	aux = funcStruct->result[Result_Next];

	//Set current Result and retVal if there is no Delay
	if(funcStruct->funcDelay[aux] == 0)
	{
		funcStruct->result[Result_Next] = funcStruct->result[Result_Current];

		retVal |= funcStruct->inUse;
	}

	//Start Timer to wait Delay
	else
		osTimerStart(funcStruct->funcTimer, pdMS_TO_TICKS(funcStruct->funcDelay[aux]*10));

	return retVal;
}

static void Data_Cast(PDM_Data_Channel_Struct* dataStruct, uint8_t* buffer)
{
	uint8_t aux = 0;

	//Cast data read from CAN bus to accommodate correct negative and positive values
	//Use data mask in case of unsigned data and byte swap in case of 16 bit data
	switch(dataStruct->cast)
	{
		case Cast_Uint8:
			dataStruct->data = (int32_t) (buffer[0] & dataStruct->mask);
			break;

		case Cast_Int8:
			dataStruct->data = (int32_t) ((int8_t) buffer[0]);
			break;

		case Cast_Uint16:

			if(dataStruct->alignment == Alignment_Swap)
			{
				aux = buffer[0];
				buffer[0] = buffer[1];
				buffer[1] = aux;
			}

			dataStruct->data = (int32_t) ((uint16_t) ((buffer[0] << 8) | buffer[1]) & dataStruct->mask);

			break;

		case Cast_Int16:

			if(dataStruct->alignment == Alignment_Swap)
			{
				aux = buffer[0];
				buffer[0] = buffer[1];
				buffer[1] = aux;
			}

			dataStruct->data = (int32_t) ((int16_t) ((buffer[0] << 8) | buffer[1]));

			break;
	}

	return;
}

static void Read_Analog_Inputs(TIM_HandleTypeDef* htim, PDM_Data_Type* readingFlag, uint16_t* adcBuffer, uint16_t* dataBuffer)
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

static void Fuse_Check_Status(PDM_Output_Ctrl_Struct* outStruct, PDM_Data_Type dataRead, uint16_t* dataBuffer)
{
	uint8_t i[2];	//fuseStruct buffer and dataBuffer minimum and maximum index

	//Select first output to be checked base on last reading
	//Will return if no current was read
	switch(dataRead)
	{
		case Data_Type_Current0:
			i[0] = Data_Curr1;
			i[1] = Data_Curr15;
			break;

		case Data_Type_Current1:
			i[0] = Data_Curr2;
			i[1] = Data_Curr16;
			break;

		default:
			return;
	}

	//Check fuse status for each even or odd number output
	for(; i[0] <= i[1]; i[0]++)
	{
		//Check if fuse is enabled
		//Skip current execution if fuse is disabled
		if(outStruct->fuseEnable == Fuse_Disabled)
			continue;

		//Enters if current is below allowed and previous fuse status was waiting for circuit opening
		if((dataBuffer[i[0]] <= outStruct[i[0]]->fuseStruct->maxCurrent) && (outStruct[i[0]]->fuseStruct->status == Fuse_Wait))
		{
			outStruct[i[0]]->fuseStruct->status = Fuse_Closed;	//Set fuse status to closed circuit
			outStruct[i[0]]->fuseStruct->retryCount = 0;	//Reset number of re-closing retry count

			if(osTimerIsRunning(outStruct[i[0]]->fuseStruct->osTimer) == 1)
				osTimerStop(outStruct[i[0]]->fuseStruct->osTimer);	//Stop timeout timer
		}

		//Check if timer isn't already running
		else if(osTimerIsRunning(outStruct[i[0]]->fuseStruct->osTimer) == 0)
		{
			//Enter if current is above allowed and fuse status indicates closed circuit
			if((dataBuffer[i[0]] > outStruct[i[0]]->fuseStruct->maxCurrent) && (outStruct[i[0]]->fuseStruct->status != Fuse_Open))
			{
				outStruct[i[0]]->fuseStruct->status = Fuse_Wait;

				//Set waiting time for disarming for the first time
				if(outStruct[i[0]]->fuseStruct->retryCount == 0)
					osTimerStart(outStruct[i[0]]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i[0]]->fuseStruct->timeout[Fuse_Time_First]));

				//Set waiting time for disarming after the first attempt
				else
					osTimerStart(outStruct[i[0]]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i[0]]->fuseStruct->timeout[Fuse_Time_Open]));
			}

			//Enter if there are re-closing retry attempts available and fuse status is open
			else if((outStruct[i[0]]->fuseStruct->retryCount < outStruct[i[0]]->fuseStruct->retry)
					&& (outStruct[i[0]]->fuseStruct->status == Fuse_Open))
				osTimerStart(outStruct[i[0]]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i[0]]->fuseStruct->timeout[Fuse_Time_Close]));
		}
	}

	return;
}

static void Fuse_Timer_Callback(void* funcStruct)
{
	PDM_Output_Fuse_Struct* fncStr = (PDM_Output_Fuse_Struct*) funcStruct;

	//Enter if fuse status is set to wait
	if(fncStr->status == Fuse_Wait)
	{
		fncStr->retryCount++;	//Increments number of retry counts
		fncStr->status = Fuse_Open;	//Set fuse to open
	}

	//Enter if fuse status is set to open
	else if(fncStr->status == Fuse_Open)
		fncStr->status = Fuse_Wait;	//Set fuse to close at waiting state

	osSemaphoreRelease(fncStr->outSemaphore);	//Release Semaphore to output Thread for it to open or close the output

	return;
}

/*END STATIC FUNCTIONS*/

/*BEGIN CALLBACK FUNCTIONS*/

//Function that indicates data reception timeout and places predefined value if configured
static void Data_Timeout_Callback(void* dataStruct)
{
	PDM_Data_Channel_Struct* dataStr = (PDM_Data_Channel_Struct*) dataStruct;

	//Change data value if timeout happens
	if(dataStr->keep == Data_Reset)
		Data_Cast(dataStr, (uint8_t*) &dataStr->defaultVal);

	//Set flag indicating timeout
	dataStr->timeoutFlag = CAN_Data_Timeout;

	return;
}

static void Function_Delay_Callback(void* funcStruct)
{
	PDM_Function_Struct* funcStr = (PDM_Function_Struct* funcStruct);

	//Change current result state


	return;
}

/*END  CALLBACK FUNCTIONS*/
