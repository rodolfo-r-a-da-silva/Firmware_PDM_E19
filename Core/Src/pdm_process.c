/*
 * pdm_channels.c
 *
 *  Created on: Mar 21, 2023
 *      Author: Rodolfo
 */

#include "pdm.h"

static uint8_t CAN_Channel_Process(PDM_Data_Queue_Struct* rxStruct, PDM_Data_Channel_Struct* dataStruct);
static uint8_t Local_Channel_Process(PDM_Data_Channel_Struct* dataStruct, int16_t* dataBuffer);
static void Data_Cast(PDM_Data_Channel_Struct* dataStruct, uint8_t* buffer);
static void Data_Timeout(void* dataStruct);
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

	uint8_t updateFlag = 0;
	uint8_t conditionFlag = 0;
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
					for(uint8_t i = 0; i < thrdStr->nbrOfChannels; i++)
						updateFlag |= CAN_Channel_Process(&rxStruct, &thrdStr->channels[i]);

					break;

				case Interrupt_Gpio:
					break;

				case Interrupt_Timer:

					Read_Analog_Inputs(thrdStr->htim, readingFlag, thrdStr->adcBuffer, dataBuffer);
					Fuse_Check_Status(thrdStr->outStruct->fuseStruct, readingFlag[1], dataBuffer);

					if(osMutexAcquire(thrdStr->dataMutexHandle, 0) == osOK)	//Check if data buffer is safe to access
					{
						osMutexRelease(thrdStr->dataMutexHandle);	//Release data buffer Mutex
					}

					break;
			}
		}

		updateFlag = 0;
		conditionFlag = 0;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit();	//Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREAD FUNCTIONS*/

/*BEGIN STATIC FUNCTIONS*/

static uint8_t CAN_Channel_Process(PDM_Data_Queue_Struct* rxStruct, PDM_Data_Channel_Struct* dataStruct)
{
	uint8_t aux[2];

	//Return if interrupt source isn't from CAN bus
	if(rxStruct->source != Interrupt_CAN)
		return DATA_INVALID;

	//Get data from Queue according to data origin
	switch(dataStruct->source)
	{
		//For fixed position data in specific frame
		//Will continue "for loop" if cast type is invalid
		case Data_CAN_Fixed:
		//{
			if((dataStruct->cast == Cast_Uint8) || (dataStruct->cast == Cast_Int8))
				aux[0] = rxStruct->data[dataStruct->position];

			else if((dataStruct->cast == Cast_Uint16) || (dataStruct->cast == Cast_Int16))
			{
				aux[0] = rxStruct->data[dataStruct->position];
				aux[1] = rxStruct->data[dataStruct->position + 1];
			}

			else
				return DATA_INVALID;

			break;
		//}

		//For data identification inside the frame's data field
		//Will continue "for loop" if frame length is invalid or data isn't available
		case Data_CAN_Channel:
		//{
			if((rxStruct->length != 4) && (rxStruct->length != 8))
			{
				if(dataStruct->position == ((rxStruct->data[1] << 8) | rxStruct->data[0]))
				{
					aux[0] = rxStruct->data[2];
					aux[1] = rxStruct->data[3];
				}

				else if((dataStruct->position == ((rxStruct->data[4] << 8) | rxStruct->data[5])) && (rxStruct->length == 8))
				{
					aux[0] = rxStruct->data[6];
					aux[1] = rxStruct->data[7];
				}

				else
					return DATA_INVALID;
			}

			else
				return DATA_INVALID;

			break;
		//}

		default:
			return DATA_INVALID;
	}

	//Reset timer if data was received and timer was created correctly
	//Set return flag to indicate received value
	if(osTimerStart(dataStruct->timer, pdMS_TO_TICKS(dataStruct->timeout)) == osOK)
		dataStruct->timeoutFlag = CAN_Data_Refresh;

	//Reset value and set flag to timeout if timer cannot be started
	else
	{
		Data_Timeout((void*) dataStruct);
		return DATA_INVALID;
	}

	//Cast received data to correct format
	Data_Cast(dataStruct, aux);

	return DATA_PROCESSED;
}

static uint8_t Local_Channel_Process(PDM_Data_Channel_Struct* dataStruct, int16_t* dataBuffer)
{
	return DATA_PROCESSED;
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

//Function that indicates data reception timeout and places predefined value if configured
static void Data_Timeout(void* dataStruct)
{
	PDM_Data_Channel_Struct* dataStr = (PDM_Data_Channel_Struct*) dataStruct;

	//Change data value if timeout happens
	if(dataStr->keep == Data_Reset)
		Data_Cast(dataStr, (uint8_t*) &dataStr->defaultVal);

	//Set flag indicating timeout
	dataStr->timeoutFlag = CAN_Data_Timeout;

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
	uint8_t i = 0;	//fuseStruct buffer and dataBuffer index
	uint8_t j = 0;	//Maximum index allowed

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
		//Skip current execution if fuse is disabled
		if(outStruct->fuseEnable == Fuse_Disabled)
			continue;

		//Enters if current is below allowed and previous fuse status was waiting for circuit opening
		if((dataBuffer[i] <= outStruct[i]->fuseStruct->maxCurrent) && (outStruct[i]->fuseStruct->status == Fuse_Wait))
		{
			outStruct[i]->fuseStruct->status = Fuse_Closed;	//Set fuse status to closed circuit
			outStruct[i]->fuseStruct->retryCount = 0;	//Reset number of re-closing retry count

			if(osTimerIsRunning(outStruct[i]->fuseStruct->osTimer) == 1)
				osTimerStop(outStruct[i]->fuseStruct->osTimer);	//Stop timeout timer
		}

		//Check if timer isn't already running
		else if(osTimerIsRunning(outStruct[i]->fuseStruct->osTimer) == 0)
		{
			//Enter if current is above allowed and fuse status indicates closed circuit
			if((dataBuffer[i] > outStruct[i]->fuseStruct->maxCurrent) && (outStruct[i]->fuseStruct->status != Fuse_Open))
			{
				outStruct[i]->fuseStruct->status = Fuse_Wait;

				//Set waiting time for disarming for the first time
				if(outStruct[i]->fuseStruct->retryCount == 0)
					osTimerStart(outStruct[i]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i]->fuseStruct->timeout[Fuse_Time_First]));

				//Set waiting time for disarming after the first attempt
				else
					osTimerStart(outStruct[i]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i]->fuseStruct->timeout[Fuse_Time_Open]));
			}

			//Enter if there are re-closing retry attempts available and fuse status is open
			else if((outStruct[i]->fuseStruct->retryCount < outStruct[i]->fuseStruct->retry)
					&& (outStruct[i]->fuseStruct->status == Fuse_Open))
				osTimerStart(outStruct[i]->fuseStruct->osTimer, pdMS_TO_TICKS(outStruct[i]->fuseStruct->timeout[Fuse_Time_Close]));
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
