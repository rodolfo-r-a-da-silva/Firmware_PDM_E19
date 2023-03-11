/*
 * pdm_readings.c
 *
 *  Created on: Nov 20, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static void Fuse_Check_Status(osTimerId_t* timerId, PDM_Output_Fuse_Struct* fuseStruct, int16_t current);
static void Fuse_Timer_Callback(void* funcStruct);

//Reads ADC value and converts into data and sets virtual fuse current flags
//Returns HAL_TIM_Base_Start_IT status
void PDM_Readings_Thread(void* threadStruct)
{
	//Struct containing Timer peripheral handle and semaphore ID
	PDM_Readings_Thread_Struct* thrdStr = (PDM_Readings_Thread_Struct*) threadStruct;

	uint8_t fuseFlag = 0;	//Sets to 1 if there is any virtual fuse that needs to be opened
	uint8_t fuseTimerStatus;	//Checks if fuse timer is running
	osTimerId_t fuseTimerId[NBR_OF_OUTPUTS];	//Timers for output fuse activation in case of overcurrent

	PDM_Data_Type readingFlag = Data_Type_Init;	//Flag for data to be read from HSDs, starts by setting reading to current

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		//Convert ADC value based on selected reading and sets delay for next reading
		switch(readingFlag)
		{
		//Convert ADC to output current and set next reading to second bank of outputs
		case Data_Type_Current0:

			thrdStr->dataBuffer[Data_Curr1] = 0;
			thrdStr->dataBuffer[Data_Curr3] = 0;
			thrdStr->dataBuffer[Data_Curr5] = 0;
			thrdStr->dataBuffer[Data_Curr7] = 0;
			thrdStr->dataBuffer[Data_Curr9] = 0;
			thrdStr->dataBuffer[Data_Curr11] = 0;
			thrdStr->dataBuffer[Data_Curr13] = 0;
			thrdStr->dataBuffer[Data_Curr15] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			readingFlag = Data_Type_Current1;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(thrdStr->htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			//Check current and virtual fuse status
			for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i+=2)
			{
				Fuse_Check_Status(fuseTimerId[i], thrdStr->outStruct[i]->fuseStruct, thrdStr->dataBuffer[i]);
			}

			break;

		//Convert ADC to output current and set next reading to HSD temperature
		case Data_Type_Current1:

			thrdStr->dataBuffer[Data_Curr2] = 0;
			thrdStr->dataBuffer[Data_Curr4] = 0;
			thrdStr->dataBuffer[Data_Curr6] = 0;
			thrdStr->dataBuffer[Data_Curr8] = 0;
			thrdStr->dataBuffer[Data_Curr10] = 0;
			thrdStr->dataBuffer[Data_Curr12] = 0;
			thrdStr->dataBuffer[Data_Curr14] = 0;
			thrdStr->dataBuffer[Data_Curr16] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			readingFlag = Data_Type_Voltage;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(thrdStr->htim, READING_DELAY_VOLT);	//set timer period to next type of analog reading

			//Check current and virtual fuse status
			for(uint8_t i = 1; i < NBR_OF_OUTPUTS; i+=2)
			{
				Fuse_Check_Status(fuseTimerId[i], thrdStr->outStruct[i]->fuseStruct, thrdStr->dataBuffer[i]);
			}

			break;

		//Convert ADC to input voltage and set next reading to first bank of output currents
		case Data_Type_Voltage:

			thrdStr->dataBuffer[Data_Volt] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag = Data_Type_Temperature;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(thrdStr->htim, READING_DELAY_TEMP);	//set timer period to next type of analog reading

			break;

		//Convert ADC to HSD temperature and set next reading to input voltage
		case Data_Type_Temperature:

			thrdStr->dataBuffer[Data_Temp1] = 0;
			thrdStr->dataBuffer[Data_Temp2] = 0;
			thrdStr->dataBuffer[Data_Temp3] = 0;
			thrdStr->dataBuffer[Data_Temp4] = 0;
			thrdStr->dataBuffer[Data_Temp5] = 0;
			thrdStr->dataBuffer[Data_Temp6] = 0;
			thrdStr->dataBuffer[Data_Temp7] = 0;
			thrdStr->dataBuffer[Data_Temp8] = 0;

			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag = Data_Type_Current0;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(thrdStr->htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;

		//If readingsFlag is set to initialize or is unspecified, set next reading to output current
		default:
			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			readingFlag = Data_Type_Current0;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(thrdStr->htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;
		}

		//Convert ADC into MCU temperature
		thrdStr->dataBuffer[Data_TempMCU] = (__PDM_LINEAR_INTERPOLATION(thrdStr->adcBuffer[9], ADC_MCU_TEMP_VAL0, ADC_MCU_TEMP_VAL1, ADC_MCU_TEMP_VOLT0, ADC_MCU_TEMP_VOLT1)) / ADC_12_BIT_DIVISION;

		__HAL_TIM_SET_COUNTER(thrdStr->htim, 0);	//Reset timer counter
		HAL_TIM_Base_Start_IT(thrdStr->htim);	//Start readings timer

		osSemaphoreAcquire(thrdStr->semaphoreHandle, osWaitForever);	//Suspend until reading delay is elapsed and semaphore is released

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

static void Fuse_Check_Status(osTimerId_t* timerId, PDM_Output_Fuse_Struct* fuseStruct, int16_t current)
{
	//Enters if current is below allowed and previous fuse status was waiting for circuit opening
	if((current <= fuseStruct->fuseCurrent) && (fuseStruct->fuseStatus == Fuse_Wait))
	{
		fuseStruct->fuseStatus = Fuse_Closed;	//Set fuse status to closed circuit
		fuseStruct->fuseRetryCount = 0;	//Reset number of re-closing retry count

		osTimerStop(*timerId);	//Stop timeout timer
	}

	//Enter if current is above allowed and fuse status indicates closed circuit
	else if((current > fuseStruct->fuseCurrent) && (fuseStruct->fuseStatus == Fuse_Closed))
	{
		fuseStruct->fuseStatus = Fuse_Wait;

		//If timer isn't running, the start it
		if(osTimerIsRunning(*timerId) == 1)
		{
			//Set waiting time for disarming for the first time
			if(fuseStruct->fuseRetryCount == 0)
				osTimerStart(*timerId, pdMS_TO_TICKS(fuseStruct->fuseTimeout[Fuse_Time_First]));

			//Set waiting time for disarming after the first attempt
			if(fuseStruct->fuseRetryCount > 0)
				osTimerStart(*timerId, pdMS_TO_TICKS(fuseStruct->fuseTimeout[Fuse_Time_Open]));
		}
	}

	//Enter if there are re-closing retry attempts available, fuse fuse status is open and the timer isn't running yet
	else if((fuseStruct->fuseRetryCount < fuseStruct->fuseRetry) && (fuseStruct->fuseStatus == Fuse_Open) && (osTimerIsRunning(*timerId) == 0))
		osTimerStart(*timerId, pdMS_TO_TICKS(fuseStruct->fuseTimeout[Fuse_Time_Close]));

	return;
}

static void Fuse_Timer_Callback(void* funcStruct)
{
	PDM_Output_Fuse_Struct* fncStr = (PDM_Output_Fuse_Struct*) funcStruct;

	//Enter if fuse status is set to wait
	if(fncStr->fuseStatus == Fuse_Wait)
	{
		fncStr->fuseRetryCount++;	//Increments number of retry counts
		fncStr->fuseStatus = Fuse_Open;	//Set fuse to open
	}

	//Enter if fuse status is set to open
	else if(fncStr->fuseStatus == Fuse_Open)
		fncStr->fuseStatus = Fuse_Wait;	//Set fuse to close at waiting state

	osSemaphoreRelease(fncStr->outSemaphore);	//Release Semaphore to output Thread for it to open or close the output

	return;
}
