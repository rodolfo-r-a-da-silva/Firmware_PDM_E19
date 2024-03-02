/*
 * pdm_fuse.c
 *
 *  Created on: Jan 4, 2024
 *      Author: Rodolfo
 */

#include "pdm.h"

/*BEGIN STATIC FUNCTION PROTOTYPES*/
static void Analog_Read(TIM_HandleTypeDef* htim, PDM_Reading_Type* readingType, uint16_t* adcBuffer, int16_t* auxBuffer);
static uint8_t Digital_Read(PDM_Channel_Local_Struct* chnStruct, PDM_Input_Struct* inStruct);
static uint8_t Convert_Filter(PDM_Channel_Local_Struct* chnStruct, int16_t adcVar);
static void Fuse_Check_Status(PDM_Output_Fuse_Struct* fuseStruct);
/*END STATIC FUNCTION PROTOTYPES*/

/*BEGIN FUNCTIONS*/

/*END FUNCTIONS*/

/*BEGIN THREADS*/

//Manages analog readings (currents, temperatures and module voltage)
//
void PDM_Readings_Thread(void* threadStruct)
{
	PDM_Readings_Thread_Struct* thrdStr = (PDM_Readings_Thread_Struct*) threadStruct;

	uint8_t processFlag;	//Indicate if any function or output must be updated
	int16_t auxBuffer[NBR_OF_CHN_LCL_TOTAL];	//Store ADC values
	PDM_Reading_Type readType = Reading_Init;	//Store next reading type

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		//Wait for timer to release Semaphore
		if(osSemaphoreAcquire(thrdStr->semHandle, osWaitForever) == osOK)
		{
			//Read analog inputs according to HSD mux
			Analog_Read(thrdStr->htim, &readType, thrdStr->adcBuffer, auxBuffer);

			//Convert and filter analog data, and check for fuse activation
			if(readType == Reading_Current0)
			{
				//Reset processing flag
				processFlag = 0;

				//Read external inputs
				for(uint8_t i = CHN_LCL_DIG_IN_OFFSET; i < CHN_LCL_DIG_IN_FINISH; i++)
					processFlag |= Digital_Read(&thrdStr->localChannels[i], &thrdStr->inputs[i]);

				//Convert and filter each local channel
				for(uint8_t i = CHN_LCL_ANL_IN_OFFSET; i < NBR_OF_CHN_LCL_TOTAL; i++)
					processFlag |= Convert_Filter(&thrdStr->localChannels[i], auxBuffer[i]);

				Fuse_Check_Status(thrdStr->fuses);

				if((processFlag & PROCESS_FUNCTION) == PROCESS_FUNCTION)
					osSemaphoreRelease(thrdStr->procSemHandle);

				if((processFlag & PROCESS_PWM) == PROCESS_PWM)
					osMessageQueuePut(thrdStr->outQueueHandle, (void*) &processFlag, 0, 0);
			}
		}

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit();	//Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}
/*END THREADS*/

/*BEGIN STATIC FUNCTIONS*/

//Read the module's analog inputs (currents, temperature and voltage)
//Set wait time for next reading and start hardware Timer
//TIM_HandleTypeDef* htim - Pointer to hardware Timer peripheral handle
//PDM_Data_Type* readingType - Pointer to array containing flag of data to read now and flag of last processed data
//uint16_t* adcBuffer - Pointer to buffer used by ADC to store read inputs
//int16_t* auxBuffer - Pointer to temporary storage array of ADC data
static void Analog_Read(TIM_HandleTypeDef* htim, PDM_Reading_Type* readingType, uint16_t* adcBuffer, int16_t* auxBuffer)
{
	//Convert ADC value based on selected reading and sets delay for next reading
	switch(*readingType)
	{
		//Convert ADC to output current and set next reading to second bank of outputs
		case Reading_Current0:

			for(uint8_t i = 1, j = Data_Curr1; j <= Data_Curr15; i++, j+=2)
				auxBuffer[j] = adcBuffer[i];

//			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			*readingType = Reading_Current1;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;

		//Convert ADC to output current and set next reading to HSD temperature
		case Reading_Current1:

			for(uint8_t i = 1, j = Data_Curr2; j <= Data_Curr16; i++, j+=2)
				auxBuffer[j] = adcBuffer[i];

			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

			*readingType = Reading_Voltage;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_VOLT);	//set timer period to next type of analog reading

			break;

		//Convert ADC to input voltage and set next reading to first bank of output currents
		case Reading_Voltage:

			auxBuffer[Data_Volt] = adcBuffer[1];

//			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			*readingType = Reading_Temperature;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_TEMP);	//set timer period to next type of analog reading

			break;

		//Convert ADC to HSD temperature and set next reading to input voltage
		case Reading_Temperature:

			for(uint8_t i = 1, j = Data_Temp1; j <= Data_Temp8; i++, j++)
				auxBuffer[j] = adcBuffer[i];

			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			*readingType = Reading_Current0;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;

		//If readingType is set to initialize or is unspecified, set next reading to output current
		default:
			HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

			*readingType = Reading_Current0;	//Set flag for next type of analog reading
			__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);	//set timer period to next type of analog reading

			break;
	}

	//Convert ADC into MCU temperature
	auxBuffer[Data_TempMCU] = adcBuffer[0];

	__HAL_TIM_SET_COUNTER(htim, 0);	//Reset timer counter
	HAL_TIM_Base_Start_IT(htim);	//Start readings timer

	return;
}

//Read Input Pin level
//PDM_Channel_Local_Struct* chnStruct - Pointer to digital input channel
//PDM_Input_Struct* inStruct - Pointer to struct containing digital input pin information
static uint8_t Digital_Read(PDM_Channel_Local_Struct* chnStruct, PDM_Input_Struct* inStruct)
{
	//Set previous input pin level
	chnStruct->data[Data_Previous] = chnStruct->data[Data_Current];

	//Read current input pin level
	chnStruct->data[Data_Current] = HAL_GPIO_ReadPin(&inStruct->gpio, inStruct->pin);

	//Return used flag only if there was a level change
	if(chnStruct->data[Data_Current] == chnStruct->data[Data_Previous])
		return chnStruct->inUse;

	else
		return PROCESS_NONE;
}

//Convert and filter (IIR) ADC value
//PDM_Channel_Local_Struct* chnStruct - Pointer to the channel's struct to be converted and filtered
//int16_t adcVar - ADC variable for conversion
static uint8_t Convert_Filter(PDM_Channel_Local_Struct* chnStruct, int16_t adcVar)
{
	//Convert ADC value
	int16_t aux = 0;

	//Set old data value
	chnStruct->data[Data_Previous] = chnStruct->data[Data_Current];

	//Calculate new data value via interpolation
	if(adcVar > chnStruct->adc[0])
	{
		//Start loop using last two points for reference
		for(uint8_t i = chnStruct->nbrOfPoints-2; i <= 0; i--)
		{
			if(adcVar >= chnStruct->adc[i])
			{
				aux = PDM_Linear_Interpolation((int32_t) adcVar,
											   chnStruct->adc[i],
											   chnStruct->adc[i+1],
											   chnStruct->value[i],
											   chnStruct->value[i+1]);
				break;
			}
		}
	}

	//Get extrapolated minimum value if below threshold
	else
		aux = chnStruct->value[0];

	//Calculate new filtered data value
	chnStruct->data[Data_Current] = (int32_t) (chnStruct->filter[0]*aux
													+ chnStruct->filter[1]*chnStruct->data[Data_Previous]);

	return chnStruct->inUse;
}

//Check if any output current is above threshold
//Start Timer for outputs above Threshold or already in open circuit
//PDM_Output_Ctrl_Struct* outStruct - Pointer to array of output structs
static void Fuse_Check_Status(PDM_Output_Fuse_Struct* fuseStruct)
{
	//Check fuse status for each output
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		//Check if fuse is enabled
		//Skip loop execution if max current is zero
		if(fuseStruct[i].maxCurrent != 0)
			continue;

		//Enters if current is below allowed and previous fuse status was waiting for circuit opening
		if((*fuseStruct[i].current <= fuseStruct[i].maxCurrent) && (fuseStruct[i].status == Fuse_Wait))
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
			if((*fuseStruct[i].current > fuseStruct[i].maxCurrent) && (fuseStruct[i].status != Fuse_Open))
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
