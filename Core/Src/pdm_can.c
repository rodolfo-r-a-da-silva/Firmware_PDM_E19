/*
 * pdm_can.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static HAL_StatusTypeDef Filter_Config(CAN_HandleTypeDef* hcan, uint32_t filter, uint8_t filterNbr);
static HAL_StatusTypeDef Tx_Channel(CAN_HandleTypeDef* hcan, Data_Freq dataFreq, Data_Freq* dataFreqBuffer, uint16_t* dataBuffer, uint16_t* dataIdBuffer);
static HAL_StatusTypeDef Tx_Fixed(CAN_HandleTypeDef* hcan, PDM_CAN_TxMsgType dataMsg, uint16_t* dataBuffer);
static uint8_t Rx_Data_Prepare(CAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData, PDM_CAN_Data_Struct* dataStruct, uint8_t dataNbr);
static void Rx_Data_Timeout(void* dataStruct);

/*BEGIN FUNCTIONS*/

//Initializes CAN bus communication
//PDM_CAN_Config* fltr_str - struct containing CAN handle, baud rate and filters
//Returns HAL_CAN_Start status
HAL_StatusTypeDef PDM_CAN_Init(PDM_CAN_Config_Struct* filterStruct)
{
	//Deinitialize CAN peripheral if baud rate is different than 1000 kbps
	if(filterStruct->baudRate != CAN_1000kbps)
	{
		//Deinitialize CAN bus for new configuration
		HAL_CAN_DeInit(filterStruct->hcan);

		//Sets CAN prescaler to match selected baud rate
		//If CAN bus is configured as disabled, leaves the function without initialization
		switch(filterStruct->baudRate)
		{
			case CAN_125kbps:
				filterStruct->hcan->Init.Prescaler = 40;
				break;

			case CAN_250kbps:
				filterStruct->hcan->Init.Prescaler = 20;
				break;

			case CAN_500kbps:
				filterStruct->hcan->Init.Prescaler = 10;
				break;

			default:
				return HAL_OK;
		}

		//Reinitialize CAN bus
		HAL_CAN_Init(filterStruct->hcan);
	}

	//Set configuration filter into buffer
	filterStruct->filters[0] = CAN_CONFIG_FILTER;

	//Sets CAN bus configuration and custom filters
	for(uint8_t i = 0; i < CAN_NBR_OF_FILTERS; i++)
		if(filterStruct->filters[i] != 0)
			Filter_Config(filterStruct->hcan, filterStruct->filters[i], i);

	//Create timers for CAN data and set them as timed out
	for(uint8_t i = 0; i < filterStruct->nbrOfDataChannels; i++)
	{
		filterStruct->dataChannels[i]->timer = osTimerNew(Rx_Data_Timeout, osTimerOnce, filterStruct->dataChannels[i], NULL);
		Rx_Data_Timeout((void*) filterStruct->dataChannels[i]);
	}

	//Initialize receive callbacks
	HAL_CAN_ActivateNotification(filterStruct->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Starts CAN bus communication and leaves the function
	return HAL_CAN_Start(filterStruct->hcan);
}
/*END FUNCTIONS*/

/*BEGIN THREADS*/

void PDM_CAN_Thread_Receive_Data(void* threadStruct)
{
	//Struct containing CAN peripheral handle, CAN data and Config Queue handles, and thread semaphore handle
	PDM_CanRxMsg_Thread_Struct* thrdStr = (PDM_CanRxMsg_Thread_Struct*) threadStruct;

	uint8_t rxFlag;
	uint8_t rxData[8];
	CAN_RxHeaderTypeDef rxHeader;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		osSemaphoreAcquire(thrdStr->semaphoreHandle, osWaitForever);	//Wait until there is a new CAN frame received

		//Get CAN message data and process if there were no errors
		if(HAL_CAN_GetRxMessage(thrdStr->canConfig->hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
		{
			rxFlag = Rx_Data_Prepare(&rxHeader, rxData, thrdStr->canConfig->dataChannels, thrdStr->canConfig->nbrOfDataChannels);	//Prepares CAN data to Queue buffer format

			if((rxFlag & 0x01) == CAN_CONFIG_RECEIVED)	//Check type of received data
			{
				osSemaphoreRelease(thrdStr->cfgSemaphoreHandle);	//Unblocks Configuration processing thread
			}

			else if((rxFlag & 0x02) == CAN_DATA_RECEIVED)
				osSemaphoreRelease(thrdStr->outSemaphoreHandle);	//Unblocks Output processing thread
		}

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

//Times CAN bus messaging according to selected frequency
//void* threadStruct - struct containing CAN handle, CAN Mutex ID and frequency Queue ID
void PDM_CAN_Thread_Transmit_Data(void* threadStruct)
{
	//Struct containing CAN peripheral, CAN Mutex and frequency Queue handles
	PDM_CanTxMsg_Thread_Struct* thrdStr = (PDM_CanTxMsg_Thread_Struct*) threadStruct;

	//Thread execution timing
	Data_Freq frequency;	//Frequency of execution
	uint32_t periodTicks;	//Period in RTOS ticks
	uint32_t delayTick;		//Kernel timestamp for when the thread should wake up

	//Maximum time to wait for CAN Mutex reception
	uint32_t timeout = pdMS_TO_TICKS(CAN_THREAD_TIMEOUT);	//Converts timeout in milliseconds to RTOS ticks

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Receive frequency to transmit message, if failed, exits and delete the thread
	if(osMessageQueueGet(thrdStr->freqQueueHandle, (Data_Freq*) &frequency, NULL, osWaitForever) != osOK)
		osThreadExit();

	__BUFFER_TO_FREQ(frequency, periodTicks);	//Converts frequency (Data_Freq) to period (milliseconds)
	periodTicks = pdMS_TO_TICKS(periodTicks);	//Converts period in milliseconds to RTOS ticks

	delayTick = osKernelGetTickCount();	//Get kernel current timestamp

	//Infinite loop for data transmission
	for(;;)
	{
		if(osMutexAcquire(thrdStr->canMutexHandle, timeout) == osOK)	//Wait for Mutex acquisition
		{
			if(thrdStr->txMsgType == CAN_Msg_Channels)
				Tx_Channel(thrdStr->hcan, frequency, thrdStr->dataFreqBuffer, thrdStr->dataBuffer, thrdStr->dataIdBuffer);	//Sends channels via CAN bus
			else
				Tx_Fixed(thrdStr->hcan, thrdStr->txMsgType, thrdStr->dataBuffer);	//Sends fixed data via CAN bus

			osMutexRelease(thrdStr->canMutexHandle);			//Release Mutex
		}

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

		delayTick += periodTicks;	//Add wake up period to timestamp after waking up
		osDelayUntil(delayTick);	//Put thread to sleep until messaging period is fully elapsed
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*END THREADS*/

/*BEGIN STATIC FUNCTIONS*/

//Initializes CAN bus filter for its respective PWM output
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//PWM_Control_Struct *pwm_struct - control struct for PWM output
//uint8_t pwm_out_number - number of PWM output
//Returns HAL_CAN_ConfigFilter status
static HAL_StatusTypeDef Filter_Config(CAN_HandleTypeDef* hcan, uint32_t filter, uint8_t filterNbr)
{
	CAN_FilterTypeDef canFilterConfig;

	//Sets CAN filter configuration
	canFilterConfig.FilterBank = filterNbr;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = filter >> 16;
	canFilterConfig.FilterIdLow = filter & 0xffff;
	canFilterConfig.FilterMaskIdHigh = CAN_DATA_FILTER_MASK >> 16;
	canFilterConfig.FilterMaskIdLow = CAN_DATA_FILTER_MASK & 0xffff;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;

	//Loads CAN filter configuration into filter bank
	return HAL_CAN_ConfigFilter(hcan, &canFilterConfig);
}

//Start data transmission for data with specific frequency
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//Data_Freq dataFreq - data transmission frequency
//Returns HAL_CAN_AddTxMessage status
static HAL_StatusTypeDef Tx_Channel(CAN_HandleTypeDef* hcan, Data_Freq dataFreq, Data_Freq* dataFreqBuffer, uint16_t* dataBuffer, uint16_t* dataIdBuffer)
{
	uint8_t txData[8];
	uint32_t txMailbox;
	CAN_TxHeaderTypeDef txHeader;
	HAL_StatusTypeDef retVal = HAL_OK;

	//Prepares transmission header
	txHeader.DLC = 0;
	txHeader.ExtId = CAN_ID_CHANNEL;
	txHeader.IDE = CAN_ID_EXT;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		//Place data and ID inside transmission buffer if the data has the same frequency as selected
		if(dataFreqBuffer[i] == dataFreq)
		{
			txData[txHeader.DLC]	  = dataIdBuffer[i] >> 8;
			txData[txHeader.DLC + 1] |= dataIdBuffer[i] & 0xFF;
			txData[txHeader.DLC + 2]  = dataBuffer[i] >> 8;
			txData[txHeader.DLC + 3] |= dataBuffer[i] & 0xFF;

			txHeader.DLC += 4;
		}

		//Sends transmission buffer if it's full
		if(txHeader.DLC == 8)
		{
			retVal = HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);

			//Wait Transmission finish
			for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);

			txHeader.DLC = 0;
		}
	}

	//If there is only one data channel not sent, send it alone
	if(txHeader.DLC == 4)
	{
		retVal = HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return retVal;
}

static HAL_StatusTypeDef Tx_Fixed(CAN_HandleTypeDef* hcan, PDM_CAN_TxMsgType dataMsg, uint16_t* dataBuffer)
{
	uint8_t txData[8];
	uint32_t txMailbox;
	CAN_TxHeaderTypeDef txHeader;
	HAL_StatusTypeDef retVal = HAL_OK;

	//Prepares transmission header
	txHeader.DLC = 0;
	txHeader.IDE = CAN_ID_EXT;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	switch(dataMsg)
	{
		case CAN_Msg_Curr1:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_CURR1;

			txData[0] = dataBuffer[Data_Curr1] >> 8;
			txData[1] = dataBuffer[Data_Curr1] & 0xff;
			txData[2] = dataBuffer[Data_Curr2] >> 8;
			txData[3] = dataBuffer[Data_Curr2] & 0xff;
			txData[4] = dataBuffer[Data_Curr3] >> 8;
			txData[5] = dataBuffer[Data_Curr3] & 0xff;
			txData[6] = dataBuffer[Data_Curr4] >> 8;
			txData[7] = dataBuffer[Data_Curr4] & 0xff;
			break;

		case CAN_Msg_Curr2:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_CURR2;

			txData[0] = dataBuffer[Data_Curr5] >> 8;
			txData[1] = dataBuffer[Data_Curr5] & 0xff;
			txData[2] = dataBuffer[Data_Curr6] >> 8;
			txData[3] = dataBuffer[Data_Curr6] & 0xff;
			txData[4] = dataBuffer[Data_Curr7] >> 8;
			txData[5] = dataBuffer[Data_Curr7] & 0xff;
			txData[6] = dataBuffer[Data_Curr8] >> 8;
			txData[7] = dataBuffer[Data_Curr8] & 0xff;
			break;

		case CAN_Msg_Curr3:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_CURR3;

			txData[0] = dataBuffer[Data_Curr9] >> 8;
			txData[1] = dataBuffer[Data_Curr9] & 0xff;
			txData[2] = dataBuffer[Data_Curr10] >> 8;
			txData[3] = dataBuffer[Data_Curr10] & 0xff;
			txData[4] = dataBuffer[Data_Curr11] >> 8;
			txData[5] = dataBuffer[Data_Curr11] & 0xff;
			txData[6] = dataBuffer[Data_Curr12] >> 8;
			txData[7] = dataBuffer[Data_Curr12] & 0xff;
			break;

		case CAN_Msg_Curr4:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_CURR4;

			txData[0] = dataBuffer[Data_Curr13] >> 8;
			txData[1] = dataBuffer[Data_Curr13] & 0xff;
			txData[2] = dataBuffer[Data_Curr14] >> 8;
			txData[3] = dataBuffer[Data_Curr14] & 0xff;
			txData[4] = dataBuffer[Data_Curr15] >> 8;
			txData[5] = dataBuffer[Data_Curr15] & 0xff;
			txData[6] = dataBuffer[Data_Curr16] >> 8;
			txData[7] = dataBuffer[Data_Curr16] & 0xff;
			break;

		case CAN_Msg_Temp1:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_TEMP1;

			txData[0] = dataBuffer[Data_Temp1] >> 8;
			txData[1] = dataBuffer[Data_Temp1] & 0xff;
			txData[2] = dataBuffer[Data_Temp2] >> 8;
			txData[3] = dataBuffer[Data_Temp2] & 0xff;
			txData[4] = dataBuffer[Data_Temp3] >> 8;
			txData[5] = dataBuffer[Data_Temp3] & 0xff;
			txData[6] = dataBuffer[Data_Temp4] >> 8;
			txData[7] = dataBuffer[Data_Temp4] & 0xff;
			break;

		case CAN_Msg_Temp2:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_TEMP2;

			txData[0] = dataBuffer[Data_Temp5] >> 8;
			txData[1] = dataBuffer[Data_Temp5] & 0xff;
			txData[2] = dataBuffer[Data_Temp6] >> 8;
			txData[3] = dataBuffer[Data_Temp6] & 0xff;
			txData[4] = dataBuffer[Data_Temp7] >> 8;
			txData[5] = dataBuffer[Data_Temp7] & 0xff;
			txData[6] = dataBuffer[Data_Temp8] >> 8;
			txData[7] = dataBuffer[Data_Temp8] & 0xff;
			break;

		case CAN_Msg_General:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_GENERAL;

			txData[0] = dataBuffer[Data_TempMCU] >> 8;
			txData[1] = dataBuffer[Data_TempMCU] & 0xff;
			txData[2] = dataBuffer[Data_Volt] >> 8;
			txData[3] = dataBuffer[Data_Volt] & 0xff;
			txData[4] = dataBuffer[Data_Output] >> 8;
			txData[5] = dataBuffer[Data_Output] & 0xff;
			txData[6] = dataBuffer[Data_Fuse] >> 8;
			txData[7] = dataBuffer[Data_Fuse] & 0xff;
			break;

		case CAN_Msg_PWM:
			txHeader.DLC = 8;
			txHeader.ExtId = CAN_ID_PWM;

			txData[0] = dataBuffer[Data_PWM1] >> 8;
			txData[1] = dataBuffer[Data_PWM1] & 0xff;
			txData[2] = dataBuffer[Data_PWM2] >> 8;
			txData[3] = dataBuffer[Data_PWM2] & 0xff;
			txData[4] = dataBuffer[Data_PWM3] >> 8;
			txData[5] = dataBuffer[Data_PWM3] & 0xff;
			txData[6] = dataBuffer[Data_PWM4] >> 8;
			txData[7] = dataBuffer[Data_PWM4] & 0xff;
			break;

		default:
			return retVal;
	}

	if(txHeader.DLC != 0)
	{
		retVal = HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return retVal;
}

//Prepares data to be sent to output process thread
static uint8_t Rx_Data_Prepare(CAN_RxHeaderTypeDef* rxHeader, uint8_t* rxData, PDM_CAN_Data_Struct* dataStruct, uint8_t dataNbr)
{
	uint8_t data[3];
	uint32_t filter = rxHeader->IDE;	//Auxiliary variable containing filter info
	uint8_t retVal = 0;	//Indicates if data was received via CAN bus

	//Places frame ID into filter variable
	if(rxHeader->IDE == CAN_ID_EXT)
		filter |= rxHeader->ExtId << 3;

	else
		filter |= rxHeader->StdId << 3;

	for(uint8_t i = 0; i < dataNbr; i++)
	{
		if(dataStruct[i]->dataFilter == filter)
		{
			//Get data from CAN frame according to frame organization
			switch(dataStruct[i]->type)
			{
				//For fixed position data in specific frame
				//Will continue "for loop" if cast type is invalid
				case CAN_Fixed:

					if((dataStruct[i]->cast == Cast_Uint8) || (dataStruct[i]->cast == Cast_Int8))
						data[0] = rxData[dataStruct[i]->offset];

					if((dataStruct[i]->cast == Cast_Uint16) || (dataStruct[i]->cast == Cast_Int16))
					{
						data[0] = rxData[dataStruct[i]->offset];
						data[1] = rxData[dataStruct[i]->offset + 1];
					}

					else
						continue;

					break;

				//For data identification inside the frame's data field
				//Will continue "for loop" if frame length is invalid or data isn't available
				case CAN_Channel:

					if((rxHeader->DLC != 4) && (rxHeader->DLC != 8))
					{
						if(dataStruct[i]->channel == ((rxData[1] << 8) | rxData[0]))
						{
							data[0] = rxData[2];
							data[1] = rxData[3];
						}

						else if((dataStruct[i]->channel == ((rxData[4] << 8) | rxData[5])) && (rxHeader->DLC == 8))
						{
							data[0] = rxData[6];
							data[1] = rxData[7];
						}

						else
							continue;
					}

					else
						continue;

					break;

				default:
					continue;
			}

			//Cast data read from CAN bus to accommodate correct negative and positive values
			//Use data mask in case of unsigned data and byte swap in case of 16 bit data
			switch(dataStruct[i]->cast)
			{
				case Cast_Uint8:

					dataStruct[i]->data = (int32_t) (data[0] & dataStruct[i]->mask);

					break;

				case Cast_Int8:

					dataStruct[i]->data = (int32_t) ((int8_t) data[0]);

					break;

				case Cast_Uint16:
					if(dataStruct[i]->alignment == Alignment_Swap)
					{
						data[2] = data[0];
						data[0] = data[1];
						data[1] = data[2];
					}

					dataStruct[i]->data = (int32_t) ((uint16_t) ((data[0] << 8) | data[1]) & dataStruct[i]->mask);

					break;

				case Cast_Int16:
					if(dataStruct[i]->alignment == Alignment_Swap)
					{
						data[2] = data[0];
						data[0] = data[1];
						data[1] = data[2];
					}

					dataStruct[i]->data = (int32_t) ((int16_t) ((data[0] << 8) | data[1]));

					break;

				default:
					continue;
			}

			//Reset timer if data was received and timer was created correctly
			//Set return flag to indicate received value
			if(osTimerStart(dataStruct[i]->timer, pdMS_TO_TICKS(dataStruct[i]->timeout)) == osOK)
			{
				dataStruct[i]->timeoutFlag = CAN_Data_Refresh;
				retVal |= CAN_DATA_RECEIVED;
			}

			//Reset value and set flag to timeout if timer cannot be started
			else
				Rx_Data_Timeout((void*) &dataStruct[i]);
		}
	}

	return retVal;
}

static void Rx_Data_Timeout(void* dataStruct)
{
	PDM_CAN_Data_Struct* dataStr = (PDM_CAN_Data_Struct*) dataStruct;

	//Change data value if timeout happens
	if(dataStr->keep == Data_Reset)
	{
		//Substitutes value based on which cast type is configured
		//Will keep current value if there is no valid cast type
		switch(dataStr->cast)
		{
			case Cast_Uint8:
				dataStr->data = (int32_t) ((uint8_t) (dataStr->defaultVal & 0xff));
				break;

			case Cast_Int8:
				dataStr->data = (int32_t) ((int8_t) (dataStr->defaultVal & 0xff));
				break;

			case Cast_Uint16:
				dataStr->data = (int32_t) dataStr->defaultVal;
				break;

			case Cast_Int16:
				dataStr->data = (int32_t) ((int16_t) dataStr->defaultVal);
				break;

			default:
				break;
		}
	}

	//Set flag indicating timeout
	dataStr->timeoutFlag = CAN_Data_Timeout;

	return;
}

/*END STATIC FUNCTIONS*/
