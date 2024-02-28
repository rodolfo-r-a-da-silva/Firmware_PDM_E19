/*
 * pdm_can.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static HAL_StatusTypeDef Filter_Config(CAN_HandleTypeDef* hcan, uint32_t filter, uint8_t filterNbr);
static uint32_t Process_Rx_Id(CAN_RxHeaderTypeDef* rxHeader);
static uint8_t Channel_CAN_Process(PDM_Channel_CAN_Struct* chnStruct, uint8_t* data, uint8_t id, uint8_t length);

/*BEGIN FUNCTIONS*/

//Initializes CAN bus communication
//PDM_CAN_Config* fltr_str - struct containing CAN handle, baud rate and filters
//Returns HAL_CAN_Start status
HAL_StatusTypeDef PDM_CAN_Init(PDM_CAN_Config_Struct* filterStruct)
{
	//Deinitialize CAN bus for configuration
	HAL_CAN_DeInit(filterStruct->hcan);

	//Sets CAN prescaler to match selected baud rate
	//If CAN bus is configured as disabled, leaves the function without initialization
	switch(filterStruct->baudRate)
	{
		case CAN_125kbps:
			filterStruct->hcan->Init.Prescaler = CAN_PRESCALER_125K;
			break;

		case CAN_250kbps:
			filterStruct->hcan->Init.Prescaler = CAN_PRESCALER_250K;
			break;

		case CAN_500kbps:
			filterStruct->hcan->Init.Prescaler = CAN_PRESCALER_500K;
			break;

		case CAN_1000kbps:
			filterStruct->hcan->Init.Prescaler = CAN_PRESCALER_1000K;
			break;

		default:
			return HAL_OK;
	}

	//Reinitialize CAN bus
	HAL_CAN_Init(filterStruct->hcan);

	//Set configuration filter into buffer
	filterStruct->filters[0] = CAN_CONFIG_FILTER;

	//Sets CAN bus configuration and custom filters
	for(uint8_t i = 0; i < CAN_NBR_OF_FILTERS; i++)
		if(filterStruct->filters[i] != 0)
			Filter_Config(filterStruct->hcan, filterStruct->filters[i], i);

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

	uint8_t rxData[8];
	uint8_t length;
	uint32_t id;
	CAN_RxHeaderTypeDef rxHeader;
	CAN_HandleTypeDef* hcan;

	uint8_t useFlag;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{
		//Wait until there is a new CAN frame received
		if(osMessageQueueGet(thrdStr->queueHandle, (void*) &hcan, 0, osWaitForever) == osOK)
		{
			//Get CAN message data and process if there were no errors
			if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
			{
				//Get and format frame ID
				id = Process_Rx_Id(&rxHeader);

				//Get frame length in bytes
				length = rxHeader.DLC;

				//Process new configuration
				if(id == CAN_CONFIG_FILTER);

				//Process channel data
				else
				{
					useFlag = Channel_CAN_Process(thrdStr->chnStruct, rxData, id, length);

					//
					if((useFlag & PROCESS_FUNCTION) == PROCESS_FUNCTION)
						osSemaphoreRelease(thrdStr->procSemHandle);

					if((useFlag & PROCESS_PWM) == PROCESS_PWM)
						osMessageQueuePut(thrdStr->outQueueHandle, (void*) &useFlag, 0, 0);
				}
			}
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

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Infinite loop for data transmission
	for(;;)
	{
#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
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

//Extracts frame ID
//CAN_RxHeaderTypeDef* rxHeader - Pointer to frame information
static uint32_t Process_Rx_Id(CAN_RxHeaderTypeDef* rxHeader)
{
	uint32_t id;

	if(rxHeader->IDE == CAN_ID_STD)
		id = (rxHeader->StdId << 3) | rxHeader->IDE;

	else
		id = (rxHeader->ExtId << 3) | rxHeader->IDE;

	return id;
}

//Try to convert data received from CAN bus
//chnStruct - Pointer to array of data Channel structs
//data - Pointer to array containing received data from CAN bus
//id - Identification of CAN data frame
//length - Length in bytes of received data
static uint8_t Channel_CAN_Process(PDM_Channel_CAN_Struct* chnStruct, uint8_t* data, uint8_t id, uint8_t length)
{
	uint8_t retVal = PROCESS_NONE;
	uint8_t aux[2];

	for(uint8_t i = CHN_CAN_OFFSET; i < CHN_CAN_FINISH; i++)
	{
		//Continue "for loop" if CAN bus frame id doesn't match channel filter or channel isn't used for any conditions
		if((*chnStruct[i].dataFilter != id)
				|| (chnStruct[i].inUse == IN_USE_NONE))
			continue;

		//Get data from Queue according to data source
		//Return if channel source isn't from CAN bus
		switch(chnStruct[i].source)
		{
			//For fixed position data in specific frame
			//Will continue "for loop" if cast type is invalid
			case Data_CAN_Fixed:
			//{
				if((chnStruct[i].cast == Cast_Uint8) || (chnStruct[i].cast == Cast_Int8))
					aux[0] = data[chnStruct[i].position];

				else
				{
					aux[0] = data[chnStruct[i].position];
					aux[1] = data[chnStruct[i].position + 1];
				}
				break;
			//}

			//For data identification inside the frame's data field
			//Will continue "for loop" if frame length is invalid or data isn't available
			case Data_CAN_Channel:
			//{
				if((chnStruct[i].position == ((data[1] << 8) | data[0])) && (length >= 4))
				{
					aux[0] = data[2];
					aux[1] = data[3];
				}

				else if((chnStruct[i].position == ((data[4] << 8) | data[5])) && (length == 8))
				{
					aux[0] = data[6];
					aux[1] = data[7];
				}

				else
					continue;

				break;
			//}

			default:
				continue;
		}

		//Reset timer if data was received
		//Set return flag to indicate received value
		if(osTimerStart(chnStruct[i].timerHandle, chnStruct[i].timeout) == osOK)
			chnStruct[i].timeoutFlag = CAN_Data_Refresh;

		//Reset value and set flag to timeout if timer cannot be started
		else
		{
			PDM_Data_Timeout_Callback((void*) &chnStruct[i]);
			continue;
		}

		//Change previous state of data
		chnStruct[i].data[Data_Previous] = chnStruct[i].data[Data_Current];

		//Cast received data to correct format
		PDM_Data_Cast(&chnStruct[i], aux);

		//Set retVal if value has changed and CAN data is used in Functions or PWM
		if(chnStruct[i].data[Data_Previous] != chnStruct[i].data[Data_Current])
			retVal |= chnStruct[i].inUse;
	}

	return retVal;
}

/*END STATIC FUNCTIONS*/
