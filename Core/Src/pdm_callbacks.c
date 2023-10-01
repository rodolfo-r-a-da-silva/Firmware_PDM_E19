/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

/*BEGIN PERIPHERAL CALLBACK FUNCTIONS*/

//Callback for received CAN messages
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	osSemaphoreRelease(canRxSemaphoreHandle);	//Release semaphore to PDM_CAN_Thread_Receive_Data Thread

	return;
}

//Callback for input pins
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	PDM_Data_Queue_Struct data = {.source = Interrupt_Gpio};

	//Send interrupt source to Readings Thread
	osMessageQueuePut(processQueueHandle, (void*) &data, 0, 0);

	return;
}

//Callback for soft start last duty cycle value update
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t source = PROCESS_PWM_SS;

	//Send process source to Output Thread
	osMessageQueuePut(outQueueHandle, &source, 0, 0);

	return;
}

/*END PERIPHERAL CALLBACK FUNCTIONS*/

/*BEGIN RTOS CALLBACK FUNCTIONS*/

//Function that indicates data reception timeout and places predefined value if configured
void PDM_Data_Timeout_Callback(void* dataStruct)
{
	PDM_Data_Channel_Struct* dataStr = (PDM_Data_Channel_Struct*) dataStruct;

	//Change data value if timeout happens
	if(dataStr->keep == Data_Reset)
		PDM_Data_Cast(dataStr, (uint8_t*) &dataStr->defaultVal);

	//Set flag indicating timeout
	dataStr->timeoutFlag = CAN_Data_Timeout;

	return;
}

//Function to process new results after delay periods
void PDM_Function_Delay_Callback(void* callbackStruct)
{
	PDM_Function_Struct* cllbckStr = (PDM_Function_Struct*) callbackStruct;

	//Struct to send interrupt source to processing Thread
	PDM_Data_Queue_Struct flagStruct;
	flagStruct.source = Interrupt_Function;

	//Auxiliary variable to select correct delay in Blink Functions
	uint8_t aux;

	//Choose process according to Function type
	if((cllbckStr->type == Function_Blink) || (cllbckStr->type == Function_Pulse))
	{
		cllbckStr->result[Result_Current] = !cllbckStr->result[Result_Current];

		//Restart Timer if processing Blink Function
		if(cllbckStr->type == Function_Blink)
		{
			aux = cllbckStr->result[Result_Current];
			osTimerStart(cllbckStr->funcTimer, cllbckStr->funcDelay[aux]);
		}

		else
			cllbckStr->result[Result_Next] = Result_False;
	}

	else
	{
		//Change current result state
		cllbckStr->result[Result_Previous] = cllbckStr->result[Result_Current];
		cllbckStr->result[Result_Current] = cllbckStr->result[Result_Next];
	}

	if(cllbckStr->inUse != IN_USE_NONE)
		osMessageQueuePut(cllbckStr->processQueue, &flagStruct, 0, 0);

	return;
}

void PDM_Fuse_Timer_Callback(void* callbackStruct)
{
	PDM_Output_Fuse_Struct* cllbckStr = (PDM_Output_Fuse_Struct*) callbackStruct;

	//Variable to send signal to output Thread
	uint8_t flag = PROCESS_FUSE;

	//Struct to send interrupt source to process Thread
	PDM_Data_Queue_Struct flagStruct;
	flagStruct.source = Interrupt_Fuse;

	//Enter if fuse status is set to wait
	if(cllbckStr->status == Fuse_Wait)
		cllbckStr->status = Fuse_Open;	//Set fuse to open

	//Enter if fuse status is set to open
	else if(cllbckStr->status == Fuse_Open)
	{
		cllbckStr->retryCount++;	//Increment number of retry counts
		cllbckStr->status = Fuse_Wait;	//Set fuse to close at waiting state
	}

	osMessageQueuePut(cllbckStr->outputQueue, &flag, 0, 0);	//Send flag to output Thread for it to open or close the output

	//Send flag to process Thread for it to update Functions if it's being used
	if(cllbckStr->funcStruct->inUse != IN_USE_NONE)
		osMessageQueuePut(cllbckStr->processQueue, &flagStruct, 0, 0);

	return;
}

/*END RTOS CALLBACK FUNCTIONS*/