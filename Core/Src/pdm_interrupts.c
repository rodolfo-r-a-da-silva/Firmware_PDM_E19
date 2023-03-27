/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	osSemaphoreRelease(canRxSemaphoreHandle);	//Release semaphore to PDM_CAN_Thread_Receive_Data Thread

	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	PDM_Data_Queue_Struct data = {.source = Interrupt_Gpio};

	//Send interrupt source to Readings Thread
	osMessageQueuePut(processQueueHandle, (void*) &data, 0, 0);

	return;
}
