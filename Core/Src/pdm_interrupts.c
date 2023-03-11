/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	osSemaphoreRelease(canRxSemaphore);	//Release semaphore to PDM_CAN_Thread_Receive_Data Thread

	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	osSemaphoreRelease(outputSemaphore);	//Release semaphore to PDM_Output_Thread

	return;
}
