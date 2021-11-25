/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM1)
	{
		Accumulator_Msg_10++;
		Accumulator_Msg_25++;
		Accumulator_Msg_50++;
		Accumulator_Msg_100++;
	}
	else if(htim->Instance == TIM4)
	{
		Accumulator_Delay++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Input_Process();

	Output_Process();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxMessage, CanRxData) == HAL_OK)
	{
		HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
	}
}
