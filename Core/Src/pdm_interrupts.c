/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM7)
	{
		if(Accumulator_Delay > 0)
			Accumulator_Delay--;

		return;
	}
	else if(htim->Instance == TIM6)
	{
		Accumulator_Output_Check++;
		Accumulator_Msg_10Hz++;
		Accumulator_Msg_25Hz++;
		Accumulator_Msg_50Hz++;
		Accumulator_Msg_100Hz++;

		if(EEPROM_Write_Status == 1)
			Accumulator_EEPROM_Write++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	PDM_Input_Process();

	PDM_Output_Process();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_Rx_Message, Can_Rx_Data) == HAL_OK)
	{
		PDM_CAN_Process_Rx_Data();

		HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
	}
}
