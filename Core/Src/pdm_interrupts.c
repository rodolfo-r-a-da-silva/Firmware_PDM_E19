/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Message, CAN_Rx_Data) == HAL_OK)
	{
		PDM_CAN_Process_Rx_Data();

		HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
	}

	return;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	PDM_Input_Process();
//
//	PDM_Output_Process();
//
//	return;
//}

//void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	return;
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM7)
	{
		Accumulator_Msg_10Hz++;
		Accumulator_Msg_25Hz++;
		Accumulator_Msg_50Hz++;
		Accumulator_Msg_80Hz++,
		Accumulator_Msg_100Hz++;

		Accumulator_Delay++;
		Accumulator_Output_Check++;
		Accumulator_Temp_Read++;
		Accumulator_Volt_Read++;

		Accumulator_USB_Data++;
	}

	return;
}
