/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

uint32_t teste = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxMessage, canRxData) == HAL_OK)
	{
		PDM_CAN_Process_Rx_Data();

		HAL_GPIO_TogglePin(CAN_RX_LED_GPIO_Port, CAN_RX_LED_Pin);
	}

	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	PDM_Input_Process();

	PDM_Output_Process();

	return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM6)
	{
		teste++;
		HAL_TIM_Base_Stop_IT(htim);
		flagReading[1] = Data_Read_Ready;
	}

	if(htim->Instance == TIM7)
	{
		accMsg10Hz++;
		accMsg25Hz++;
		accMsg50Hz++;
		accMsg80Hz++,
		accMsg100Hz++;

		accUsbData++;

		accOutputFuse[0]++;
		accOutputFuse[1]++;
		accOutputFuse[2]++;
		accOutputFuse[3]++;
		accOutputFuse[4]++;
		accOutputFuse[5]++;
		accOutputFuse[6]++;
		accOutputFuse[7]++;
		accOutputFuse[8]++;
		accOutputFuse[9]++;
		accOutputFuse[10]++;
		accOutputFuse[11]++;
		accOutputFuse[12]++;
		accOutputFuse[13]++;
		accOutputFuse[14]++;
		accOutputFuse[15]++;
	}

	return;
}
