/*
 * pdm_timers.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "stdlib.h"

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
		HAL_TIM_Base_Stop_IT(htim);
		flagReading[1] = Data_Read_Ready;
	}

	else if(htim->Instance == TIM7)
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

//	else if((htim->Instance == TIM3) && (pwmOutStruct[0].softStart == SoftStart_Enabled) && (pwmOutStruct[0].softStartStruct != NULL))
//	{
//		if(pwmOutStruct[0].softStartStruct->dutyCycles > 0) pwmOutStruct[0].softStartStruct->dutyCycles--;
//
//		if(pwmOutStruct[0].softStartStruct->dutyCycles == 0)
//		{
//			free(&pwmOutStruct[0].softStartStruct->dutyCycleBuffer);
//			HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
//			HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
//			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, pwmOutStruct[0].dutyCycle);
//		}
//	}
//
//	else if((htim->Instance == TIM8) && (pwmOutStruct[1].softStart == SoftStart_Enabled) && (pwmOutStruct[1].softStartStruct != NULL))
//	{
//		if(pwmOutStruct[1].softStartStruct->dutyCycles > 0) pwmOutStruct[1].softStartStruct->dutyCycles--;
//
//		if(pwmOutStruct[1].softStartStruct->dutyCycles == 0)
//		{
//			free(&pwmOutStruct[1].softStartStruct->dutyCycleBuffer);
//			HAL_TIMEx_PWMN_Stop_DMA(htim, TIM_CHANNEL_2);
//			HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
//			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pwmOutStruct[1].dutyCycle);
//		}
//	}
//
//	else if((htim->Instance == TIM2) && (pwmOutStruct[2].softStart == SoftStart_Enabled) && (pwmOutStruct[2].softStartStruct != NULL))
//	{
//		if(pwmOutStruct[2].softStartStruct->dutyCycles > 0) pwmOutStruct[2].softStartStruct->dutyCycles--;
//
//		if(pwmOutStruct[2].softStartStruct->dutyCycles == 0)
//		{
//			free(&pwmOutStruct[2].softStartStruct->dutyCycleBuffer);
//			HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
//			HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
//			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, pwmOutStruct[2].dutyCycle);
//		}
//	}
//
//	else if((htim->Instance == TIM1) && (pwmOutStruct[3].softStart == SoftStart_Enabled) && (pwmOutStruct[3].softStartStruct != NULL))
//	{
//		if(pwmOutStruct[3].softStartStruct->dutyCycles > 0) pwmOutStruct[3].softStartStruct->dutyCycles--;
//
//		if(pwmOutStruct[3].softStartStruct->dutyCycles == 0)
//		{
//			free(&pwmOutStruct[3].softStartStruct->dutyCycleBuffer);
//			HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
//			HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
//			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, pwmOutStruct[3].dutyCycle);
//		}
//	}

	return;
}
