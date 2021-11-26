/*
 * pdm_can.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"



HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t CAN_BaudRate)
{
	HAL_CAN_DeInit(hcan);

	if(CAN_BaudRate == CAN_125KBPS)
	{
		hcan->Init.Prescaler = 40;
	}

	else if(CAN_BaudRate == CAN_250KBPS)
	{
		hcan->Init.Prescaler = 20;
	}

	else if(CAN_BaudRate == CAN_500KBPS)
	{
		hcan->Init.Prescaler = 10;
	}

	else if(CAN_BaudRate == CAN_1000KBPS)
	{
		hcan->Init.Prescaler = 5;
	}

	HAL_CAN_Init(hcan);

	if(PWM_Pins[0].PWM_CAN_Enable == OUTPUT_PWM_CAN_ENABLE
		|| PWM_Pins[1].PWM_CAN_Enable == OUTPUT_PWM_CAN_ENABLE
		|| PWM_Pins[2].PWM_CAN_Enable == OUTPUT_PWM_CAN_ENABLE
		|| PWM_Pins[3].PWM_CAN_Enable == OUTPUT_PWM_CAN_ENABLE)
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef PDM_PWM_CAN_Filter_Config(CAN_HandleTypeDef *hcan, PWM_Control_Struct *pwm_struct, uint8_t can_filter_bank)
{
	if(pwm_struct->PWM_CAN_Enable == OUTPUT_PWM_CAN_DISABLE)
		return HAL_OK;

	CAN_FilterTypeDef CAN_Filter_Config;

	CAN_Filter_Config.FilterBank = can_filter_bank;
	CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDLIST;
	CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = pwm_struct->Command_Var_CAN_ID[0] << 13;
	CAN_Filter_Config.FilterIdLow = (pwm_struct->Command_Var_CAN_ID[0] << 3) & 0xFFF8;
	CAN_Filter_Config.FilterMaskIdHigh = pwm_struct->Command_Var_CAN_ID[1] << 13;
	CAN_Filter_Config.FilterMaskIdLow = (pwm_struct->Command_Var_CAN_ID[1] << 3) & 0xFFF8;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_Filter_Config.FilterActivation = ENABLE;

	return HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Config);
}

HAL_StatusTypeDef PDM_CAN_Transmit_Data(CAN_HandleTypeDef* hcan, uint16_t data_freq)
{
	HAL_StatusTypeDef Ret_Val = HAL_OK;

	uint8_t data_index = 0;
	uint16_t data_buffer[NBR_OF_DATA_CHANNELS * 2];

	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		if(Data_Freq_Buffer[i] == data_freq)
		{
			data_buffer[2 * data_index] = (Data_ID_Buffer[i] << 1) | Data_Verify_Buffer[i];
			data_buffer[(2 * data_index) + 1] = Data_Buffer[i];
			data_index++;
		}
	}

	switch(data_freq)
	{
	case DATA_FREQ_100HZ:
		Can_Tx_Message.ExtId = 0x1E35C003;
		break;
	case DATA_FREQ_50HZ:
		Can_Tx_Message.ExtId = 0x1E35C004;
		break;
	case DATA_FREQ_25HZ:
		Can_Tx_Message.ExtId = 0x1E35C005;
		break;
	case DATA_FREQ_10HZ:
		Can_Tx_Message.ExtId = 0x1E35C006;
		break;
	default:
		return HAL_ERROR;
	}

	Can_Tx_Message.IDE = CAN_ID_EXT;
	Can_Tx_Message.RTR = CAN_RTR_DATA;
	Can_Tx_Message.TransmitGlobalTime = DISABLE;

	for(uint8_t i = 0; i <= (2 * data_index); i += 2)
	{
		Can_Tx_Message.DLC = 4;

		Can_Tx_Data[0] = data_buffer[i] >> 8;
		Can_Tx_Data[1] = data_buffer[i] & 0xFF;
		Can_Tx_Data[2] = data_buffer[i + 1] >> 8;
		Can_Tx_Data[3] = data_buffer[i + 1] & 0xFF;

		if((data_index - i) >= 2)
		{
			Can_Tx_Message.DLC = 8;

			i += 2;

			Can_Tx_Data[4] = data_buffer[i] >> 8;
			Can_Tx_Data[5] = data_buffer[i] & 0xFF;
			Can_Tx_Data[6] = data_buffer[i + 1] >> 8;
			Can_Tx_Data[7] = data_buffer[i + 1] & 0xFF;
		}

		Ret_Val = HAL_CAN_AddTxMessage(hcan, &Can_Tx_Message, Can_Tx_Data, &pTxMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return Ret_Val;
}

void PDM_CAN_Process_Rx_Data()
{
	uint8_t return_flag = 0;
	uint32_t Rx_ID = 0;

	if(Can_Rx_Message.IDE == CAN_ID_STD)
	{
		Rx_ID = Can_Rx_Message.StdId;
	}else{
		Rx_ID = Can_Rx_Message.ExtId;
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		if(Rx_ID == PWM_Pins[i].Command_Var_CAN_ID[0])
		{
			PWM_Pins[i].Command_Var[0]  = (Can_Rx_Data[PWM_Pins[i].Command_Var_Position[0]]) << 8;
			PWM_Pins[i].Command_Var[0] |= (Can_Rx_Data[PWM_Pins[i].Command_Var_Position[0] + 1]) & 0xFF;

			return_flag = 1;
		}
		else if(Rx_ID == PWM_Pins[i].Command_Var_CAN_ID[1])
		{
			PWM_Pins[i].Command_Var[1]  = (Can_Rx_Data[PWM_Pins[i].Command_Var_Position[1]]) << 8;
			PWM_Pins[i].Command_Var[1] |= (Can_Rx_Data[PWM_Pins[i].Command_Var_Position[1] + 1]) & 0xFF;

			return_flag = 1;
		}

		if(return_flag == 1)
			return;
	}

	return;
}
