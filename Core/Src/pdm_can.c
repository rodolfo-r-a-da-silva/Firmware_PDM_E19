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

	return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef PDM_Can_Transmit_Data(CAN_HandleTypeDef* hcan, uint16_t data_freq)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data_index = 0;
	uint16_t data_buffer[50];

	for(uint8_t i = 0; i < 25; i++)
	{
		if(Data_Freq_Buffer[i] == data_freq)
		{
			data_buffer[2 * data_index] = (Data_ID_Buffer[i] << 1) | Data_Verify_Buffer[i];
			data_buffer[(2 * data_index) + 1] = Converted_Data_Buffer[i];
			data_index++;
		}
	}

	switch(data_freq)
	{
	case DATA_FREQ_100HZ:
		CanTxMessage.ExtId = 0x1E35C003;
		break;
	case DATA_FREQ_50HZ:
		CanTxMessage.ExtId = 0x1E35C004;
		break;
	case DATA_FREQ_25HZ:
		CanTxMessage.ExtId = 0x1E35C005;
		break;
	case DATA_FREQ_10HZ:
		CanTxMessage.ExtId = 0x1E35C006;
		break;
	}

	CanTxMessage.IDE = CAN_ID_EXT;
	CanTxMessage.RTR = CAN_RTR_DATA;
	CanTxMessage.TransmitGlobalTime = DISABLE;

	for(uint8_t i = 0; i <= (2 * data_index); i += 2)
	{
		CanTxMessage.DLC = 4;

		CanTxData[0] = data_buffer[i] >> 8;
		CanTxData[1] = data_buffer[i] & 0xff;
		CanTxData[2] = data_buffer[i + 1] >> 8;
		CanTxData[3] = data_buffer[i + 1] & 0xff;

		if((data_index - i) >= 2)
		{
			CanTxMessage.DLC = 8;

			i += 2;

			CanTxData[4] = data_buffer[i] >> 8;
			CanTxData[5] = data_buffer[i] & 0xff;
			CanTxData[6] = data_buffer[i + 1] >> 8;
			CanTxData[7] = data_buffer[i + 1] & 0xff;
		}

		HAL_CAN_AddTxMessage(hcan, &CanTxMessage, CanTxData, &pTxMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return status;
}
