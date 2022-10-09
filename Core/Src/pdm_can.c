/*
 * pdm_can.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static HAL_StatusTypeDef PDM_CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t filter_nbr, uint32_t filter_id, uint32_t filter_mask, uint32_t filter_ide);

//Initializes CAN bus communication
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//uint8_t CAN_BaudRate - Baud Rate value in enum: 	0: Disable
//													1: 125	kbps
//													2: 250	kbps
//													3: 500	kbps
//													4: 1000 kbps
//Returns HAL_CAN_Start status
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, PDM_CAN_Config* filter_struct)
{
	//Deinitialize CAN bus for new configuration
	HAL_CAN_DeInit(hcan);

	//Sets CAN prescaler to match selected baud rate
	//If CAN bus is configured as disabled, leaves the function without initialization
	switch(filter_struct->baudRate)
	{
	case CAN_125kbps:
		hcan->Init.Prescaler = 40;
		break;

	case CAN_250kbps:
		hcan->Init.Prescaler = 20;
		break;

	case CAN_500kbps:
		hcan->Init.Prescaler = 10;
		break;

	case CAN_1000kbps:
		hcan->Init.Prescaler = 5;
		break;

		default:
			return HAL_OK;
	}

	//Reinitialize CAN bus
	HAL_CAN_Init(hcan);

	PDM_CAN_Filter_Config(hcan, 0, CAN_CONFIG_FILTER, CAN_CONFIG_MASK, CAN_ID_EXT);

	//Initialize receive callbacks
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Starts CAN bus communication and leaves the function
	return HAL_CAN_Start(hcan);
}

//Start data transmission for data with specific frequency
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//uint8_t data_freq - data transmit frequency:		0: Transmission disabled
//													1: 10 Hz transmission
//													2: 25 Hz transmission
//													3: 50 Hz transmission
//													4: 80 Hz transmission
//													5: 100 Hz transmission
//Returns HAL_CAN_AddTxMessage status
HAL_StatusTypeDef PDM_CAN_Transmit_Data(CAN_HandleTypeDef* hcan, uint8_t data_freq)
{
	HAL_StatusTypeDef ret_val = HAL_OK;

	return ret_val;

	//Selects CAN transmission ID based on data transmission frequency
	switch(data_freq)
	{
	case Data_Freq_100Hz:
		canTxMessage.ExtId = 0x1E35C003;
		break;
	case Data_Freq_80Hz:
		canTxMessage.ExtId = 0x1E35C004;
		break;
	case Data_Freq_50Hz:
		canTxMessage.ExtId = 0x1E35C005;
		break;
	case Data_Freq_25Hz:
		canTxMessage.ExtId = 0x1E35C005;
		break;
	case Data_Freq_10Hz:
		canTxMessage.ExtId = 0x1E35C006;
		break;
	default:
		return HAL_ERROR;
	}

	//Prepares transmission header
	canTxMessage.DLC = 0;
	canTxMessage.IDE = CAN_ID_EXT;
	canTxMessage.RTR = CAN_RTR_DATA;
	canTxMessage.TransmitGlobalTime = DISABLE;

	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		//Place data and ID inside transmission buffer if the data has the same frequency as selected
		if(dataFreqBuffer[i] == data_freq)
		{
			canTxData[canTxMessage.DLC]		 = dataIdBuffer[i] >> 8;
			canTxData[canTxMessage.DLC + 1] |= dataIdBuffer[i] & 0xFF;
			canTxData[canTxMessage.DLC + 2]  = dataBuffer[i] >> 8;
			canTxData[canTxMessage.DLC + 3] |= dataBuffer[i] & 0xFF;

			canTxMessage.DLC += 4;
		}

		//Sends transmission buffer if it's full
		if(canTxMessage.DLC == 8)
		{
			ret_val = HAL_CAN_AddTxMessage(hcan, &canTxMessage, canTxData, &canTxMailbox);

			//Wait Transmission finish
			for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);

			canTxMessage.DLC = 0;
		}
	}

	//If there is only one data channel not sent, send it alone
	if(canTxMessage.DLC == 4)
	{
		ret_val = HAL_CAN_AddTxMessage(hcan, &canTxMessage, canTxData, &canTxMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return ret_val;
}

//Process received data to set PWM output duty cycle
void PDM_CAN_Process_Rx_Data()
{
	return;
}

//Initializes CAN bus filter for its respective PWM output
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//PWM_Control_Struct *pwm_struct - control struct for PWM output
//uint8_t pwm_out_number - number of PWM output
//Returns HAL_CAN_ConfigFilter status
static HAL_StatusTypeDef PDM_CAN_Filter_Config(CAN_HandleTypeDef* hcan, uint8_t filter_nbr, uint32_t filter_id, uint32_t filter_mask, uint32_t filter_ide)
{
	CAN_FilterTypeDef canFilterConfig;

	//Sets CAN filter configuration
	canFilterConfig.FilterBank = filter_nbr;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = filter_id >> 13;
	canFilterConfig.FilterIdLow = (filter_id << 3) | (filter_ide & 0x0004);
	canFilterConfig.FilterMaskIdHigh = filter_mask >> 13;
	canFilterConfig.FilterMaskIdLow = (filter_mask << 3) | 0x0004;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;

	//Loads CAN filter configuration into filter bank
	return HAL_CAN_ConfigFilter(hcan, &canFilterConfig);
}
