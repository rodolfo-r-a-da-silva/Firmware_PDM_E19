/*
 * pdm_can.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

//Initializes CAN bus communication
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//uint8_t CAN_BaudRate - Baud Rate value in enum: 	0: Disable
//													1: 125	kbps
//													2: 250	kbps
//													3: 500	kbps
//													4: 1000 kbps
//Returns HAL_CAN_Start status
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t CAN_BaudRate)
{
	//Deinitialize CAN bus for new configuration
	HAL_CAN_DeInit(hcan);

	//Sets CAN prescaler to match selected baud rate
	//If CAN bus is configured as disabled, leaves the function without initialization
	if(CAN_BaudRate == CAN_Disable)
	{
		return HAL_OK;
	}
	else if(CAN_BaudRate == CAN_125kbps)
	{
		hcan->Init.Prescaler = 40;
	}

	else if(CAN_BaudRate == CAN_250kbps)
	{
		hcan->Init.Prescaler = 20;
	}

	else if(CAN_BaudRate == CAN_500kbps)
	{
		hcan->Init.Prescaler = 10;
	}

	else if(CAN_BaudRate == CAN_1000kbps)
	{
		hcan->Init.Prescaler = 5;
	}

	//Reinitialize CAN bus
	HAL_CAN_Init(hcan);

	//Initialize receive callbacks if there is at least one PWM CAN enabled
	if((PWM_Pin_Status & 0xF0) != 0x00)
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Starts CAN bus communication and leaves the function
	return HAL_CAN_Start(hcan);
}

//Initializes CAN bus filter for its respective PWM output
//CAN_HandleTypeDef *hcan - CAN handler struct pointer
//PWM_Control_Struct *pwm_struct - control struct for PWM output
//uint8_t pwm_out_number - number of PWM output
//Returns HAL_CAN_ConfigFilter status
HAL_StatusTypeDef PDM_PWM_CAN_Filter_Config(CAN_HandleTypeDef *hcan, PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number)
{
	//Double check if PWM CAN is enabled for this specific output
	if(((PWM_Pin_Status >> pwm_out_number) & 0x10) != OUTPUT_PWM_CAN_ENABLE)
		return HAL_ERROR;

	CAN_FilterTypeDef CAN_Filter_Config;

	//Sets CAN filter configuration
	CAN_Filter_Config.FilterBank = pwm_out_number;
	CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDLIST;
	CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = pwm_struct->Command_Var_CAN_ID[0] >> 13;
	CAN_Filter_Config.FilterIdLow = (pwm_struct->Command_Var_CAN_ID[0] << 3) & 0xFFF8;
	CAN_Filter_Config.FilterMaskIdHigh = pwm_struct->Command_Var_CAN_ID[1] >> 13;
	CAN_Filter_Config.FilterMaskIdLow = (pwm_struct->Command_Var_CAN_ID[1] << 3) & 0xFFF8;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_Filter_Config.FilterActivation = ENABLE;

	//Loads CAN filter configuration into filter bank
	return HAL_CAN_ConfigFilter(hcan, &CAN_Filter_Config);
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

	//Selects CAN transmission ID based on data transmission frequency
	switch(data_freq)
	{
	case Data_Freq_100Hz:
		CAN_Tx_Message.ExtId = 0x1E35C003;
		break;
	case Data_Freq_80Hz:
		CAN_Tx_Message.ExtId = 0x1E35C004;
		break;
	case Data_Freq_50Hz:
		CAN_Tx_Message.ExtId = 0x1E35C005;
		break;
	case Data_Freq_25Hz:
		CAN_Tx_Message.ExtId = 0x1E35C005;
		break;
	case Data_Freq_10Hz:
		CAN_Tx_Message.ExtId = 0x1E35C006;
		break;
	default:
		return HAL_ERROR;
	}

	//Prepares transmission header
	CAN_Tx_Message.DLC = 0;
	CAN_Tx_Message.IDE = CAN_ID_EXT;
	CAN_Tx_Message.RTR = CAN_RTR_DATA;
	CAN_Tx_Message.TransmitGlobalTime = DISABLE;

	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		//Place data and ID inside transmission buffer if the data has the same frequency as selected
		if(Data_Freq_Buffer[i] == data_freq)
		{
			CAN_Tx_Data[CAN_Tx_Message.DLC]		 = Data_ID_Buffer[i] >> 8;
			CAN_Tx_Data[CAN_Tx_Message.DLC + 1] |= Data_ID_Buffer[i] & 0xFF;
			CAN_Tx_Data[CAN_Tx_Message.DLC + 2]  = Data_Buffer[i] >> 8;
			CAN_Tx_Data[CAN_Tx_Message.DLC + 3] |= Data_Buffer[i] & 0xFF;

			CAN_Tx_Message.DLC += 4;
		}

		//Sends transmission buffer if it's full
		if(CAN_Tx_Message.DLC == 8)
		{
			ret_val = HAL_CAN_AddTxMessage(hcan, &CAN_Tx_Message, CAN_Tx_Data, &pTxMailbox);

			//Wait Transmission finish
			for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);

			CAN_Tx_Message.DLC = 0;
		}
	}

	//If there is only one data channel not sent, send it alone
	if(CAN_Tx_Message.DLC == 4)
	{
		ret_val = HAL_CAN_AddTxMessage(hcan, &CAN_Tx_Message, CAN_Tx_Data, &pTxMailbox);

		//Wait Transmission finish
		for(uint8_t i = 0; HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && i < 3; i++);
	}

	return ret_val;
}

//Process received data to set PWM output duty cycle
void PDM_CAN_Process_Rx_Data()
{
	uint8_t receive_flag = 0;
	uint32_t rx_id = 0;

	if(CAN_Rx_Message.IDE == CAN_ID_STD)
	{
		rx_id = CAN_Rx_Message.StdId;
	}else{
		rx_id = CAN_Rx_Message.ExtId;
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		//Checks if received message contains data to any PWN CAN output
		if(rx_id == PWM_Pins[i].Command_Var_CAN_ID[0])
		{
			PWM_Pins[i].Command_Var[0]  = (CAN_Rx_Data[PWM_Pins[i].Command_Var_Position[0]]) << 8;
			PWM_Pins[i].Command_Var[0] |= (CAN_Rx_Data[PWM_Pins[i].Command_Var_Position[0] + 1]) & 0xFF;

			receive_flag = 1;
		}

		if(rx_id == PWM_Pins[i].Command_Var_CAN_ID[1])
		{
			PWM_Pins[i].Command_Var[1]  = (CAN_Rx_Data[PWM_Pins[i].Command_Var_Position[1]]) << 8;
			PWM_Pins[i].Command_Var[1] |= (CAN_Rx_Data[PWM_Pins[i].Command_Var_Position[1] + 1]) & 0xFF;

			receive_flag = 1;
		}

		//If the received message contains data to any PWM CAN output, set it's duty cycle
		if(receive_flag == 1)
		{
			PDM_PWM_Output_Process(&PWM_Pins[i], i);
			return;
		}
	}

	return;
}
