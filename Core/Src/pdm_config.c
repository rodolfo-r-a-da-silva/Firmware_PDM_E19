/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "usbd_cdc_if.h"

//Use to load pin, current limit and basic pwm control into a buffer
//uint8_t *data_buffer - buffer to receive config
//uint16_t Size - size of the buffer, must match the size of the config (EEPROM_BUFFER_SIZE)
static void PDM_Load_Config_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	if(Size != EEPROM_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	for(uint16_t i= 0; i < NBR_OF_OUTPUTS; i++)
	{
		for(uint8_t j = 0; j < 2 ; j++)
		{
			Output_Pin[i].Enabled_Inputs[j]  = data_buffer[data_index++] << 8;
			Output_Pin[i].Enabled_Inputs[j] |= data_buffer[data_index++] & 0xFF;

			Output_Pin[i].Input_Levels[j]	 = data_buffer[data_index++] << 8;
			Output_Pin[i].Input_Levels[j]	|= data_buffer[data_index++] & 0xFF;
		}
		Output_Pin[i].Current_Thresholds   = data_buffer[data_index++] << 8;
		Output_Pin[i].Current_Thresholds  |= data_buffer[data_index++] & 0xFF;

		Output_Pin[i].Timeout_Output_Fuse  = data_buffer[data_index++] << 8;
		Output_Pin[i].Timeout_Output_Fuse |= data_buffer[data_index++] & 0xFF;
	}

	CAN_Baud_Rate = data_buffer[data_index++];

	PWM_Pin_Status = data_buffer[data_index++];

	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		PWM_Pins[i].PWM_Frequency = data_buffer[data_index++];

		for(uint8_t j = 0; j < 4; j++)
		{
			PWM_Pins[i].Input_DC_Preset_Enable[j]  = data_buffer[data_index++] << 8;
			PWM_Pins[i].Input_DC_Preset_Enable[j] |= data_buffer[data_index++] & 0xFF;

			PWM_Pins[i].Input_DC_Preset[j] 		   = data_buffer[data_index++] << 8;
			PWM_Pins[i].Input_DC_Preset[j] 		  |= data_buffer[data_index++] & 0xFF;
		}

		for(uint8_t j = 0; j < 2; j++)
		{
			PWM_Pins[i].Duty_Cycle_Preset[j]	 = data_buffer[data_index++] << 8;
			PWM_Pins[i].Duty_Cycle_Preset[j]	|= data_buffer[data_index++] & 0xFF;

			PWM_Pins[i].Command_Var_Position[j]  = data_buffer[data_index++];

			PWM_Pins[i].Command_Var_CAN_ID[j]	 = data_buffer[data_index++] << 24;
			PWM_Pins[i].Command_Var_CAN_ID[j]	|= data_buffer[data_index++] << 16;
			PWM_Pins[i].Command_Var_CAN_ID[j]	|= data_buffer[data_index++] << 8;
			PWM_Pins[i].Command_Var_CAN_ID[j]	|= data_buffer[data_index++] & 0xFF;

			PWM_Pins[i].Map_Lengths[j]			 = data_buffer[data_index++] & 0xFF;

			PWM_Pins[i].Command_Var_Lim[j][0]	 = data_buffer[data_index++] << 8;
			PWM_Pins[i].Command_Var_Lim[j][1]	|= data_buffer[data_index++] & 0xFF;
		}
	}

	for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		Data_Freq_Buffer[i] = data_buffer[data_index++];
	}

	return;
}

//Use to load pwm 3D map into a buffer
//uint8_t *data_buffer - buffer to receive 3D map
//uint16_t Size - size of the buffer, must match the size of the config (EEPROM_MAP_BUFFER_SIZE)
static void PDM_Load_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	if(Size != EEPROM_MAP_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		for(uint8_t j = 0; j < PWM_TABLE_MAX_SIZE; j++)
		{
			for(uint8_t k = 0; k < PWM_TABLE_MAX_SIZE; k++)
			{
				PWM_Pins[i].Duty_Cycle_Map[j][k]  = data_buffer[data_index++] << 8;
				PWM_Pins[i].Duty_Cycle_Map[j][k] |= data_buffer[data_index++] & 0xFF;
			}
		}
	}

	return;
}

//Use to write pin, current limit and basic pwm control into their respective variables
//uint8_t *data_buffer - buffer to send config
//uint16_t Size - size of the buffer, must match the size of the config (EEPROM_BUFFER_SIZE)
static void PDM_Write_Config_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	if(Size != EEPROM_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	for(uint16_t i= 0; i < NBR_OF_OUTPUTS; i++)
	{
		for(uint8_t j = 0; j < 2 ; j++)
		{
			data_buffer[data_index++] = Output_Pin[i].Enabled_Inputs[j] >> 8;
			data_buffer[data_index++] = Output_Pin[i].Enabled_Inputs[j] & 0xFF;

			data_buffer[data_index++] = Output_Pin[i].Input_Levels[j] >> 8;
			data_buffer[data_index++] = Output_Pin[i].Input_Levels[j] & 0xFF;
		}
		data_buffer[data_index++] = Output_Pin[i].Current_Thresholds >> 8;
		data_buffer[data_index++] = Output_Pin[i].Current_Thresholds & 0xFF;

		data_buffer[data_index++] = Output_Pin[i].Timeout_Output_Fuse >> 8;
		data_buffer[data_index++] = Output_Pin[i].Timeout_Output_Fuse & 0xFF;
	}

	data_buffer[data_index++] = CAN_Baud_Rate;

	data_buffer[data_index++] = PWM_Pin_Status;

	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		data_buffer[data_index++] = PWM_Pins[i].PWM_Frequency;

		for(uint8_t j = 0; j < 4; j++)
		{
			data_buffer[data_index++] = PWM_Pins[i].Input_DC_Preset_Enable[j] >> 8;
			data_buffer[data_index++] = PWM_Pins[i].Input_DC_Preset_Enable[j] & 0xFF;

			data_buffer[data_index++] = PWM_Pins[i].Input_DC_Preset[j] >> 8;
			data_buffer[data_index++] = PWM_Pins[i].Input_DC_Preset[j] & 0xFF;
		}

		for(uint8_t j = 0; j < 2; j++)
		{
			data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Preset[j] >> 8;
			data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Preset[j] & 0xFF;

			data_buffer[data_index++] = PWM_Pins[i].Command_Var_Position[j];

			data_buffer[data_index++] = PWM_Pins[i].Command_Var_CAN_ID[j] >> 24;
			data_buffer[data_index++] = PWM_Pins[i].Command_Var_CAN_ID[j] >> 16;
			data_buffer[data_index++] = PWM_Pins[i].Command_Var_CAN_ID[j] >> 8;
			data_buffer[data_index++] = PWM_Pins[i].Command_Var_CAN_ID[j] & 0xFF;

			data_buffer[data_index++] = PWM_Pins[i].Map_Lengths[j];

			data_buffer[data_index++] = PWM_Pins[i].Command_Var_Lim[j][0] >> 8;
			data_buffer[data_index++] = PWM_Pins[i].Command_Var_Lim[j][0] & 0xFF;
		}
	}

	return;
}

//Use to load pwm 3D map into their respective variables
//uint8_t *data_buffer - buffer to send 3D map
//uint16_t Size - size of the buffer, must match the size of the config (EEPROM_MAP_BUFFER_SIZE)
static void PDM_Write_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	if(Size != EEPROM_MAP_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		for(uint8_t j = 0; j < PWM_TABLE_MAX_SIZE; j++)
		{
			for(uint8_t k = 0; k < PWM_TABLE_MAX_SIZE; k++)
			{
				data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Map[j][k] >> 8;
				data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Map[j][k] & 0xFF;
			}
		}
	}

	return;
}

//Initialize PDM
//Loads from EEPROM
//Initializes PWM
void PDM_Init(CAN_HandleTypeDef *hcan, I2C_HandleTypeDef *hi2c)
{
	uint8_t data_buffer0[EEPROM_BUFFER_SIZE];
	uint8_t data_buffer1[EEPROM_MAP_BUFFER_SIZE];

	AT24Cxx_Read(hi2c, 0x0000, data_buffer0, EEPROM_BUFFER_SIZE);

	PDM_Load_Config_Buffer(data_buffer0, EEPROM_BUFFER_SIZE);

	AT24Cxx_Read(hi2c, EEPROM_BUFFER_SIZE, data_buffer1, EEPROM_MAP_BUFFER_SIZE);

	PDM_Load_Map_Buffer(data_buffer1, EEPROM_MAP_BUFFER_SIZE);

	PDM_Hard_Code_Config();

	__PDM_ID_BUFFER_INIT();

	for(uint8_t i = 0; i < 4; i++)
	{
		PDM_PWM_Init(hcan, &PWM_Pins[i], i);
	}

	PDM_Input_Process();

	PDM_Output_Process();

	return;
}

//Proccess received USB data (placed inside CDC_Receive_FS)
//Will setup pins, current limits and basic pwm controls if Data[0] == 0x0F and Data[1] == 0x0F
//Will setup pwm 3D maps if Data[0] == 0xF0 and Data[1] == 0xF0
//uint8_t *Data - buffer received via USB port
//uint16_t Size - size of the buffer must be 6 bytes bigger than the information received (2 bytes of command and 4 bytes of CRC)
//Returns HAL_I2C_Mem_Write_DMA status
HAL_StatusTypeDef PDM_USB_Receive(uint8_t *Data, uint16_t Size)
{
	uint8_t data_buffer[Size];
	uint16_t data_address = 0;
	uint32_t crc[2];

	if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)))
	{
		if((Size + 6) != EEPROM_BUFFER_SIZE)
			return HAL_ERROR;

		data_address = 0x0000;

		PDM_Write_Config_Buffer(&data_buffer[2], EEPROM_BUFFER_SIZE);
	}
	else if((Data[0] == (USB_COMMAND_WRITE_MAP >> 8)) && (Data[1] == (USB_COMMAND_WRITE_MAP & 0xFF)))
	{
		if((Size + 6) != EEPROM_MAP_BUFFER_SIZE)
			return HAL_ERROR;

		data_address = EEPROM_MAP_BUFFER_SIZE;

		PDM_Write_Map_Buffer(&data_buffer[2], EEPROM_MAP_BUFFER_SIZE);
	}
	else if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)) && (Size == 2))
	{
		USB_Connected_Flag = 1;
		return HAL_OK;
	}
	else
		return HAL_ERROR;

	crc[0] = HAL_CRC_Calculate(&hcrc, (uint32_t*) data_buffer, (EEPROM_BUFFER_SIZE + 2) / (sizeof(uint32_t)));

	crc[1]  = Data[Size - 4] << 24;
	crc[1] |= Data[Size - 3] << 16;
	crc[1] |= Data[Size - 2] << 8;
	crc[1] |= Data[Size - 1] & 0xFF;

	if(crc[0] != crc[1])
		return HAL_ERROR;

	if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)))
		PDM_Load_Config_Buffer(&Data[2], Size - 4);
	else
		PDM_Load_Map_Buffer(&Data[2], Size - 4);

	return AT24Cxx_Write_DMA(&hi2c1, data_address, &Data[2], Size - 4);
}

//Sends pins, current limits, basic pwm controls and pwm 3D maps via USB port
//uint8_t *command - request for loaded config ()
//uint16_t Size - check if the command only has 2 bytes
void PDM_USB_Transmit_Config(uint8_t *command, uint16_t Size)
{
	if((Size != 2) && (command[0] != (USB_COMMAND_READ_CONFIG >> 8)) && (command[1] != (USB_COMMAND_READ_CONFIG & 0xFF)))
		return;

	uint8_t data_buffer0[EEPROM_BUFFER_SIZE + 6];
	uint8_t data_buffer1[EEPROM_MAP_BUFFER_SIZE + 6];
	uint32_t crc = 0;

	PDM_Load_Config_Buffer(&data_buffer0[2], EEPROM_BUFFER_SIZE);

	data_buffer0[0] = USB_COMMAND_WRITE_CONFIG >> 8;
	data_buffer0[1] = USB_COMMAND_WRITE_CONFIG & 0xFF;

	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data_buffer0, (EEPROM_BUFFER_SIZE + 2) / (sizeof(uint32_t)));

	data_buffer0[EEPROM_BUFFER_SIZE + 2]  = crc >> 24;
	data_buffer0[EEPROM_BUFFER_SIZE + 3] |= crc >> 16;
	data_buffer0[EEPROM_BUFFER_SIZE + 4] |= crc >> 8;
	data_buffer0[EEPROM_BUFFER_SIZE + 5] |= crc & 0xFF;

	CDC_Transmit_FS(data_buffer0, EEPROM_BUFFER_SIZE + 6);

	PDM_Load_Map_Buffer(&data_buffer1[2], EEPROM_MAP_BUFFER_SIZE);

	data_buffer1[0] = USB_COMMAND_WRITE_MAP >> 8;
	data_buffer1[1] = USB_COMMAND_WRITE_MAP & 0xFF;

	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data_buffer1, (EEPROM_MAP_BUFFER_SIZE + 2) / (sizeof(uint32_t)));

	data_buffer0[EEPROM_BUFFER_SIZE + 2]  = crc >> 24;
	data_buffer0[EEPROM_BUFFER_SIZE + 3] |= crc >> 16;
	data_buffer0[EEPROM_BUFFER_SIZE + 4] |= crc >> 8;
	data_buffer0[EEPROM_BUFFER_SIZE + 5] |= crc & 0xFF;

	CDC_Transmit_FS(data_buffer1, EEPROM_MAP_BUFFER_SIZE + 6);

	return;
}

void PDM_USB_Transmit_Data()
{
	uint8_t data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 6];
	uint32_t crc = 0;

	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		data_buffer[(i * 2) + 2]  = Data_Buffer[i] >> 8;
		data_buffer[(i * 2) + 3] |= Data_Buffer[i] & 0xFF;
	}

	data_buffer[0] = USB_COMMAND_READ_DATA >> 8;
	data_buffer[1] = USB_COMMAND_READ_DATA & 0xFF;

	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data_buffer, (NBR_OF_DATA_CHANNELS + 2) / sizeof(uint32_t));

	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 2]  = crc >> 24;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 3] |= crc >> 16;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 4] |= crc >> 8;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 5] |= crc & 0xFF;

	CDC_Transmit_FS(data_buffer, EEPROM_MAP_BUFFER_SIZE + 6);

	return;
}

//Use for configuration without or with partial EEPROM data
__weak void PDM_Hard_Code_Config(){}
