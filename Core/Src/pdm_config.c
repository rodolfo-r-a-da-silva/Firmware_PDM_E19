/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "usbd_cdc_if.h"

//Use to load pin, current limit and basic pwm control into a buffer
//uint8_t *data_buffer - buffer to receive configuration
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_BUFFER_SIZE)
static void PDM_Load_Config_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	//Checks if data buffer has the correct size
	if(Size != EEPROM_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	//Loads output configurations (enabled inputs, levels, fuse current thresholds and fuse timeouts)
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

	//Loads CAN bus baud rate
	CAN_Baud_Rate = data_buffer[data_index++];

	//Loads PWM outputs general configuration (PWM CAN and PWM enabled)
	PWM_Pin_Status = data_buffer[data_index++];

	//Loads PWM outputs specific configuration (frequency, duty cycle presets and enabled inputs,
	//command variable position inside CAN packet, CAN packet ID, 3D map lengths and limits)
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

	//Loads each data channel transmission frequency
	for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		Data_Freq_Buffer[i] = data_buffer[data_index++];
	}

	return;
}

//Use to load pwm 3D map into a buffer
//uint8_t *data_buffer - buffer to receive 3D map
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_MAP_BUFFER_SIZE)
static void PDM_Load_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	//Checks if data buffer has the correct size
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
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_BUFFER_SIZE)
static void PDM_Write_Config_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	//Checks if data buffer has the correct size
	if(Size != EEPROM_BUFFER_SIZE)
		return;

	uint16_t data_index = 0;

	//Writes output configurations (enabled inputs, levels, fuse current thresholds and fuse timeouts)
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

	//Writes CAN bus baud rate
	data_buffer[data_index++] = CAN_Baud_Rate;

	//Writes PWM outputs general configuration (PWM CAN and PWM enabled)
	data_buffer[data_index++] = PWM_Pin_Status;

	//Writes PWM outputs specific configuration (frequency, duty cycle presets and enabled inputs,
	//command variable position inside CAN packet, CAN packet ID, 3D map lengths and limits)
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
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_MAP_BUFFER_SIZE)
static void PDM_Write_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
{
	//Checks if data buffer has the correct size
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
	uint8_t data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE];

	//Reads general configuration from EEPROM
	AT24Cxx_Read(hi2c, 0x0000, data_buffer, EEPROM_BUFFER_SIZE);

	//Loads configuration into global variables
	PDM_Load_Config_Buffer(data_buffer, EEPROM_BUFFER_SIZE);

	//Reads PWM 3D maps from EEPROM
	AT24Cxx_Read(hi2c, EEPROM_BUFFER_SIZE, &data_buffer[EEPROM_BUFFER_SIZE], EEPROM_MAP_BUFFER_SIZE);

	//Load maps into global PWM structs
	PDM_Load_Map_Buffer(&data_buffer[EEPROM_BUFFER_SIZE], EEPROM_MAP_BUFFER_SIZE);

	//Weak function for data overwriting during initialization
	PDM_Hard_Code_Config();

	//Initializes CAN ID buffer
	__PDM_ID_BUFFER_INIT();

	//Initializes each PWM able output
	for(uint8_t i = 0; i < 4; i++)
	{
		PDM_PWM_Init(hcan, &PWM_Pins[i], i);
	}

	//Checks input pin levels
	PDM_Input_Process();

	//Initializes CAN bus
	PDM_CAN_Init(hcan, CAN_Baud_Rate);

	//Sets outputs based on input levels
	PDM_Output_Process();

	return;
}

//Process received USB data (placed inside CDC_Receive_FS)
//Will setup pins, current limits and basic pwm controls if Data[0] == 0x0F and Data[1] == 0x0F
//Will setup pwm 3D maps if Data[0] == 0xF0 and Data[1] == 0xF0
//uint8_t *Data - buffer received via USB port
//uint16_t Size - size of the buffer must be 6 bytes bigger than the information received (2 bytes of command and 4 bytes of CRC)
//Returns HAL_I2C_Mem_Write_DMA status
HAL_StatusTypeDef PDM_USB_Receive(uint8_t *Data, uint16_t Size)
{
	uint16_t data_address = 0;
	uint32_t crc[2];

	//Checks first 2 bytes from USB buffer for command
	if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)))
	{
		//Checks received buffer size
		if((Size + 6) != EEPROM_BUFFER_SIZE)
			return HAL_ERROR;

		//Select EEPROM address to write general configurations
		data_address = 0x0000;
	}
	else if((Data[0] == (USB_COMMAND_WRITE_MAP >> 8)) && (Data[1] == (USB_COMMAND_WRITE_MAP & 0xFF)))
	{
		//Checks received buffer size
		if((Size + 6) != EEPROM_MAP_BUFFER_SIZE)
			return HAL_ERROR;

		//Select EEPROM address to write PWM 3D maps
		data_address = EEPROM_MAP_BUFFER_SIZE;
	}
	else if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)) && (Size == 2))
	{
		//Enables USB data transmission
		USB_Connected_Flag = 1;
		return HAL_OK;
	}
	else
		//Unknown command
		return HAL_ERROR;

	//Calculate data buffer checksum
	crc[0] = HAL_CRC_Calculate(&hcrc, (uint32_t*) Data, (Size - 4) / (sizeof(uint32_t)));

	crc[1]  = Data[Size - 4] << 24;
	crc[1] |= Data[Size - 3] << 16;
	crc[1] |= Data[Size - 2] << 8;
	crc[1] |= Data[Size - 1] & 0xFF;

	//Compare received buffer calculated and received
	if(crc[0] != crc[1])
		return HAL_ERROR;

	if((Data[0] == (USB_COMMAND_WRITE_CONFIG >> 8)) && (Data[1] == (USB_COMMAND_WRITE_CONFIG & 0xFF)))
		//Load received configuration into global variables
		PDM_Load_Config_Buffer(&Data[2], Size - 4);
	else
		//Load received PWM 3D maps into PWM structs
		PDM_Load_Map_Buffer(&Data[2], Size - 4);

	//Writes received configuration into EEPROM
	return AT24Cxx_Write_DMA(&hi2c1, data_address, &Data[2], Size - 4);
}

//Sends pins, current limits, basic pwm controls and pwm 3D maps via USB port
//uint8_t *command - request for loaded configuration
//uint16_t Size - check if the command only has 2 bytes
void PDM_USB_Transmit_Config(uint8_t *command, uint16_t Size)
{
	//Check size and first 2 bytes from USB buffer for command
	if((command[0] != (USB_COMMAND_READ_CONFIG >> 8)) && (command[1] != (USB_COMMAND_READ_CONFIG & 0xFF)) && (Size != 6))
		return;

	uint8_t data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE + 12];
	uint32_t crc[2];

	crc[1] = USB_COMMAND_READ_CONFIG;
	crc[0] = HAL_CRC_Calculate(&hcrc, &crc[1], 1);

	//Return if calculated CRC is different from received CRC
	if(crc[0] != crc[1])
		return;

	//Load general configuration from global variables
	PDM_Write_Config_Buffer(&data_buffer[2], EEPROM_BUFFER_SIZE);

	//Load command into transmission buffer
	data_buffer[0] = USB_COMMAND_WRITE_CONFIG >> 8;
	data_buffer[1] = USB_COMMAND_WRITE_CONFIG & 0xFF;

	//Calculate buffer CRC
	crc[0] = HAL_CRC_Calculate(&hcrc, (uint32_t*) &data_buffer[0], (EEPROM_BUFFER_SIZE + 2) / (sizeof(uint32_t)));

	//Load CRC into transmission buffer
	data_buffer[EEPROM_BUFFER_SIZE + 2]  = crc[0] >> 24;
	data_buffer[EEPROM_BUFFER_SIZE + 3] |= crc[0] >> 16;
	data_buffer[EEPROM_BUFFER_SIZE + 4] |= crc[0] >> 8;
	data_buffer[EEPROM_BUFFER_SIZE + 5] |= crc[0] & 0xFF;

	//Transmit general configuration buffer via USB
	CDC_Transmit_FS(data_buffer, EEPROM_BUFFER_SIZE + 6);

	//Load PWM 3D maps from PWM structs
	PDM_Write_Map_Buffer(&data_buffer[EEPROM_BUFFER_SIZE + 8], EEPROM_MAP_BUFFER_SIZE);

	//Load command into transmission buffer
	data_buffer[EEPROM_BUFFER_SIZE + 6] = USB_COMMAND_WRITE_MAP >> 8;
	data_buffer[EEPROM_BUFFER_SIZE + 7] = USB_COMMAND_WRITE_MAP & 0xFF;

	//Calculate buffer CRC
	crc[0] = HAL_CRC_Calculate(&hcrc, (uint32_t*) &data_buffer[EEPROM_BUFFER_SIZE + 6], (EEPROM_MAP_BUFFER_SIZE + 2) / (sizeof(uint32_t)));

	//Load CRC into transmission buffer
	data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE + 8]  = crc[0] >> 24;
	data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE + 9] |= crc[0] >> 16;
	data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE + 10] |= crc[0] >> 8;
	data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE + 11] |= crc[0] & 0xFF;

	//Transmit PWM 3D map buffer via USB
	CDC_Transmit_FS(&data_buffer[EEPROM_BUFFER_SIZE + 6], EEPROM_MAP_BUFFER_SIZE + 6);

	return;
}

//Sends data channels via USB
void PDM_USB_Transmit_Data()
{
	uint8_t data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 6];
	uint32_t crc = 0;

	//Load data channels inside transmission buffer
	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		data_buffer[(i * 2) + 2]  = Data_Buffer[i] >> 8;
		data_buffer[(i * 2) + 3] |= Data_Buffer[i] & 0xFF;
	}

	//Load command into transmission buffer
	data_buffer[0] = USB_COMMAND_READ_DATA >> 8;
	data_buffer[1] = USB_COMMAND_READ_DATA & 0xFF;

	//Calculate buffer CRC
	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) data_buffer, (NBR_OF_DATA_CHANNELS + 2) / sizeof(uint32_t));

	//Load CRC into transmission buffer
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 2]  = crc >> 24;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 3] |= crc >> 16;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 4] |= crc >> 8;
	data_buffer[(NBR_OF_DATA_CHANNELS * 2) + 5] |= crc & 0xFF;

	//Transmit data buffer via USB
	CDC_Transmit_FS(data_buffer, EEPROM_MAP_BUFFER_SIZE + 6);

	return;
}

//Use for configuration without or with partial EEPROM data
__weak void PDM_Hard_Code_Config(){}
