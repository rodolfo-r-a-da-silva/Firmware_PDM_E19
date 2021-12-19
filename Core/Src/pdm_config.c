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

	//Loads each data channel transmission frequency
	for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		Data_Freq_Buffer[i] = data_buffer[data_index++];
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

	//Loads PWM 3D maps
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

//Use to load pwm 3D map into a buffer
//uint8_t *data_buffer - buffer to receive 3D map
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_MAP_BUFFER_SIZE)
//static void PDM_Load_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
//{
//	//Checks if data buffer has the correct size
//	if(Size != EEPROM_BUFFER_SIZE)
//		return;
//
//	uint16_t data_index = 0;
//
//	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
//	{
//		for(uint8_t j = 0; j < PWM_TABLE_MAX_SIZE; j++)
//		{
//			for(uint8_t k = 0; k < PWM_TABLE_MAX_SIZE; k++)
//			{
//				PWM_Pins[i].Duty_Cycle_Map[j][k]  = data_buffer[data_index++] << 8;
//				PWM_Pins[i].Duty_Cycle_Map[j][k] |= data_buffer[data_index++] & 0xFF;
//			}
//		}
//	}
//
//	return;
//}

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

	//Writes each data channel transmission frequency
	for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		data_buffer[data_index++] = Data_Freq_Buffer[i];
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

	//Writes PWM 3D maps
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

//Use to load pwm 3D map into their respective variables
//uint8_t *data_buffer - buffer to send 3D map
//uint16_t Size - size of the buffer, must match the size of the configuration (EEPROM_MAP_BUFFER_SIZE)
//static void PDM_Write_Map_Buffer(uint8_t *data_buffer, uint16_t Size)
//{
//	//Checks if data buffer has the correct size
//	if(Size != EEPROM_BUFFER_SIZE)
//		return;
//
//	uint16_t data_index = 0;
//
//	for(uint16_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
//	{
//		for(uint8_t j = 0; j < PWM_TABLE_MAX_SIZE; j++)
//		{
//			for(uint8_t k = 0; k < PWM_TABLE_MAX_SIZE; k++)
//			{
//				data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Map[j][k] >> 8;
//				data_buffer[data_index++] = PWM_Pins[i].Duty_Cycle_Map[j][k] & 0xFF;
//			}
//		}
//	}
//
//	return;
//}

//Sets up pins, current limits, pwm controls and pwm 3D maps with received data from USB port
//uint8_t *Data - buffer received via USB port
//uint16_t Size - size of the buffer must be 5 bytes bigger than the information received (1 byte of command and 4 bytes of CRC)
static void PDM_USB_Receive_Config(uint8_t *Data, uint16_t Size)
{
	uint32_t crc[2];

	crc[0] = HAL_CRC_Calculate(&hcrc, (uint32_t*) &Data[5], (EEPROM_BUFFER_SIZE / sizeof(uint32_t)));

	crc[1]  = Data[1] << 24;
	crc[1] |= Data[2] << 16;
	crc[1] |= Data[3] << 8;
	crc[1] |= Data[4];

	if(crc[0] != crc[1])
		return;

	PDM_Write_Config_Buffer(&Data[5], EEPROM_BUFFER_SIZE);

	AT24Cxx_Write_DMA(&hi2c1, 0x0000, &Data[5], EEPROM_BUFFER_SIZE);

	return;
}

//Sends pins, current limits, pwm controls and pwm 3D maps via USB port
static void PDM_USB_Transmit_Config()
{
	uint8_t *data_buffer = malloc((EEPROM_BUFFER_SIZE + 5) * sizeof(uint8_t));
	uint32_t crc = 0;

	PDM_Load_Config_Buffer(&data_buffer[5], EEPROM_BUFFER_SIZE);

	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &data_buffer[5], (EEPROM_BUFFER_SIZE / sizeof(uint32_t)));

	data_buffer[1] = crc >> 24;
	data_buffer[2] = crc >> 16;
	data_buffer[3] = crc >> 8;
	data_buffer[4] = crc & 0xFF;

	CDC_Transmit_FS(data_buffer, (EEPROM_BUFFER_SIZE + 5));

	free(data_buffer);
	data_buffer = NULL;

	return;
}

//Initialize PDM
//Loads from EEPROM
//Initializes PWM
void PDM_Init(CAN_HandleTypeDef *hcan, I2C_HandleTypeDef *hi2c)
{
//	uint8_t data_buffer[EEPROM_BUFFER_SIZE + EEPROM_MAP_BUFFER_SIZE];
	uint8_t *data_buffer = malloc(EEPROM_BUFFER_SIZE * sizeof(uint8_t));

	//Reads general configuration from EEPROM
	AT24Cxx_Read(hi2c, 0x0000, data_buffer, EEPROM_BUFFER_SIZE);

	//Loads configuration into global variables
	PDM_Load_Config_Buffer(data_buffer, EEPROM_BUFFER_SIZE);

	//Reads PWM 3D maps from EEPROM
//	AT24Cxx_Read(hi2c, EEPROM_BUFFER_SIZE, &data_buffer[EEPROM_BUFFER_SIZE], EEPROM_MAP_BUFFER_SIZE);

	//Load maps into global PWM structs
//	PDM_Load_Map_Buffer(&data_buffer[EEPROM_BUFFER_SIZE], EEPROM_MAP_BUFFER_SIZE);

	free(data_buffer);
	data_buffer = NULL;

	//Weak function for data overwriting during initialization
	PDM_Hard_Code_Config();

	//Initializes CAN ID buffer
	__PDM_ID_BUFFER_INIT();

	//Initializes each PWM able output
	for(uint8_t i = 0; i < 4; i++)
		PDM_PWM_Init(hcan, &PWM_Pins[i], i);

	//Checks input pin levels
	PDM_Input_Process();

	//Initializes CAN bus
	PDM_CAN_Init(hcan, CAN_Baud_Rate);

	//Sets outputs based on input levels
	PDM_Output_Process();

	//Initializates timers and ADC conversion
	HAL_ADC_Start_DMA(&hadc1, &ADC_BUFFER[5], 5);
	HAL_ADC_Start_DMA(&hadc2, &ADC_BUFFER[0], 5);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	return;
}

void PDM_USB_Process(uint8_t *Data, uint16_t Size)
{
	if((Data[0] == USB_COMMAND_READ_CONFIG) && (Size == 1))
		PDM_USB_Transmit_Config();

	else if((Data[0] == USB_COMMAND_WRITE_CONFIG) && (Size == (EEPROM_BUFFER_SIZE + 5)))
		PDM_USB_Receive_Config(Data, Size);

	else if((Data[0] == USB_COMMAND_CONNECT) && (Size == 1))
		USB_Connected_Flag = 1;

	else if((Data[0] == USB_COMMAND_DISCONNECT) && (Size == 1))
		USB_Connected_Flag = 0;
}

//Sends data channels via USB
void PDM_USB_Transmit_Data()
{
	uint8_t *data_buffer = malloc((NBR_OF_DATA_CHANNELS * 2) * sizeof(uint8_t));
	uint32_t crc = 0;

	//Load data channels inside transmission buffer
	for(uint8_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)
	{
		data_buffer[(i * 2) + 5]  = Data_Buffer[i] >> 8;
		data_buffer[(i * 2) + 6] |= Data_Buffer[i] & 0xFF;
	}

	//Load command into transmission buffer
	data_buffer[0] = USB_COMMAND_READ_DATA & 0xFF;

	//Calculate buffer CRC
	crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &data_buffer[5], (NBR_OF_DATA_CHANNELS / sizeof(uint32_t)));

	//Load CRC into transmission buffer
	data_buffer[1]  = crc >> 24;
	data_buffer[2] |= crc >> 16;
	data_buffer[3] |= crc >> 8;
	data_buffer[4] |= crc;

	//Transmit data buffer via USB
	CDC_Transmit_FS(data_buffer, EEPROM_BUFFER_SIZE + 5);

	free(data_buffer);
	data_buffer = NULL;

	return;
}


//Use for configuration without or with partial EEPROM data
__weak void PDM_Hard_Code_Config(){}
