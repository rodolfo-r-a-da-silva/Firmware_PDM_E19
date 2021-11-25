/*
 * pdm_readings.c
 *
 *  Created on: Nov 20, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

uint16_t PDM_Convert_Current(uint16_t adc_current)
{
	return (adc_current * 2750 * 330) / (374 * 4095);
}

uint16_t PDM_Convert_Temperature(uint16_t adc_temperature, uint16_t gnd_vnd)
{
	return (((gnd_vnd - adc_temperature + 1838) * (180 + 374) * 330 * 682) / (374 * 4095)) + 2500;
}

uint16_t PDM_Convert_Voltage(uint16_t adc_voltage, uint16_t gnd_vnd)
{
	return ((adc_voltage - gnd_vnd) * (180 + 374) * 330) / (374 * 8 * 4095);
}

uint16_t PDM_Convert_MCU_Temperature(uint16_t adc_temperature)
{
	return (((adc_temperature - 943) * 4964) / 4095) + 2500;
}

void PDM_Next_Data_Conversion(uint8_t Next_Data)
{
	switch(Next_Data)
	{
	case Data_Read_Current0:
		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);
		break;

	case Data_Read_Current1:
		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);
		break;

	case Data_Read_Temperature:
		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);
		break;

	case Data_Read_Voltage:
		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);
		break;
	}
}

void PDM_Read_Data(uint8_t *Data_read)
{
	HAL_TIM_Base_Stop_IT(&htim4);

	switch(*Data_read)
	{
	case Data_Read_Current0:
		*Data_read = Data_Read_Current0;

		for(uint8_t i = 0; i < 8; i++)
		{
			Converted_Data_Buffer[i * 2] = PDM_Convert_Current(ADC_BUFFER[i]);

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[i * 2] = 1;
			else
				Data_Verify_Buffer[i * 2] = 0;
		}
		break;

	case Data_Read_Current1:
		*Data_read = Data_Read_Temperature;

		for(uint8_t i = 0; i < 8; i++)
		{
			Converted_Data_Buffer[(i * 2) + 1] = PDM_Convert_Current(ADC_BUFFER[i]);

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[(i * 2) + 1] = 1;
			else
				Data_Verify_Buffer[(i * 2) + 1] = 0;
		}
		break;

	case Data_Read_Temperature:
		*Data_read = Data_Read_Voltage;

		for(uint8_t i = 0; i < 8; i++)
		{
			Converted_Data_Buffer[16 + i] = PDM_Convert_Temperature(ADC_BUFFER[i], ADC_BUFFER[8]);

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[16 + i] = 1;
			else
				Data_Verify_Buffer[16 + i] = 0;
		}
		break;

	case Data_Read_Voltage:
		*Data_read = Data_Read_Current0;

		for(uint8_t i = 0; i < 8; i++)
		{
			if((ADC_BUFFER[i] < 10) || (ADC_BUFFER[i] > 4000))
				Data_Verify_Buffer[24] = 0;
			else
			{
				Converted_Data_Buffer[24] = PDM_Convert_Voltage(ADC_BUFFER[i], ADC_BUFFER[8]);
				Data_Verify_Buffer[24] = 1;
			}
		}

		if(Data_Verify_Buffer[24] == 0)
			Converted_Data_Buffer[0] = PDM_Convert_Voltage(ADC_BUFFER[0], ADC_BUFFER[8]);
		break;
	}

	Converted_Data_Buffer[25] = PDM_Convert_MCU_Temperature(ADC_BUFFER[9]);

	PDM_Next_Data_Conversion(*Data_read);

	Accumulator_Delay = 0;

	HAL_TIM_Base_Start_IT(&htim4);
}
