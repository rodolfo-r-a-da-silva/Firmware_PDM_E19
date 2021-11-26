/*
 * pdm_readings.c
 *
 *  Created on: Nov 20, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

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
	HAL_TIM_Base_Stop_IT(&htim7);

	switch(*Data_read)
	{
	case Data_Read_Current0:
		*Data_read = Data_Read_Current0;
		Accumulator_Delay = READING_DELAY_CURRENT1;

		for(uint8_t i = 0; i < 8; i++)
		{
			Data_Buffer[i * 2] = __PDM_CONVERT_CURRENT(ADC_BUFFER[i]);

			if(Data_Buffer[i * 2] > Current_Thresholds[i * 2])
				Driver_Overcurrent_Flag |= (1 << (i * 2));

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[i * 2] = 1;
			else
				Data_Verify_Buffer[i * 2] = 0;
		}
		break;

	case Data_Read_Current1:
		*Data_read = Data_Read_Temperature;
		Accumulator_Delay = READING_DELAY_TEMPERATURE;

		for(uint8_t i = 0; i < 8; i++)
		{
			Data_Buffer[(i * 2) + 1] = __PDM_CONVERT_CURRENT(ADC_BUFFER[i]);

			if(Data_Buffer[(i * 2) + 1] > Current_Thresholds[(i * 2) + 1])
				Driver_Overcurrent_Flag |= (1 << ((i * 2) + 1));

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[(i * 2) + 1] = 1;
			else
				Data_Verify_Buffer[(i * 2) + 1] = 0;
		}
		break;

	case Data_Read_Temperature:
		*Data_read = Data_Read_Voltage;
		Accumulator_Delay = READING_DELAY_VOLTAGE;

		for(uint8_t i = 0; i < 8; i++)
		{
			Data_Buffer[16 + i] = __PDM_CONVERT_TEMPERATURE(ADC_BUFFER[i], ADC_BUFFER[8]);

			if(ADC_BUFFER[i] < 4000)
				Data_Verify_Buffer[16 + i] = 1;
			else
				Data_Verify_Buffer[16 + i] = 0;
		}
		break;

	case Data_Read_Voltage:
		*Data_read = Data_Read_Current0;
		Accumulator_Delay = READING_DELAY_CURRENT0;

		for(uint8_t i = 0; i < 8; i++)
		{
			if((ADC_BUFFER[i] < 10) || (ADC_BUFFER[i] > 4000))
				Data_Verify_Buffer[24] = 0;
			else
			{
				Data_Buffer[24] = __PDM_CONVERT_VOLTAGE(ADC_BUFFER[i], ADC_BUFFER[8]);
				Data_Verify_Buffer[24] = 1;
			}
		}

		if(Data_Verify_Buffer[24] == 0)
			Data_Buffer[0] = __PDM_CONVERT_VOLTAGE(ADC_BUFFER[0], ADC_BUFFER[8]);
		break;
	}

	Data_Buffer[25] = __PDM_CONVERT_MCU_TEMPERATURE(ADC_BUFFER[9]);

	PDM_Next_Data_Conversion(*Data_read);

	HAL_TIM_Base_Start_IT(&htim7);
}
