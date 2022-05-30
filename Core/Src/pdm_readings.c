/*
 * pdm_readings.c
 *
 *  Created on: Nov 20, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"


//Reads ADC value and converts into data
//Returns HAL_TIM_Base_Start_IT status
HAL_StatusTypeDef PDM_Data_Conversion(TIM_HandleTypeDef *htim)
{
	uint8_t flag_fuse = 0;
	HAL_StatusTypeDef retVal = HAL_OK;

	flagReading[1] = Data_Read_Waiting;

	//Convert ADC value based on selected reading and sets delay for next reading
	switch(flagReading[0])
	{
	case Data_Read_Current0:

		for(uint8_t i = 0; i < 8; i++)
		{
			//Convert ADC into current
			dataBuffer[i * 2] = __PDM_CONVERT_CURRENT(adcBuffer[i]);

			if((dataBuffer[i * 2] <= outputStruct[i * 2].currentThresholds)
					|| (outputStruct[i * 2].timeoutOutputFuse == 0))
				accOutputFuse[i * 2] = 0;

			else if(accOutputFuse[i * 2] > outputStruct[i * 2].timeoutOutputFuse)
			{
				flag_fuse = 1;
				flagDriverSafety |= 1 << (i * 2);
			}

			if(adcBuffer[i] < ADC_THRESHOLD_HIGH)
				dataIdBuffer[i * 2] |= 1;
			else
				dataIdBuffer[i * 2] &= 0xFFFE;
		}

		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

		flagReading[0] = Data_Read_Current1;
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);

		break;

	case Data_Read_Current1:

		for(uint8_t i = 0; i < 8; i++)
		{
			//Convert ADC into current
			dataBuffer[(i * 2) + 1] = __PDM_CONVERT_CURRENT(adcBuffer[i]);

			if((dataBuffer[(i * 2) + 1] <= outputStruct[(i * 2) + 1].currentThresholds)
					|| (outputStruct[(i * 2) + 1].timeoutOutputFuse == 0))
				accOutputFuse[(i * 2) + 1] = 0;

			else if(accOutputFuse[(i * 2) + 1] > outputStruct[(i * 2) + 1].timeoutOutputFuse)
			{
				flag_fuse = 1;
				flagDriverSafety |= 1 << ((i * 2) + 1);
			}

			if(adcBuffer[i] < ADC_THRESHOLD_HIGH)
				dataIdBuffer[(i * 2) + 1] |= 1;
			else
				dataIdBuffer[(i * 2) + 1] &= 0xFFFE;
		}

		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

		flagReading[0] = Data_Read_Temperature;
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_TEMP);

		break;

	case Data_Read_Temperature:

		for(uint8_t i = 0; i < 8; i++)
		{
			//Convert ADC into temperature
			dataBuffer[16 + i] = __PDM_CONVERT_TEMPERATURE(adcBuffer[i], adcBuffer[8]);

			if((adcBuffer[i] < ADC_THRESHOLD_LOW) || (adcBuffer[i] > ADC_THRESHOLD_HIGH))
				dataIdBuffer[16 + i] |= 1;
			else
				dataIdBuffer[16 + i] &= 0xFFFE;
		}

		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET);

		flagReading[0] = Data_Read_Voltage;
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_VOLT);

		break;

	case Data_Read_Voltage:

		//Convert ADC into voltage if the ADC value is valid
		if((adcBuffer[0] < ADC_THRESHOLD_LOW) || (adcBuffer[0] > ADC_THRESHOLD_HIGH))
			dataIdBuffer[24] &= 0xFFFE;
		else
		{
			dataBuffer[24] = __PDM_CONVERT_VOLTAGE(adcBuffer[0], adcBuffer[8]);
			dataIdBuffer[24] |= 1;
		}

		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

		flagReading[0] = Data_Read_Current0;
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);

		break;

	default:
		HAL_GPIO_WritePin(SEN_GPIO_Port, SEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);

		flagReading[0] = Data_Read_Current0;
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, READING_DELAY_CURR);

		break;
	}

	//Convert ADC into MCU temperature
	dataBuffer[25] = __PDM_CONVERT_MCU_TEMPERATURE(adcBuffer[9]);

	//Start readings timer
	retVal = HAL_TIM_Base_Start_IT(htim);

	//If there is any virtual fuse tripped
	if(flag_fuse != 0)
		PDM_Output_Process();

	return retVal;
}


