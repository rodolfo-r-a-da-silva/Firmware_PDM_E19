/*
 * pdm_eeprom.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Rodolfo
 */

#include "pdm.h"

/*BEGIN STATIC FUNCTIONS*/

static uint32_t Data_Freq_To_Period(Data_Freq freq)
{
	switch(freq){
		case Data_Freq_1Hz:
			return DATA_PER_1HZ;
		case Data_Freq_2Hz:
			return DATA_PER_2HZ;
		case Data_Freq_5Hz:
			return DATA_PER_5HZ;
		case Data_Freq_10Hz:
			return DATA_PER_10HZ;
		case Data_Freq_20Hz:
			return DATA_PER_20HZ;
		case Data_Freq_50Hz:
			return DATA_PER_50HZ;
		case Data_Freq_100Hz:
			return DATA_PER_100HZ;
		case Data_Freq_200Hz:
			return DATA_PER_200HZ;
		case Data_Freq_500Hz:
			return DATA_PER_500HZ;
		default:
			break;
	}

	return DATA_PER_DISABLED;
}

static Data_Freq Data_Period_To_Freq(uint32_t period)
{
	switch(period){
		case DATA_PER_1HZ:
			return Data_Freq_1Hz;
		case DATA_PER_2HZ:
			return Data_Freq_2Hz;
		case DATA_PER_5HZ:
			return Data_Freq_5Hz;
		case DATA_PER_10HZ:
			return Data_Freq_10Hz;
		case DATA_PER_20HZ:
			return Data_Freq_20Hz;
		case DATA_PER_50HZ:
			return Data_Freq_50Hz;
		case DATA_PER_100HZ:
			return Data_Freq_100Hz;
		case DATA_PER_200HZ:
			return Data_Freq_200Hz;
		case DATA_PER_500HZ:
			return Data_Freq_500Hz;
	}

	return Data_Freq_Disabled;
}

/*END STATIC FUNCTIONS*/
