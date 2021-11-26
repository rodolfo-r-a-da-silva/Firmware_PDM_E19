/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

HAL_StatusTypeDef EEPROM_Read(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef ret_val = HAL_OK;

//	ret_val = AT24Cxx_Read_Page_DMA(hi2c, MemAddress_Start, pData, Size);

	Accumulator_EEPROM_Write = 0;

	return ret_val;
}
