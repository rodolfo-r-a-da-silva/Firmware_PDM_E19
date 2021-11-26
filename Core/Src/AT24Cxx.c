/*
 * AT24Cxx.c
 *
 *  Created on: 16 de fev de 2021
 *      Author: Rodolfo
 */

#include "AT24Cxx.h"

HAL_StatusTypeDef AT24Cxx_Read_Byte(I2C_HandleTypeDef* hi2c, uint16_t MemAddress, uint8_t* pData)
{
	return HAL_I2C_Mem_Read(hi2c, 0xA1 | ((MemAddress >> 7) & 0x0E), MemAddress & 0xff, 1, pData, 1, 1);
}

HAL_StatusTypeDef AT24Cxx_Write_Byte(I2C_HandleTypeDef* hi2c, uint16_t MemAddress, uint8_t* pData)
{
	return HAL_I2C_Mem_Write(hi2c, 0xA0 | ((MemAddress >> 7) & 0x0E), MemAddress & 0xff, 1, pData, 1, 1);
}

HAL_StatusTypeDef AT24Cxx_Read_Page(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	//Checks if it's out of page bound
	if(((MemAddress_Start & 0xff) + Size) > 255)
	{
		return HAL_ERROR;
	}

	return HAL_I2C_Mem_Read(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size, 1);
}

HAL_StatusTypeDef AT24Cxx_Read_Page_DMA(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	//Checks if it's out of page bound
	if(((MemAddress_Start & 0xff) + Size) > 255)
	{
		return HAL_ERROR;
	}

	return HAL_I2C_Mem_Read_DMA(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size);
}


HAL_StatusTypeDef AT24Cxx_Write_Page(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	//Checks if it's out of page bound
	if(((MemAddress_Start & 0xff) + Size) > 255)
	{
		return HAL_ERROR;
	}

	return HAL_I2C_Mem_Write(hi2c, 0xA0 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size, 1);
}
