/*
 * AT24Cxx.c
 *
 *  Created on: 16 de fev de 2021
 *      Author: Rodolfo
 */

#include "AT24Cxx.h"

HAL_StatusTypeDef AT24Cxx_Read(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	return HAL_I2C_Mem_Read(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size, 1);
}

HAL_StatusTypeDef AT24Cxx_Read_DMA(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	return HAL_I2C_Mem_Read_DMA(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size);
}

HAL_StatusTypeDef AT24Cxx_Random_Read(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	HAL_StatusTypeDef ret_val = HAL_OK;

	ret_val = HAL_I2C_Master_Transmit(hi2c, 0xA0 | ((MemAddress_Start >> 7) & 0x0E), (uint8_t*) 0x00, 1, 1);

	if(ret_val != HAL_OK)
		return ret_val;

	return HAL_I2C_Master_Receive(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), pData, Size, 50);
}

HAL_StatusTypeDef AT24Cxx_Write(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	return HAL_I2C_Mem_Write(hi2c, 0xA0 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size, 1);
}

HAL_StatusTypeDef AT24Cxx_Write_DMA(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size)
{
	return HAL_I2C_Mem_Write_DMA(hi2c, 0xA1 | ((MemAddress_Start >> 7) & 0x0E), MemAddress_Start, 1, pData, Size);
}
