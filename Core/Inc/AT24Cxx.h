/*
 * AT24Cxx.h
 *
 *  Created on: 16 de fev de 2021
 *      Author: Rodolfo
 */

#ifndef INC_AT24CXX_H_
#define INC_AT24CXX_H_

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef AT24Cxx_Read(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef AT24Cxx_Random_Read(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef AT24Cxx_Read_DMA(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef AT24Cxx_Write(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef AT24Cxx_Write_DMA(I2C_HandleTypeDef* hi2c, uint16_t MemAddress_Start, uint8_t* pData, uint16_t Size);

#endif /* INC_AT24CXX_H_ */
