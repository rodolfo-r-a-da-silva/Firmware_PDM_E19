/*
 * PCF8574.c
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#include "PCF8574.h"

HAL_StatusTypeDef PCF8574_Write_Pins(I2C_HandleTypeDef* hi2c, uint8_t pin_buffer, uint8_t address_pins)
{
	return HAL_I2C_Master_Transmit(hi2c, 0xE0 | ((address_pins & 0x07) << 1), &pin_buffer, 1, 1);
}


HAL_StatusTypeDef PCF8574A_Write_Pins(I2C_HandleTypeDef* hi2c, uint8_t pin_buffer, uint8_t address_pins)
{
	return HAL_I2C_Master_Transmit(hi2c, 0xE1 | ((address_pins & 0x07) << 1), &pin_buffer, 1, 1);
}
