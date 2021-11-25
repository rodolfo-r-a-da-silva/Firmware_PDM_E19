/*
 * PCF8574.h
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#ifndef INC_PCF8574_H_
#define INC_PCF8574_H_

#include "main.h"

HAL_StatusTypeDef PCF8574_Write_Pins(I2C_HandleTypeDef* hi2c, uint8_t pin_buffer, uint8_t address_pins);

HAL_StatusTypeDef PCF8574A_Write_Pins(I2C_HandleTypeDef* hi2c, uint8_t pin_buffer, uint8_t address_pins);

#endif /* INC_PCF8574_H_ */
