/*
 * pdm_specific.h
 *
 *  Created on: 24 de jan de 2023
 *      Author: Rodolfo
 */

#ifndef INC_PDM_SPECIFIC_H_
#define INC_PDM_SPECIFIC_H_

#include "main.h"
#include "pdm.h"

#define PDM_INPUT_GPIO	{&GPIOF, &GPIOF, &GPIOF, &GPIOF, \
						 &GPIOB, &GPIOB, &GPIOC, &GPIOC, \
						 &GPIOC, &GPIOC, &GPIOC, &GPIOC, \
						 &GPIOB, &GPIOB, &GPIOB, &GPIOB}

#define PDM_INPUT_PIN	{GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, \
						 GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, \
						 GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, \
						 GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}

#define PDM_OUTPUT_GPIO	{GPIOB, GPIOB, GPIOB, GPIOE, \
						 GPIOD, GPIOD, GPIOD, GPIOD, \
						 GPIOD, GPIOD, GPIOD, GPIOD, \
						 GPIOG, GPIOG, GPIOG, GPIOG}

#define PDM_OUTPUT_PIN	{GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_10, GPIO_PIN_14,   \
						 GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_11, GPIO_PIN_10,   \
						 GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_14, \
						 GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_5, GPIO_PIN_4}

#define PDM_OUT_TYPES	{Output_PWM, Output_PWMN, Output_PWM, Output_PWM,    \
						 Output_GPIO, Output_GPIO, Output_GPIO, Output_GPIO, \
						 Output_GPIO, Output_GPIO, Output_GPIO, Output_GPIO, \
						 Output_GPIO, Output_GPIO, Output_GPIO, Output_GPIO}

#define PDM_PWM_TYPES	{Output_PWM, Output_PWMN, Output_PWM, Output_PWM}
#define PDM_PWM_TIMS	{&htim3, &htim8, &htim2, &htim4}
#define PDM_PWM_CHNS	{TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}

/*BEGIN EXTERNAL VARIABLES*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
/*END EXTERNAL VARIABLES*/

#endif /* INC_PDM_SPECIFIC_H_ */
