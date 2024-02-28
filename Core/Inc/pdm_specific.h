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

//Indicate if HSD ICs can read temperature and voltage
#define READ_TEMP
#define READ_VOLT

//Buffer sizes and "for" stops
#define NBR_OF_ADC_HANDLERS		2
#define NBR_OF_ADC_CHANNELS		10
#define NBR_OF_DATA_CHANNELS	34
#define NBR_OF_CAN_CHANNELS		25
#define NBR_OF_DATA_MSGS		10
#define NBR_OF_DIGITAL_INPUTS	16
#define NBR_OF_ANALOG_INPUTS	0
#define NBR_OF_OUTPUTS			16
#define NBR_OF_PWM_OUTPUTS		4
#define NBR_OF_CUSTOM_FUNCS		100

//Amount of each type of local channel
#define NBR_OF_CHN_LCL_DIG_IN	NBR_OF_DIGITAL_INPUTS
#define NBR_OF_CHN_LCL_ANL_IN	NBR_OF_ANALOG_INPUTS
#define NBR_OF_CHN_LCL_CURR		NBR_OF_OUTPUTS
#define NBR_OF_CHN_LCL_TEMP 	(NBR_OF_OUTPUTS+1)/2 + 1
#define NBR_OF_CHN_LCL_VOLT		1
#define NBR_OF_CHN_LCL_ANALOG	NBR_OF_CHN_LCL_ANL_IN + NBR_OF_CHN_LCL_CURR + NBR_OF_CHN_LCL_TEMP + NBR_OF_CHN_LCL_VOLT
#define NBR_OF_CHN_LCL_TOTAL	NBR_OF_CHN_LCL_DIG_IN + NBR_OF_CHN_LCL_ANL_IN + NBR_OF_CHN_LCL_CURR + NBR_OF_CHN_LCL_TEMP + NBR_OF_CHN_LCL_VOLT


//Offsets for each type of channel in local channels array
#define CHN_LCL_DIG_IN_OFFSET	0
#define CHN_LCL_DIG_IN_FINISH	CHN_LCL_DIG_IN_OFFSET + NBR_OF_CHN_LCL_DIG_IN
#define CHN_LCL_ANL_IN_OFFSET	CHN_LCL_DIG_IN_FINISH
#define CHN_LCL_ANL_IN_FINISH	CHN_LCL_ANL_IN_OFFSET + NBR_OF_CHN_LCL_ANL_IN
#define CHN_LCL_CURR_OFFSET		CHN_LCL_ANL_IN_FINISH
#define CHN_LCL_CURR_FINISH		CHN_LCL_CURR_OFFSET + NBR_OF_CHN_LCL_CURR
#define CHN_LCL_TEMP_OFFSET		CHN_LCL_CURR_FINISH
#define CHN_LCL_TEMP_FINISH		CHN_LCL_TEMP_OFFSET + NBR_OF_CHN_LCL_TEMP
#define CHN_LCL_VOLT_OFFSET		CHN_LCL_TEMP_FINISH
#define CHN_LCL_VOLT_FINISH		CHN_LCL_VOLT_OFFSET + NBR_OF_CHN_LCL_VOLT

//Offsets for each type of channel in channels array
#define CHN_CAN_OFFSET	0
#define CHN_CAN_FINISH	CHN_CAN_OFFSET + NBR_OF_CAN_CHANNELS

//Offsets for each type of function in functions array
#define FNC_DIG_IN_OFFSET	0
#define FNC_DIG_IN_FINISH	FNC_DIG_IN_OFFSET + NBR_OF_CHN_LCL_DIG_IN
#define FNC_ANALOG_OFFSET	FNC_DIG_IN_FINISH
#define FNC_ANALOG_FINISH	FNC_ANALOG_OFFSET + NBR_OF_CHN_LCL_ANALOG
#define FNC_CUSTOM_OFFSET	FNC_ANALOG_FINISH
#define FNC_CUSTOM_FINISH	FNC_CUSTOM_OFFSET + NBR_OF_CUSTOM_FUNCS

//Starting position for ADCs
#define PDM_ADC_BUFFER_POS	{0, 5}

//Input Pin hardware
#define PDM_INPUT_GPIO	{&GPIOF, &GPIOF, &GPIOF, &GPIOF, \
						 &GPIOB, &GPIOB, &GPIOC, &GPIOC, \
						 &GPIOC, &GPIOC, &GPIOC, &GPIOC, \
						 &GPIOB, &GPIOB, &GPIOB, &GPIOB}

#define PDM_INPUT_PIN	{GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, \
						 GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, \
						 GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, \
						 GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}

//Output Pin hardware
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

//PWM Outputs
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
