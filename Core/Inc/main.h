/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEL1_Pin GPIO_PIN_2
#define SEL1_GPIO_Port GPIOE
#define SEL0_Pin GPIO_PIN_3
#define SEL0_GPIO_Port GPIOE
#define FAULTRST_Pin GPIO_PIN_4
#define FAULTRST_GPIO_Port GPIOE
#define SEN_Pin GPIO_PIN_5
#define SEN_GPIO_Port GPIOE
#define INPUT1_Pin GPIO_PIN_0
#define INPUT1_GPIO_Port GPIOF
#define INPUT2_Pin GPIO_PIN_1
#define INPUT2_GPIO_Port GPIOF
#define INPUT3_Pin GPIO_PIN_2
#define INPUT3_GPIO_Port GPIOF
#define INPUT4_Pin GPIO_PIN_3
#define INPUT4_GPIO_Port GPIOF
#define ANALOG1_Pin GPIO_PIN_0
#define ANALOG1_GPIO_Port GPIOA
#define ANALOG2_Pin GPIO_PIN_1
#define ANALOG2_GPIO_Port GPIOA
#define ANALOG3_Pin GPIO_PIN_2
#define ANALOG3_GPIO_Port GPIOA
#define ANALOG4_Pin GPIO_PIN_3
#define ANALOG4_GPIO_Port GPIOA
#define ANALOG5_Pin GPIO_PIN_4
#define ANALOG5_GPIO_Port GPIOA
#define ANALOG6_Pin GPIO_PIN_5
#define ANALOG6_GPIO_Port GPIOA
#define ANALOG7_Pin GPIO_PIN_6
#define ANALOG7_GPIO_Port GPIOA
#define ANALOG8_Pin GPIO_PIN_7
#define ANALOG8_GPIO_Port GPIOA
#define ANALOG9_Pin GPIO_PIN_4
#define ANALOG9_GPIO_Port GPIOC
#define OUTPUT2_Pin GPIO_PIN_0
#define OUTPUT2_GPIO_Port GPIOB
#define OUTPUT1_Pin GPIO_PIN_1
#define OUTPUT1_GPIO_Port GPIOB
#define OUTPUT4_Pin GPIO_PIN_2
#define OUTPUT4_GPIO_Port GPIOB
#define OUTPUT3_Pin GPIO_PIN_10
#define OUTPUT3_GPIO_Port GPIOB
#define INPUT13_Pin GPIO_PIN_12
#define INPUT13_GPIO_Port GPIOB
#define INPUT14_Pin GPIO_PIN_13
#define INPUT14_GPIO_Port GPIOB
#define INPUT15_Pin GPIO_PIN_14
#define INPUT15_GPIO_Port GPIOB
#define INPUT16_Pin GPIO_PIN_15
#define INPUT16_GPIO_Port GPIOB
#define OUTPUT6_Pin GPIO_PIN_8
#define OUTPUT6_GPIO_Port GPIOD
#define OUTPUT5_Pin GPIO_PIN_9
#define OUTPUT5_GPIO_Port GPIOD
#define OUTPUT8_Pin GPIO_PIN_10
#define OUTPUT8_GPIO_Port GPIOD
#define OUTPUT7_Pin GPIO_PIN_11
#define OUTPUT7_GPIO_Port GPIOD
#define OUTPUT10_Pin GPIO_PIN_12
#define OUTPUT10_GPIO_Port GPIOD
#define OUTPUT9_Pin GPIO_PIN_13
#define OUTPUT9_GPIO_Port GPIOD
#define OUTPUT12_Pin GPIO_PIN_14
#define OUTPUT12_GPIO_Port GPIOD
#define OUTPUT11_Pin GPIO_PIN_15
#define OUTPUT11_GPIO_Port GPIOD
#define OUTPUT14_Pin GPIO_PIN_2
#define OUTPUT14_GPIO_Port GPIOG
#define OUTPUT13_Pin GPIO_PIN_3
#define OUTPUT13_GPIO_Port GPIOG
#define OUTPUT16_Pin GPIO_PIN_4
#define OUTPUT16_GPIO_Port GPIOG
#define OUTPUT15_Pin GPIO_PIN_5
#define OUTPUT15_GPIO_Port GPIOG
#define CAN_RX_LED_Pin GPIO_PIN_7
#define CAN_RX_LED_GPIO_Port GPIOG
#define CAN_TX_LED_Pin GPIO_PIN_8
#define CAN_TX_LED_GPIO_Port GPIOG
#define INPUT7_Pin GPIO_PIN_6
#define INPUT7_GPIO_Port GPIOC
#define INPUT8_Pin GPIO_PIN_7
#define INPUT8_GPIO_Port GPIOC
#define INPUT9_Pin GPIO_PIN_8
#define INPUT9_GPIO_Port GPIOC
#define INPUT10_Pin GPIO_PIN_9
#define INPUT10_GPIO_Port GPIOC
#define INPUT11_Pin GPIO_PIN_10
#define INPUT11_GPIO_Port GPIOC
#define INPUT12_Pin GPIO_PIN_11
#define INPUT12_GPIO_Port GPIOC
#define INPUT5_Pin GPIO_PIN_4
#define INPUT5_GPIO_Port GPIOB
#define INPUT6_Pin GPIO_PIN_5
#define INPUT6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
