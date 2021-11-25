/*
 * pdm_driver_control.h
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_

#include "main.h"

#include "PCF8574.h"

/*BEGIN DEFINES*/
//#define LQFP64

#define OUTPUT_PWM_ENABLE		0x01
#define OUTPUT_PWM_DISABLE		0x00

#define PWM_FREQ_1000HZ			9999
#define PWM_FREQ_2000HZ			4999
#define PWM_FREQ_5000HZ			1999
#define PWM_FREQ_8000HZ			1249
#define PWM_FREQ_10000HZ		999

#define DATA_FREQ_10HZ			1000
#define DATA_FREQ_25HZ			400
#define DATA_FREQ_50HZ			200
#define DATA_FREQ_100HZ			100

#define ADC_DELAY				60 + 140
/*END DEFINES*/

/*BEGIN ENUMS TYPEDEFS*/
typedef enum{
	Data_Read_Current0		= 0x00,
	Data_Read_Current1		= 0x02,
	Data_Read_Temperature	= 0x03,
	Data_Read_Voltage		= 0x04
}PDM_Data_Read;

typedef enum{
	CAN_125KBPS			= 0x00,
	CAN_250KBPS			= 0x01,
	CAN_500KBPS 		= 0x02,
	CAN_1000KBPS		= 0x03
}PDM_CAN_BaudRate;

typedef enum{
	CAN_ID_CURRENT_1	= 0x0001,
	CAN_ID_CURRENT_2,
	CAN_ID_CURRENT_3,
	CAN_ID_CURRENT_4,
	CAN_ID_CURRENT_5,
	CAN_ID_CURRENT_6,
	CAN_ID_CURRENT_7,
	CAN_ID_CURRENT_8,
	CAN_ID_CURRENT_9,
	CAN_ID_CURRENT_10,
	CAN_ID_CURRENT_11,
	CAN_ID_CURRENT_12,
	CAN_ID_CURRENT_13,
	CAN_ID_CURRENT_14,
	CAN_ID_CURRENT_15,
	CAN_ID_CURRENT_16,
	CAN_ID_TEMP_1,
	CAN_ID_TEMP_2,
	CAN_ID_TEMP_3,
	CAN_ID_TEMP_4,
	CAN_ID_TEMP_5,
	CAN_ID_TEMP_6,
	CAN_ID_TEMP_7,
	CAN_ID_TEMP_8,
	CAN_ID_TEMP_MCU,
	CAN_ID_VCC
}PDM_CAN_IDs;
/*END ENUMS TYPEDEFS*/

/*BEGIN STRUCT TYPEDEFS*/
typedef struct{
	uint16_t Enabled_Inputs[2];
	uint16_t Input_Levels[2];
}Output_Control_Struct;

typedef struct{
	uint8_t Map_Lengths[2];
	uint16_t Input_DC_Preset_Enable[2];
	uint16_t Input_DC_Preset[4];
	uint16_t Duty_Cycle;
	uint16_t Duty_Cycle_Preset[2];
	uint16_t Duty_Cycle_Map[15][15];
	int16_t Command_Var[2];
	int16_t Command_Var_Lim[2][2];
	int16_t Command_Var_Step[15][2];
}PWM_Control_Struct;
/*END STRUCT TYPEDEFS*/

/*BEGIN EXTERNAL VARIABLES*/
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/*END EXTERNAL VARIABLES*/

/*BEGIN DECLARED VARIABLES*/
uint8_t Data_Conversion;

uint8_t PWM_Pin_Status;

uint8_t CanRxData[8];
uint8_t CanTxData[8];

uint8_t Data_Freq_Buffer[26];
uint8_t Data_Verify_Buffer[26];

uint16_t Input_Pin_Levels;

uint16_t Data_ID_Buffer[15];
uint16_t Converted_Data_Buffer[26];

uint16_t Current_Thresholds[16];

uint32_t
	Accumulator_Delay,
	Accumulator_Msg_10,
	Accumulator_Msg_25,
	Accumulator_Msg_50,
	Accumulator_Msg_100;

uint32_t pTxMailbox;

uint32_t ADC_BUFFER[10];

CAN_RxHeaderTypeDef CanRxMessage;
CAN_TxHeaderTypeDef CanTxMessage;

Output_Control_Struct Output_Pin[16];

PWM_Control_Struct PWM_Pins[4];
/*END DECLARED VARIABLES*/

/*BEGIN HSD CONTROL FUNCTION PROTOTYPES*/
void Input_Process();

void Output_Process();
/*END HSD CONTROL FUNCTION PROTOTYPES*/

/*BEGIN HSD PWM CONTROL FUNCTION PROTOTYPES*/
void PDM_PWM_Init(PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number);

void PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number);
/*END HSD PWM CONTROL FUNCTION PROTOTYPES*/

/*BEGIN CAN FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t CAN_BaudRate);

HAL_StatusTypeDef PDM_Can_Transmit_Data(CAN_HandleTypeDef* hcan, uint16_t data_freq);
/*END CAN FUNCTION PROTOTYPES*/

/*BEGIN CONVERSION FUNCTION PROTOTYPES*/
void PDM_Read_Data(uint8_t *Data_read);
/*END CONVERSION FUNCTION PROTOTYPES*/

#endif /* INC_PDM_H_ */
