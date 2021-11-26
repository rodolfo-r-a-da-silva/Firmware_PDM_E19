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
#include "AT24Cxx.h"

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
	CAN_ID_VCC,
	CAN_ID_PWM_DC1,
	CAN_ID_PWM_DC2,
	CAN_ID_PWM_DC3,
	CAN_ID_PWM_DC4
}PDM_CAN_IDs;
/*END ENUMS TYPEDEFS*/

/*BEGIN STRUCT TYPEDEFS*/
typedef struct{
	uint16_t Enabled_Inputs[2];
	uint16_t Input_Levels[2];
}Output_Control_Struct;

typedef struct{
	uint16_t PWM_Frequency;

	uint16_t Input_DC_Preset_Enable[2];
	uint16_t Input_DC_Preset[4];
	uint16_t Duty_Cycle_Preset[2];

	uint8_t PWM_CAN_Enable;
	uint8_t Command_Var_Position[2];
	uint32_t Command_Var_CAN_ID[2];

	uint8_t Map_Lengths[2];
	int16_t Command_Var[2];
	int16_t Command_Var_Lim[2][2];
	int16_t Command_Var_Step[15][2];
	uint16_t Duty_Cycle;
	uint16_t Duty_Cycle_Map[15][15];
}PWM_Control_Struct;
/*END STRUCT TYPEDEFS*/

/*BEGIN DEFINES*/
//#define LQFP64

#define OUTPUT_PWM_ENABLE			0x01
#define OUTPUT_PWM_DISABLE			0x00

#define OUTPUT_PWM_CAN_ENABLE		0x01
#define OUTPUT_PWM_CAN_DISABLE		0x00

#define OUTPUT_FUSE_FREQ			100

#define EEPROM_WRITE_CYCLE			50

#define PWM_FREQ_1000HZ				9999
#define PWM_FREQ_2000HZ				4999
#define PWM_FREQ_5000HZ				1999
#define PWM_FREQ_8000HZ				1249
#define PWM_FREQ_10000HZ			999

#define DATA_FREQ_10HZ				1000
#define DATA_FREQ_25HZ				400
#define DATA_FREQ_50HZ				200
#define DATA_FREQ_100HZ				100

#define NBR_OF_OUTPUTS				16
#define NBR_OF_DATA_CHANNELS		30

#define CURRENT_LINEAR				5879
#define CURRENT_OFFSET				37596
#define CURRENT_DIV_FACT			10000
#define TEMPERATURE_LINEAR			-22
#define TEMPERATURE_OFFSET			40677
#define TEMPERATURE_DIV_FACT		10
#define VOLTAGE_LINEAR				4804
#define VOLTAGE_DIV_FACT			10000
#define MCU_TEMP_LINEAR				32234
#define MCU_TEMP_OFFSET				-2790000
#define MCU_TEMP_DIV_FACT			1000

#define REDING_ADC_DELAY			140
#define READING_DELAY_CURRENT0		20 + REDING_ADC_DELAY
#define READING_DELAY_CURRENT1		20 + REDING_ADC_DELAY
#define READING_DELAY_TEMPERATURE	60 + REDING_ADC_DELAY
#define READING_DELAY_VOLTAGE		20 + REDING_ADC_DELAY
/*END DEFINES*/

/*BEGIN MACROS*/
#define __PDM_INPUT_CONDITION_COMPARE(__ENABLED_INPUTS0__, __INPUT_LEVELS0__, __ENABLED_INPUTS1__, __INPUT_LEVELS1__)	\
		(((Input_Pin_Levels & (__ENABLED_INPUTS0__)) == (__INPUT_LEVELS0__)) 											\
		|| ((Input_Pin_Levels & (__ENABLED_INPUTS1__)) == (__INPUT_LEVELS1__)))

#define __PDM_PWM_SELECT_TIM(__HANDLE__, __CHANNEL__)	\
		htim = (__HANDLE__);							\
		tim_channel = (__CHANNEL__);

#define __PDM_CONVERT_CURRENT(__ADC_VALUE__)	((((__ADC_VALUE__) * CURRENT_LINEAR) + CURRENT_OFFSET) / CURRENT_DIV_FACT)

#define __PDM_CONVERT_MCU_TEMPERATURE(__ADC_VALUE__)	((((__ADC_VALUE__) * MCU_TEMP_LINEAR) + MCU_TEMP_OFFSET) / MCU_TEMP_DIV_FACT)

#define __PDM_CONVERT_TEMPERATURE(__ADC_VALUE__, __VND_GND__)	(((((__ADC_VALUE__) - (__VND_GND__)) * TEMPERATURE_LINEAR) + TEMPERATURE_LINEAR) / TEMPERATURE_DIV_FACT)

#define __PDM_CONVERT_VOLTAGE(__ADC_VALUE__, __VND_GND__)	((((__ADC_VALUE__) - (__VND_GND__)) * VOLTAGE_LINEAR) / VOLTAGE_DIV_FACT)
/*END MACROS*/

/*BEGIN EXTERNAL VARIABLES*/
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
/*END EXTERNAL VARIABLES*/

/*BEGIN DECLARED VARIABLES*/
uint8_t Data_Conversion;

uint8_t PWM_Pin_Status;

uint8_t EEPROM_Write_Status;

uint8_t Can_Rx_Data[8];
uint8_t Can_Tx_Data[8];

uint8_t Data_Freq_Buffer[30];
uint8_t Data_Verify_Buffer[30];

uint16_t Input_Pin_Levels;
uint16_t Driver_Overcurrent_Flag;
uint16_t Driver_Safety_Flag;

uint16_t Data_Buffer[30];
uint16_t Data_ID_Buffer[30];

uint16_t Current_Thresholds[16];
uint16_t Timeout_Output_Fuse[16];

uint32_t
	Accumulator_Delay,
	Accumulator_EEPROM_Write,
	Accumulator_Output_Check,
	Accumulator_Msg_10Hz,
	Accumulator_Msg_25Hz,
	Accumulator_Msg_50Hz,
	Accumulator_Msg_100Hz;

uint32_t pTxMailbox;

uint32_t Accumulator_Output_Fuse[16];

uint32_t ADC_BUFFER[10];

CAN_RxHeaderTypeDef Can_Rx_Message;
CAN_TxHeaderTypeDef Can_Tx_Message;

Output_Control_Struct Output_Pin[16];

PWM_Control_Struct PWM_Pins[4];
/*END DECLARED VARIABLES*/

/*BEGIN CONFIGURATION FUNCTION PROTOTYPES*/
HAL_StatusTypeDef EEPROM_Read(I2C_HandleTypeDef *hi2c);
/*END CONFIGURATION FUNCTION PROTOTYPES*/

/*BEGIN HSD CONTROL FUNCTION PROTOTYPES*/
void PDM_Input_Process();

void PDM_Output_Process();

void PDM_Output_Fuse();
/*END HSD CONTROL FUNCTION PROTOTYPES*/

/*BEGIN HSD PWM CONTROL FUNCTION PROTOTYPES*/
void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number);

void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number);
/*END HSD PWM CONTROL FUNCTION PROTOTYPES*/

/*BEGIN CAN FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t CAN_BaudRate);

HAL_StatusTypeDef PDM_PWM_CAN_Filter_Config(CAN_HandleTypeDef *hcan, PWM_Control_Struct *pwm_struct, uint8_t can_filter_bank);

HAL_StatusTypeDef PDM_CAN_Transmit_Data(CAN_HandleTypeDef* hcan, uint16_t data_freq);

void PDM_CAN_Process_Rx_Data();
/*END CAN FUNCTION PROTOTYPES*/

/*BEGIN CONVERSION FUNCTION PROTOTYPES*/
void PDM_Read_Data(uint8_t *Data_read);
/*END CONVERSION FUNCTION PROTOTYPES*/

#endif /* INC_PDM_H_ */
