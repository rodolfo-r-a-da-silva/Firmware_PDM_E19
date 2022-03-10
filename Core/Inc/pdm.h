/*
 * pdm_driver_control.h
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_

#include "main.h"

//#include "PCF8574.h"
#include "AT24Cxx.h"

/*BEGIN DEFINES*/
//#define LQFP64

#define OUTPUT_FUSE_FREQ			25

#define NBR_OF_DATA_CHANNELS		30
#define NBR_OF_OUTPUTS				16
#define NBR_OF_PWM_OUTPUTS			4

#define EEPROM_OUTPUT_SIZE			(NBR_OF_OUTPUTS * 6 * sizeof(uint16_t))
#define EEPROM_PWM_CONFIG_SIZE		(NBR_OF_PWM_OUTPUTS * ((5 * sizeof(uint8_t)) + (14 * sizeof(uint16_t)) + (2 * sizeof(uint32_t))))
#define EEPROM_PWM_MAP_BUFFER_SIZE	(NBR_OF_PWM_OUTPUTS * PWM_TABLE_MAX_SIZE * PWM_TABLE_MAX_SIZE * sizeof(uint16_t))
#define EEPROM_BUFFER_SIZE			(EEPROM_OUTPUT_SIZE + EEPROM_PWM_CONFIG_SIZE + EEPROM_PWM_MAP_BUFFER_SIZE + ((2 + NBR_OF_DATA_CHANNELS) * sizeof(uint8_t)))
#define EEPROM_WRITE_CYCLE			50

#define OUTPUT_PWM_ENABLE			0x01
#define OUTPUT_PWM_DISABLE			0x00

#define OUTPUT_PWM_CAN_ENABLE		0x10
#define OUTPUT_PWM_CAN_DISABLE		0x00

#define PWM_FREQ_1000HZ				9999
#define PWM_FREQ_2000HZ				4999
#define PWM_FREQ_5000HZ				1999
#define PWM_FREQ_8000HZ				1249
#define PWM_FREQ_10000HZ			999

#define PWM_TABLE_MAX_SIZE			15

#define DATA_FREQ_USB				1000
#define DATA_FREQ_10HZ				1000
#define DATA_FREQ_25HZ				400
#define DATA_FREQ_50HZ				200
#define DATA_FREQ_80HZ				125
#define DATA_FREQ_100HZ				100
#define DATA_NO_TRANSMISSION		0

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

#define ADC_THRESHOLD_LOW			10
#define ADC_THRESHOLD_HIGH			4000

//us *100
#define READING_ADC_DELAY			140
#define READING_DELAY				3//60 + READING_ADC_DELAY
//ms
#define READING_DELAY_TEMP			10
#define READING_DELAY_VOLT			10

#define USB_COMMAND_CONNECT			0x01
#define USB_COMMAND_DISCONNECT		0x02
#define USB_COMMAND_READ_CONFIG		0x03
#define USB_COMMAND_WRITE_CONFIG	0x04
#define USB_COMMAND_READ_DATA		0x05
/*END DEFINES*/

/*BEGIN MACROS*/
#define __PDM_ID_BUFFER_INIT()								\
		for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)	\
			Data_ID_Buffer[i] = i << 1;

#define __PDM_INPUT_CONDITION_COMPARE(__ENABLED_INPUTS__, __INPUT_LEVELS__)	\
		(((Input_Pin_Levels & (__ENABLED_INPUTS__)) == (__INPUT_LEVELS__)))

#ifdef LQFP64
#define __PDM_PWM_SELECT_TIM(__PWM_OUT_NUMBER__)	\
		switch((__PWM_OUT_NUMBER__))				\
		{											\
		case 0:										\
			htim = &htim3;							\
			tim_channel = TIM_CHANNEL_4;			\
			break;									\
		case 1:										\
			htim = &htim3;							\
			tim_channel = TIM_CHANNEL_3;			\
			break;									\
		case 2:										\
			htim = &htim2;							\
			tim_channel = TIM_CHANNEL_3;			\
			break;									\
		case 3:										\
			htim = &htim2;							\
			tim_channel = TIM_CHANNEL_4;			\
			break;									\
		default:									\
			return;									\
		}
#else
#define __PDM_PWM_SELECT_TIM(__PWM_OUT_NUMBER__)	\
		switch((__PWM_OUT_NUMBER__))				\
		{											\
		case 0:										\
			htim = &htim3;							\
			tim_channel = TIM_CHANNEL_4;			\
			break;									\
		case 1:										\
			htim = &htim8;							\
			tim_channel = TIM_CHANNEL_2;			\
			break;									\
		case 2:										\
			htim = &htim2;							\
			tim_channel = TIM_CHANNEL_3;			\
			break;									\
		case 3:										\
			htim = &htim1;							\
			tim_channel = TIM_CHANNEL_4;			\
			break;									\
		default:									\
			return;									\
		}
#endif

#define __PDM_LINEAR_INTERPOLATION(__X__, __X0__, __X1__, __Y0__, __Y1__)						\
		(((((__X__) - (__X0__)) * ((__Y1__) - (__Y0__))) / ((__X1__) - (__X0__))) + (__Y0__))

#define __PDM_BILINEAR_INTERPOLATION(__X__, __Y__, __X0__, __X1__, __Y0__, __Y1__, __Z00__, __Z01__, __Z10__, __Z11__)	\
		__PDM_LINEAR_INTERPOLATION((__Y__), (__Y0__), (__Y1__),															\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z00__), (__Z01__))),						\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z10__), (__Z11__))))

#define __PDM_CONVERT_CURRENT(__ADC_VALUE__)	((((__ADC_VALUE__) * CURRENT_LINEAR) + CURRENT_OFFSET) / CURRENT_DIV_FACT)

#define __PDM_CONVERT_MCU_TEMPERATURE(__ADC_VALUE__)	((((__ADC_VALUE__) * MCU_TEMP_LINEAR) + MCU_TEMP_OFFSET) / MCU_TEMP_DIV_FACT)

#define __PDM_CONVERT_TEMPERATURE(__ADC_VALUE__, __VND_GND__)	(((((__ADC_VALUE__) - (__VND_GND__)) * TEMPERATURE_LINEAR) + TEMPERATURE_LINEAR) / TEMPERATURE_DIV_FACT)

#define __PDM_CONVERT_VOLTAGE(__ADC_VALUE__, __VND_GND__)	((((__ADC_VALUE__) - (__VND_GND__)) * VOLTAGE_LINEAR) / VOLTAGE_DIV_FACT)
/*END MACROS*/

/*BEGIN ENUMS TYPEDEFS*/
typedef enum{
	CAN_Disable				= 0x00,
	CAN_125kbps,
	CAN_250kbps,
	CAN_500kbps,
	CAN_1000kbps
}PDM_CAN_BaudRate;

typedef enum{
	CAN_ID_CURRENT_1		= 0x0001,
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

typedef enum{
	Data_Disable			= 0x00,
	Data_Freq_10Hz,
	Data_Freq_25Hz,
	Data_Freq_50Hz,
	Data_Freq_80Hz,
	Data_Freq_100Hz
}PDM_Data_Freq;

typedef enum{
	Data_Read_Current0		= 0x00,
	Data_Read_Current1,
	Data_Read_Temperature,
	Data_Read_Voltage
}PDM_Data_Read;

typedef enum{
	PWM_FREQ_1kHz			= 0x01,
	PWM_FREQ_2kHz,
	PWM_FREQ_5kHz,
	PWM_FREQ_8kHz,
	PWM_FREQ_10kHz
}PDM_PWM_FREQ;
/*END ENUMS TYPEDEFS*/

/*BEGIN STRUCT TYPEDEFS*/
typedef struct{
	uint16_t Enabled_Inputs[2];
	uint16_t Input_Levels[2];
	uint16_t Current_Thresholds;
	uint16_t Timeout_Output_Fuse;
}Output_Control_Struct;

typedef struct{
	uint16_t Duty_Cycle;
	uint16_t PWM_Frequency;

	//Used to set specific duty cycles
	uint16_t Input_DC_Preset_Enable[2];
	uint16_t Input_DC_Preset[2];
	uint16_t Duty_Cycle_Preset[2];

	//Used for PWM CAN
	uint8_t Command_Var_Position[2];
	uint32_t Command_Var_CAN_ID[2];

	uint8_t Map_Lengths[2];
	int16_t Command_Var[2];
	//[0][i]: column; [1][j]: line
	int16_t Command_Var_Lim[2][2];
	int16_t Command_Var_Step[2][PWM_TABLE_MAX_SIZE];
	//[line][column]
	uint16_t Duty_Cycle_Map[PWM_TABLE_MAX_SIZE][PWM_TABLE_MAX_SIZE];
}PWM_Control_Struct;
/*END STRUCT TYPEDEFS*/

/*BEGIN DECLARED VARIABLES*/
//AUTO GENERATED VARIABLES
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;

#ifndef LQFP64
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
#endif

//CAN
extern uint8_t
	CAN_Baud_Rate,
	CAN_Rx_Data[8],
	CAN_Tx_Data[8];
extern uint32_t pTxMailbox;
extern CAN_RxHeaderTypeDef CAN_Rx_Message;
extern CAN_TxHeaderTypeDef CAN_Tx_Message;

//CONFIGURATION
extern uint8_t
	USB_Connected_Flag,
	USB_VCP_Parameters[7];

//CURRENT MONITORING
extern uint16_t
	Driver_Overcurrent_Flag,
	Driver_Safety_Flag;

//DATA
extern uint8_t
	Data_Conversion,
	Data_Freq_Buffer[30];
extern uint16_t
	Data_Buffer[30],
	Data_ID_Buffer[30];
extern uint16_t ADC_BUFFER[10];

//OUTPUTS
extern uint8_t PWM_Pin_Status;
extern uint16_t Input_Pin_Levels;
extern Output_Control_Struct Output_Pin[16];
extern PWM_Control_Struct PWM_Pins[4];

//TIMING
extern int32_t Accumulator_Delay;

extern uint32_t
	Accumulator_Msg_10Hz,
	Accumulator_Msg_25Hz,
	Accumulator_Msg_50Hz,
	Accumulator_Msg_80Hz,
	Accumulator_Msg_100Hz,
	Accumulator_USB_Data,
	Accumulator_Output_Check,
	Accumulator_Temp_Read,
	Accumulator_Volt_Read,
	Accumulator_Output_Fuse[16];
/*END DECLARED VARIABLES*/

/*BEGIN CAN FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t CAN_BaudRate);

HAL_StatusTypeDef PDM_PWM_CAN_Filter_Config(CAN_HandleTypeDef *hcan, PWM_Control_Struct *pwm_struct, uint8_t can_filter_bank);

HAL_StatusTypeDef PDM_CAN_Transmit_Data(CAN_HandleTypeDef* hcan, uint8_t data_freq);

void PDM_CAN_Process_Rx_Data();
/*END CAN FUNCTION PROTOTYPES*/

/*BEGIN CONFIGURATION FUNCTION PROTOTYPES*/
void PDM_Init(CAN_HandleTypeDef *hcan, I2C_HandleTypeDef *hi2c);

void PDM_USB_Process(uint8_t *Data, uint16_t Size);

void PDM_USB_Transmit_Data();

void PDM_Hard_Code_Config();
/*END CONFIGURATION FUNCTION PROTOTYPES*/

/*BEGIN CONVERSION FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_Read_Data(uint8_t *Data_read);
/*END CONVERSION FUNCTION PROTOTYPES*/

/*BEGIN HSD CONTROL FUNCTION PROTOTYPES*/
void PDM_Input_Process();

void PDM_Output_Process();

void PDM_Output_Fuse();
/*END HSD CONTROL FUNCTION PROTOTYPES*/

/*BEGIN HSD PWM CONTROL FUNCTION PROTOTYPES*/
void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number);

void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number);
/*END HSD PWM CONTROL FUNCTION PROTOTYPES*/

#endif /* INC_PDM_H_ */
