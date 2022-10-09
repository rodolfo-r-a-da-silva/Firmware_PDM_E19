/*
 * pdm_driver_control.h
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_

#include "main.h"

/*BEGIN DEFINES*/
//CAN
#define CAN_CONFIG_FILTER			0x00
#define CAN_CONFIG_MASK				0x00
#define CAN_DATA_ID					0x1E35C000
#define CAN_DATA_MASK				0x1FFFF000

#define OUTPUT_FUSE_FREQ			25

#define NBR_OF_DATA_CHANNELS		30
#define NBR_OF_OUTPUTS				16
#define NBR_OF_PWM_OUTPUTS			4

#define OUTPUT_PWM_ENABLE			0x01
#define OUTPUT_PWM_DISABLE			0x00

#define OUTPUT_PWM_CAN_ENABLE		0x10
#define OUTPUT_PWM_CAN_DISABLE		0x00

#define PWM_FREQ_100HZ				899
#define PWM_FREQ_250HZ				359
#define PWM_FREQ_500HZ				179
#define PWM_FREQ_750HZ				119
#define PWM_FREQ_1000HZ				89
#define PWM_FREQ_2500HZ				35
#define PWM_FREQ_5000HZ				17
#define PWM_FREQ_7500HZ				11
#define PWM_FREQ_10000HZ			8
#define PWM_FREQ_15000HZ			5

#define PWM_TABLE_MAX_SIZE			16

//DATA TRANSMISSION
#define DATA_FREQ_USB				1000
#define DATA_FREQ_10HZ				1000
#define DATA_FREQ_25HZ				400
#define DATA_FREQ_50HZ				200
#define DATA_FREQ_80HZ				125
#define DATA_FREQ_100HZ				100
#define DATA_NO_TRANSMISSION		0

//DATA CONVERSION CONSTANTS
#define CURRENT_LINEAR				588
#define CURRENT_OFFSET				-40000//3760
#define CURRENT_DIV_FACT			1000
#define TEMPERATURE_LINEAR			-22
#define TEMPERATURE_OFFSET			40677
#define TEMPERATURE_DIV_FACT		1
#define VOLTAGE_LINEAR				7400//9567
#define VOLTAGE_DIV_FACT			10000
#define MCU_TEMP_LINEAR				32
#define MCU_TEMP_OFFSET				-27900
#define MCU_TEMP_DIV_FACT			1

#define ADC_THRESHOLD_LOW			10
#define ADC_THRESHOLD_HIGH			4000

//us
#define READING_DELAY_CURR			1000
#define READING_DELAY_TEMP			1000
#define READING_DELAY_VOLT			1000

#define EEPROM_I2C_ADDRESS			0xA0
#define EEPROM_TIMEOUT				10
#define EEPROM_OUT_CFG_ADDRESS		0x0000
#define EEPROM_PWM_CFG_ADDRESS		0x00C0
#define EEPROM_PWM1_CFG1_ADDRESS	0x0100
#define EEPROM_PWM1_CFG2_ADDRESS	0x107F
#define EEPROM_PWM1_SST1_ADDRESS	0x0
#define EEPROM_PWM1_SST2_ADDRESS	0x0
#define EEPROM_PWM2_CFG1_ADDRESS	0x1FFE
#define EEPROM_PWM2_CFG2_ADDRESS	0x2F7D
#define EEPROM_PWM2_SST1_ADDRESS	0x0
#define EEPROM_PWM2_SST2_ADDRESS	0x0
#define EEPROM_PWM3_CFG1_ADDRESS	0x3EFC
#define EEPROM_PWM3_CFG2_ADDRESS	0x4E7B
#define EEPROM_PWM3_SST1_ADDRESS	0x0
#define EEPROM_PWM3_SST2_ADDRESS	0x0
#define EEPROM_PWM4_CFG1_ADDRESS	0x5DFA
#define EEPROM_PWM4_CFG2_ADDRESS	0x6D79
#define EEPROM_PWM4_SST1_ADDRESS	0x0
#define EEPROM_PWM4_SST2_ADDRESS	0x0
#define EEPROM_OUT_CFG_BUFFER_SIZE	NBR_OF_OUTPUTS * 6 * sizeof(uint16_t)
#define EEPROM_PWM_CFG_BUFFER_SIZE	NBR_OF_PWM_OUTPUTS * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))
#define EEPROM_PWM_CFG_MAX_SIZE		0x0F7F
/*END DEFINES*/

/*BEGIN MACROS*/
#define __PDM_ID_BUFFER_INIT()								\
		for(uint16_t i = 0; i < NBR_OF_DATA_CHANNELS; i++)	\
			dataIdBuffer[i] = i << 1;

#define __PDM_INPUT_CONDITION_COMPARE(__ENABLED_INPUTS__, __INPUT_LEVELS__, __OUTPUT_ENABLE__)	\
		(((inputLevels & (__ENABLED_INPUTS__)) == ((__INPUT_LEVELS__) & (__ENABLED_INPUTS__))) && ((__OUTPUT_ENABLE__) == Output_Enabled))

#define __PDM_FUSE_CONDITION(__OUT_NUMBER__)													\
		(outputCurrent[(__OUT_NUMBER__) - 1] > thresholdFuse[(__OUT_NUMBER__) - 1])				\
		&& (accFuse[(__OUT_NUMBER__) - 1] > timeoutFuse[(__OUT_NUMBER__) - 1])					\
		&& (timeoutFuse[(__OUT_NUMBER__) - 1] > 0) && (thresholdFuse[(__OUT_NUMBER__) - 1] > 0)	\
		&& (((flagDriverSafety >> ((__OUT_NUMBER__) - 1)) & 0x01) == 0x00)

#define __PDM_PWM_SELECT_TIM(__PWM_OUT_NUMBER__)	\
		switch((__PWM_OUT_NUMBER__))				\
		{											\
		case 0:										\
			htim = &htim3;							\
			timChannel = TIM_CHANNEL_4;				\
			break;									\
		case 1:										\
			htim = &htim8;							\
			timChannel = TIM_CHANNEL_2;				\
			break;									\
		case 2:										\
			htim = &htim2;							\
			timChannel = TIM_CHANNEL_3;				\
			break;									\
		case 3:										\
			htim = &htim1;							\
			timChannel = TIM_CHANNEL_4;				\
			break;									\
		default:									\
			return;									\
		}

#define __PDM_LINEAR_INTERPOLATION(__X__, __X0__, __X1__, __Y0__, __Y1__)						\
		(((((__X__) - (__X0__)) * ((__Y1__) - (__Y0__))) / ((__X1__) - (__X0__))) + (__Y0__))

#define __PDM_BILINEAR_INTERPOLATION(__X__, __Y__, __X0__, __X1__, __Y0__, __Y1__, __Z00__, __Z01__, __Z10__, __Z11__)	\
		__PDM_LINEAR_INTERPOLATION((__Y__), (__Y0__), (__Y1__),															\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z00__), (__Z01__))),						\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z10__), (__Z11__))))

#define __PDM_GET_PWM_MAP_BYTE_SIZE(__PWM_OUT_NUMBER__)																								\
		pwmOutStruct[__PWM_OUT_NUMBER__].pwmMapStruct->mapLengths[0] * pwmOutStruct[__PWM_OUT_NUMBER__].pwmMapStruct->mapLengths[1] * sizeof(uint16_t)

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
	Data_Read_Ready,
	Data_Read_Waiting,
	Data_Read_Current0,
	Data_Read_Current1,
	Data_Read_Temperature,
	Data_Read_Voltage
}PDM_Data_Read;

//typedef enum{
//	PWM_FREQ_1kHz			= 0x01,
//	PWM_FREQ_2kHz,
//	PWM_FREQ_5kHz,
//	PWM_FREQ_8kHz,
//	PWM_FREQ_10kHz
//}PDM_PWM_FREQ;

typedef enum{
	Output_Disabled,
	Output_Enabled
}PDM_Output_Enabled;

typedef enum{
	OutType_Standard 		= 0x00,
	OutType_Preset,
	OutType_Map,
	OutType_ANN,
	OutType_Error
}PDM_OutType;

typedef enum{
	ANN_In_CAN,
	ANN_In_Neuron,
	ANN_In_Local
}PDM_ANN_InType;

typedef enum{
	SoftStart_Disabled,
	SoftStart_Enabled
}PDM_SoftStart;
/*END ENUMS TYPEDEFS*/

/*BEGIN STRUCT TYPEDEFS*/
typedef struct{
	PDM_CAN_BaudRate baudRate;
	uint16_t enabledFilters;

	uint16_t filterIde;
	uint32_t filters[13];
	uint32_t masks[13];
}PDM_CAN_Config;

typedef struct{
	uint16_t inputEnable[2];
	uint16_t inputLevels[2];
	uint16_t currentThresholds;
	uint16_t timeoutOutputFuse;
	PDM_Output_Enabled outEnable[2];
}Output_Control_Struct;

typedef struct{
	uint16_t dutyCycles;
	uint16_t turnOnTime;
	uint16_t* dutyCycleBuffer;
	uint32_t slope;
}PDM_PWM_SoftStart_Struct;

typedef struct{
	int16_t output;
	int16_t* weigths;
	uint16_t* inputId;
	PDM_ANN_InType* inputType;
}PDM_PWM_ANN_Struct;

typedef struct{
	//Used for PWM CAN
	uint16_t canVarID[2];

	uint8_t mapLengths[2];
	int16_t commandVar[2];
	//[0][i]: column; [1][j]: line
	int16_t* commandVarStep[2];
	//[column][line] or [x][y]
	uint16_t** dutyCycleMap;
}PDM_PWM_Map_Struct;

typedef struct{
	uint16_t dutyCycle;

	//Used for configuration
	uint16_t pwmFrequency;
	PDM_OutType outputType;
	PDM_SoftStart softStart;

	//Used to set specific duty cycles
	uint16_t presetEnable[2];
	uint16_t presetInputs[2];
	uint16_t presetDutyCycle[2];

	PDM_PWM_ANN_Struct* pwmAnnStruct;
	PDM_PWM_Map_Struct* pwmMapStruct;
	PDM_PWM_SoftStart_Struct* softStartStruct;
}PWM_Control_Struct;
/*END STRUCT TYPEDEFS*/

/*BEGIN DECLARED VARIABLES*/
//AUTO GENERATED VARIABLES
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

//CAN
extern uint8_t canRxData[8];
extern uint8_t canTxData[8];
extern uint32_t canTxMailbox;
extern CAN_RxHeaderTypeDef canRxMessage;
extern CAN_TxHeaderTypeDef canTxMessage;
extern PDM_CAN_Config canConfig;

//CONFIGURATION
extern uint8_t
	usbConnectedFlag,
	usbVcpParameters[7];

//DATA
extern uint8_t dataFreqBuffer[30];
extern uint16_t
	dataBuffer[30],
	dataIdBuffer[30];
extern uint16_t adcBuffer[10];

//FLAGS
extern uint16_t flagDriverSafety;
extern PDM_Data_Read flagReading[2];

//OUTPUTS
extern uint8_t pwmPinStatus;
extern uint16_t inputLevels;
extern Output_Control_Struct outputStruct[16];
extern PWM_Control_Struct pwmOutStruct[4];

//TIMING
extern uint32_t
	accMsg10Hz,
	accMsg25Hz,
	accMsg50Hz,
	accMsg80Hz,
	accMsg100Hz,
	accUsbData,
	accOutputFuse[16];
/*END DECLARED VARIABLES*/

/*BEGIN CAN FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_CAN_Init(CAN_HandleTypeDef *hcan, PDM_CAN_Config* filter_struct);

HAL_StatusTypeDef PDM_PWM_CAN_Filter_Config(CAN_HandleTypeDef *hcan, PWM_Control_Struct *pwm_struct, uint8_t can_filter_bank);

HAL_StatusTypeDef PDM_CAN_Transmit_Data(CAN_HandleTypeDef* hcan, uint8_t data_freq);

void PDM_CAN_Process_Rx_Data();
/*END CAN FUNCTION PROTOTYPES*/

/*BEGIN CONFIGURATION FUNCTION PROTOTYPES*/
void PDM_Init(CAN_HandleTypeDef *hcan, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef PDM_PWM_Load_SoftStart_From_EEPROM(I2C_HandleTypeDef*hi2c, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number);

HAL_StatusTypeDef PDM_PWM_Map_Load_From_EEPROM(I2C_HandleTypeDef* hi2c, PWM_Control_Struct* pwm_struct, uint16_t mem_address);

void PDM_USB_Process(uint8_t *Data, uint16_t Size);

void PDM_USB_Transmit_Data();

void PDM_Hard_Code_Config();
/*END CONFIGURATION FUNCTION PROTOTYPES*/

/*BEGIN CONVERSION FUNCTION PROTOTYPES*/
HAL_StatusTypeDef PDM_Data_Conversion(TIM_HandleTypeDef *htim);
/*END CONVERSION FUNCTION PROTOTYPES*/

/*BEGIN HSD CONTROL FUNCTION PROTOTYPES*/
void PDM_Input_Process();

void PDM_Output_Process();
/*END HSD CONTROL FUNCTION PROTOTYPES*/

/*BEGIN HSD PWM CONTROL FUNCTION PROTOTYPES*/
void PDM_PWM_Init(CAN_HandleTypeDef *hcan, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number);

void PDM_PWM_Output_Process(PWM_Control_Struct *pwm_struct, uint8_t pwm_out_number, GPIO_PinState output_level);
/*END HSD PWM CONTROL FUNCTION PROTOTYPES*/

#endif /* INC_PDM_H_ */
