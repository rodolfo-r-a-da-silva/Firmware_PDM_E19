/*
 * pdm_driver_control.h
 *
 *  Created on: Oct 18, 2021
 *      Author: Rodolfo
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_

#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "usbd_def.h"

/*BEGIN DEFINES*/
//FREERTOS QUEUE
#define RTOS_QUEUE_CANCFG_SIZE		sizeof(PDM_Data_Queue_Struct)
#define RTOS_QUEUE_CANCFG_COUNT		5
#define RTOS_QUEUE_CANCHN_SIZE		sizeof(PDM_Data_Queue_Struct)
#define RTOS_QUEUE_CANCHN_COUNT		5
#define RTOS_QUEUE_CANTXFREQ_SIZE	1
#define RTOS_QUEUE_CANTXFREQ_COUNT	1
#define RTOS_QUEUE_OUT_SIZE			sizeof(uint8_t)
#define RTOS_QUEUE_OUT_COUNT		5
#define RTOS_QUEUE_USB_SIZE			34
#define RTOS_QUEUE_USB_COUNT		1

//FREERTOS SEMAPHORES
#define RTOS_SEMAPHORE_CANCFG_MAX	1
#define RTOS_SEMAPHORE_CANCFG_INIT	0
#define RTOS_SEMAPHORE_CANCHN_MAX	1
#define RTOS_SEMAPHORE_CANCHN_INIT	0
#define RTOS_SEMAPHORE_CANRX_MAX	1
#define RTOS_SEMAPHORE_CANRX_INIT	0
#define RTOS_SEMAPHORE_READ_MAX		1
#define RTOS_SEMAPHORE_READ_INIT	1

//FREERTOS THREADS
#define RTOS_THREAD_MEMSZ_CANRX	128
#define RTOS_THREAD_MEMSZ_CANTX	128
#define RTOS_THREAD_MEMSZ_CFG	128
#define RTOS_THREAD_MEMSZ_OUT	128
#define RTOS_THREAD_MEMSZ_PRC	128
#define RTOS_THREAD_MEMSZ_READ	128
#define RTOS_THREAD_MEMSZ_USBRX	128
#define RTOS_THREAD_MEMSZ_USBTX	128
#define RTOS_THREAD_PRIO_CANRX	osPriorityAboveNormal
#define RTOS_THREAD_PRIO_CANTX	osPriorityLow
#define RTOS_THREAD_PRIO_CFG	osPriorityHigh1
#define RTOS_THREAD_PRIO_OUT	osPriorityAboveNormal1
#define RTOS_THREAD_PRIO_PRC	osPriorityAboveNormal2
#define RTOS_THREAD_PRIO_READ	osPriorityHigh
#define RTOS_THREAD_PRIO_USBRX	osPriorityHigh2
#define RTOS_THREAD_PRIO_USBTX	osPriorityLow

//CAN
#define CAN_PDM_ID				0x500
#define CAN_CONFIG_FILTER		0x00
#define CAN_NBR_OF_FILTERS		14
#define CAN_DATA_FILTER_MASK	0xfffffff4
#define CAN_ID_CHANNEL			0x1E35C000
#define CAN_ID_CURR1			0x00
#define CAN_ID_CURR2			0x00
#define CAN_ID_CURR3			0x00
#define CAN_ID_CURR4			0x00
#define CAN_ID_TEMP1			0x00
#define CAN_ID_TEMP2			0x00
#define CAN_ID_GENERAL			0x00
#define CAN_ID_PINS				0x00
#define CAN_ID_PWM				0x00
#define CAN_CONFIG_RECEIVED		0x01
#define CAN_DATA_RECEIVED		0x02
#define CAN_QUEUE_SIZE			14
#define OUTPUT_FUSE_FREQ		25

//CHANNELS AND FUNCTIONS
#define NBR_OF_ANALOG_CHANNELS	27
#define CHANNEL_ANALOG_OFFSET	0
#define CHANNEL_ANALOG_FINISH	CHANNEL_ANALOG_OFFSET+NBR_OF_ANALOG_CHANNELS
#define CHANNEL_CAN_OFFSET		CHANNEL_ANALOG_FINISH
#define CHANNEL_TEMP_MCU		CHANNEL_ANALOG_OFFSET+Data_TempMCU

#define NBR_OF_INPUT_FUNCTIONS	NBR_OF_INPUTS
#define FUNCTION_INPUT_OFFSET	0
#define FUNCTION_INPUT_FINISH	FUNCTION_INPUT_OFFSET+NBR_OF_INPUT_FUNCTIONS
#define FUNCTION_FUSE_OFFSET	FUNCTION_INPUT_FINISH
#define FUNCTION_FUSE_FINISH	FUNCTION_FUSE_OFFSET+NBR_OF_OUTPUTS
#define FUNCTION_CUSTOM_OFFSET	FUNCTION_FUSE_FINISH

#define NBR_OF_DATA_RESULTS	2

#define NBR_OF_FUNC_TIMES	2
#define NBR_OF_FUNC_CONSTS	6
#define NBR_OF_FUNC_INPUTS	4
#define NBR_OF_FUNC_RESULTS	3

#define IN_USE_NONE			0x00
#define IN_USE_FUNCTION		0x01
#define IN_USE_OUTPUT		0x02
#define IN_USE_PWM			0x04

#define PROCESS_NONE		IN_USE_NONE
#define PROCESS_FUNCTION	IN_USE_FUNCTION
#define PROCESS_OUTPUT		IN_USE_OUTPUT
#define PROCESS_PWM			IN_USE_PWM
#define PROCESS_FUSE		0x08

//DATA
#define NBR_OF_ADC_CHANNELS		10
#define NBR_OF_DATA_CHANNELS	34
#define NBR_OF_DATA_MSGS		10
#define NBR_OF_FREQ_OPTS		Data_Freq_Max-1
#define NBR_OF_INPUTS			16
#define NBR_OF_OUTPUTS			16
#define NBR_OF_PWM_OUTPUTS		4
#define NBR_OF_DELAY_TIMES		5

//PWM OUTPUTS
#define PWM_FREQ_100HZ		899
#define PWM_FREQ_250HZ		359
#define PWM_FREQ_500HZ		179
#define PWM_FREQ_750HZ		119
#define PWM_FREQ_1000HZ		89
#define PWM_FREQ_2500HZ		35
#define PWM_FREQ_5000HZ		17
#define PWM_FREQ_7500HZ		11
#define PWM_FREQ_10000HZ	8
#define PWM_FREQ_15000HZ	5

#define PWM_MIN_DUTY_CYCLE	0
#define PWM_MAX_DUTY_CYCLE	1000
#define PWM_NBR_OF_PRESETS	4
#define PWM_TABLE_MAX_SIZE	16

//FUSES
#define FUSE_RETRY_INF	255

//FREQUENCIES
#define DATA_DISABLED	0
#define DATA_FREQ_1HZ	1000
#define DATA_FREQ_2HZ	500
#define DATA_FREQ_5HZ	200
#define DATA_FREQ_10HZ	100
#define DATA_FREQ_20HZ	50
#define DATA_FREQ_25HZ	40
#define DATA_FREQ_50HZ	20
#define DATA_FREQ_100HZ	10
#define DATA_FREQ_200HZ	5
#define DATA_FREQ_250HZ	4
#define DATA_FREQ_500HZ	2

//TIMEOUTS
#define CAN_THREAD_TIMEOUT		pdMS_TO_TICKS(2)
#define PROCESS_THREAD_TIMEOUT	pdMS_TO_TICKS(2)

//DATA CONVERSION CONSTANTS
//ADC CONSTANTS
#define ADC_12_BIT_DIVISION			4095
#define ADC_THRESHOLD_HIGH			4000
#define ADC_THRESHOLD_LOW			90

//MCU TEMPERATURE
#define ADC_MCU_TEMP_VAL0			250
#define ADC_MCU_TEMP_VOLT0			760
#define ADC_MCU_TEMP_VAL1			1010
#define ADC_MCU_TEMP_VOLT1			350

//us
#define READING_DELAY_CURR			250
#define READING_DELAY_TEMP			500
#define READING_DELAY_VOLT			250

#define EEPROM_I2C_ADDRESS			0xA0
#define EEPROM_TIMEOUT				10
/*END DEFINES*/

/*BEGIN MACROS*/
#define __BUFFER_TO_FREQ(__BUFFER__, __PER__)	\
	switch(__BUFFER__)							\
	{											\
		case Data_Freq_1Hz:						\
			(__PER__) = DATA_FREQ_1HZ;			\
			break;								\
		case Data_Freq_2Hz:						\
			(__PER__) = DATA_FREQ_2HZ;			\
			break;								\
		case Data_Freq_5Hz:						\
			(__PER__) = DATA_FREQ_5HZ;			\
			break;								\
		case Data_Freq_10Hz:					\
			(__PER__) = DATA_FREQ_10HZ;			\
			break;								\
		case Data_Freq_20Hz:					\
			(__PER__) = DATA_FREQ_20HZ;			\
			break;								\
		case Data_Freq_25Hz:					\
			(__PER__) = DATA_FREQ_25HZ;			\
			break;								\
		case Data_Freq_50Hz:					\
			(__PER__) = DATA_FREQ_50HZ;			\
			break;								\
		case Data_Freq_100Hz:					\
			(__PER__) = DATA_FREQ_100HZ;		\
			break;								\
		case Data_Freq_200Hz:					\
			(__PER__) = DATA_FREQ_200HZ;		\
			break;								\
		case Data_Freq_250Hz:					\
			(__PER__) = DATA_FREQ_250HZ;		\
			break;								\
		case Data_Freq_500Hz:					\
			(__PER__) = DATA_FREQ_500HZ;		\
			break;								\
		default:								\
			(__PER__) = DATA_DISABLED;			\
			break;								\
	}

#define __FREQ_TO_BUFFER(__BUFFER__, __PER__)	\
	switch(__PER__)								\
	{											\
		case DATA_FREQ_1HZ:						\
			(__BUFFER__) = Data_Freq_1Hz;		\
			break;								\
		case DATA_FREQ_2HZ:						\
			(__BUFFER__) = Data_Freq_2Hz;		\
			break;								\
		case DATA_FREQ_5HZ:						\
			(__BUFFER__) = Data_Freq_5Hz;		\
			break;								\
		case DATA_FREQ_10HZ:					\
			(__BUFFER__) = Data_Freq_10Hz;		\
			break;								\
		case DATA_FREQ_20HZ:					\
			(__BUFFER__) = Data_Freq_20Hz;		\
			break;								\
		case DATA_FREQ_25HZ:					\
			(__BUFFER__) = Data_Freq_25Hz;		\
			break;								\
		case DATA_FREQ_50HZ:					\
			(__BUFFER__) = Data_Freq_50Hz;		\
			break;								\
		case DATA_FREQ_100HZ:					\
			(__BUFFER__) = Data_Freq_100Hz;		\
			break;								\
		case DATA_FREQ_200HZ:					\
			(__BUFFER__) = Data_Freq_200Hz;		\
			break;								\
		case DATA_FREQ_250HZ:					\
			(__BUFFER__) = Data_Freq_250Hz;		\
			break;								\
		case DATA_FREQ_500HZ:					\
			(__BUFFER__) = Data_Freq_500Hz;		\
			break;								\
		default:								\
			(__BUFFER__) = Data_Disabled;		\
			break;								\
	}

#define __F32_TO_BUFFER(__HIGH__, __LOW__, __FLOAT__)								\
		(__HIGH__) = (((__FLOAT__) >> 24) & 0xc0) | (((__FLOAT__) >> 21) & 0x3f);	\
		(__LOW__) = (((__FLOAT__) >> 13) & 0xff);

#define __BUFFER_TO_F32(__HIGH__, __LOW__, __UNION__)												\
		(__UNION__) = (((__HIGH__) & 0xc0) << 24) | (((__HIGH__) & 0x3f) << 21) | ((__LOW__) <<13);	\
		if(((__HIGH__) & 0x40) == 0x00)	(__UNION__) |= 0x38000000;

#define __PDM_READINGS_FUSE	thrdStr->outStruct[i]->fuseStruct

#define __PDM_READINGS_TIMER(__HTIM__, __HSEMAPHORE__)	\
		if(__HTIM__->Instance == TIM6)					\
			osSemaphoreRelease(__HSEMAPHORE__);

#define __PDM_FUNCTION_EDGE_CONDITION(__FUNC_STRUCT__, __INPUT__)														\
		(((__FUNC_STRUCT__)->inEdge == Edge_Both)																		\
			|| (((__FUNC_STRUCT__)->inEdge == Edge_Falling) && (*(__FUNC_STRUCT__)->inputs[__INPUT__] == Result_Low))	\
			|| (((__FUNC_STRUCT__)->inEdge == Edge_Rising) && (*(__FUNC_STRUCT__)->inputs[__INPUT__] == Result_High)))

#define __PDM_LINEAR_INTERPOLATION(__X__, __X0__, __X1__, __Y0__, __Y1__)						\
		(((((__X__) - (__X0__)) * ((__Y1__) - (__Y0__))) / ((__X1__) - (__X0__))) + (__Y0__))

#define __PDM_BILINEAR_INTERPOLATION(__X__, __Y__, __X0__, __X1__, __Y0__, __Y1__, __Z00__, __Z01__, __Z10__, __Z11__)	\
		__PDM_LINEAR_INTERPOLATION((__Y__), (__Y0__), (__Y1__),															\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z00__), (__Z01__))),						\
				(__PDM_LINEAR_INTERPOLATION((__X__), (__X0__), (__X1__), (__Z10__), (__Z11__))))

#define __PDM_OUT_SET_LEVEL(__OUT_STRUCT__)				\
		HAL_GPIO_WritePin((__OUT_STRUCT__)->outputGPIO,	\
		(__OUT_STRUCT__)->outputPin,					\
		(__OUT_STRUCT__)->outputState)

#define __PDM_PWM_SET_COMPARE(__OUT_STRUCT__)						\
		__HAL_TIM_SET_COMPARE((__OUT_STRUCT__)->pwmStruct->htim,	\
		(__OUT_STRUCT__)->pwmStruct->timChannel,					\
		(__OUT_STRUCT__)->pwmStruct->dutyCycle)

/*END MACROS*/

/*BEGIN ENUMS TYPEDEFS*/
typedef enum{
	CAN_Disable	= 0x00,
	CAN_125kbps,
	CAN_250kbps,
	CAN_500kbps,
	CAN_1000kbps
}PDM_CAN_BaudRate;

typedef enum{
	CAN_Data_Timeout,
	CAN_Data_Refresh
}PDM_CAN_Data_Timeout;

typedef enum{
	CAN_Msg_Channels,
	CAN_Msg_Curr1,
	CAN_Msg_Curr2,
	CAN_Msg_Curr3,
	CAN_Msg_Curr4,
	CAN_Msg_Temp1,
	CAN_Msg_Temp2,
	CAN_Msg_General,
	CAN_Msg_Pins,
	CAN_Msg_PWM
}PDM_CAN_TxMsgType;

typedef enum{
	Data_Disabled	= 0x00,
	Data_Freq_1Hz,
	Data_Freq_2Hz,
	Data_Freq_5Hz,
	Data_Freq_10Hz,
	Data_Freq_20Hz,
	Data_Freq_25Hz,
	Data_Freq_50Hz,
	Data_Freq_100Hz,
	Data_Freq_200Hz,
	Data_Freq_250Hz,
	Data_Freq_500Hz,
	Data_Freq_Max
}Data_Freq;

typedef enum{
	Data_Type_Init,
	Data_Type_Current0,
	Data_Type_Current1,
	Data_Type_Temperature,
	Data_Type_Voltage
}PDM_Data_Type;

typedef enum{
	Data_Curr1 = 0,
	Data_Curr3,
	Data_Curr5,
	Data_Curr7,
	Data_Curr9,
	Data_Curr11,
	Data_Curr13,
	Data_Curr15,
	Data_Curr2,
	Data_Curr4,
	Data_Curr6,
	Data_Curr8,
	Data_Curr10,
	Data_Curr12,
	Data_Curr14,
	Data_Curr16,
	Data_CurrTotal,
	Data_Temp1,
	Data_Temp2,
	Data_Temp3,
	Data_Temp4,
	Data_Temp5,
	Data_Temp6,
	Data_Temp7,
	Data_Temp8,
	Data_TempMCU,
	Data_Volt,
	Data_Input,
	Data_Output,
	Data_Fuse,
	Data_PWM1,
	Data_PWM2,
	Data_PWM3,
	Data_PWM4
}PDM_Data_Channels;

typedef enum{
	Alignment_Normal,
	Alignment_Swap
}PDM_Data_Alignment;

typedef enum{
	Cast_Uint8,
	Cast_Int8,
	Cast_Uint16,
	Cast_Int16
}PDM_Data_Cast;

typedef enum{
	Data_ADC,
	Data_CAN_Channel,
	Data_CAN_Fixed,
	Data_Input_Pins
}PDM_Data_Source;

typedef enum{
	Interrupt_CAN,
	Interrupt_Function,
	Interrupt_Fuse,
	Interrupt_Gpio,
	Interrupt_Timer
}PDM_Interrupt_Source;

typedef enum{
	Output_GPIO,
	Output_PWM,
	Output_PWMN
}PDM_Output_Hardware;

typedef enum{
	Output_Disabled,
	Output_Enabled
}PDM_Output_Enabled;

typedef enum{
	OutType_Error,
	OutType_Standard,
	OutType_Pwm_Preset,
	OutType_Pwm_Map,
	OutType_Pwm_Ann
}PDM_PWM_Output_Type;

typedef enum{
	Input_Channel,
	Input_Const,
	Input_Function
}PDM_Input_Type;

typedef enum{
	Data_Keep,
	Data_Reset
}PDM_CAN_Data_Keep;

typedef enum{
	Data_Current,
	Data_Previous
}PDM_Channel_DataTime;

typedef enum{
	Time_False,
	Time_True
}PDM_Function_Time;

typedef enum{
	Result_False,
	Result_True
}PDM_Function_Result;

typedef enum{
	Edge_None,
	Edge_Falling,
	Edge_Rising,
	Edge_Both
}PDM_Function_Edge;

typedef enum{
	Result_Current,
	Result_Next,
	Result_Previous
}PDM_Function_ResultTime;

typedef enum{
	Function_Disabled,
	Function_NOT,
	Function_AND,
	Function_OR,
	Function_XOR,
	Function_BitAND,
	Function_Equals,
	Function_Less,
	Function_More,
	Function_Hysteresis,
	Function_Blink,
	Function_Pulse,
	Function_SetReset,
	Function_Toggle,
	Function_Counter
}PDM_Function_Type;

typedef enum{
	FuncIn_Current1,
	FuncIn_Previous1,
	FuncIn_Current2,
	FuncIn_Previous2,
	FuncIn_Current3,
	FuncIn_Previous3,
	FuncIn_Current4,
	FuncIn_Previous4
}PDM_Function_InputChange;

typedef enum{
	FuncConst_Low,
	FuncConst_High,
	FuncConst_Ovrr1,
	FuncConst_Ovrr2,
	FuncConst_Dec,
	FuncConst_Inc
}PDM_Function_Const;

typedef enum{
	Fuse_Disabled,
	Fuse_Enabled
}PDM_Fuse_Enabled;

typedef enum{
	Fuse_Time_First,
	Fuse_Time_Open,
	Fuse_Time_Close
}PDM_Fuse_Time;

typedef enum{
	Fuse_Closed,
	Fuse_Wait,
	Fuse_Open
}PDM_Fuse_Status;

typedef enum{
	SoftStart_Disabled,
	SoftStart_Enabled
}PDM_SoftStart_Enabled;

typedef enum{
	ANN_In_Can,
	ANN_In_Neuron,
	ANN_In_Local
}PDM_ANN_InType;
/*END ENUMS TYPEDEFS*/

/*BEGIN UNION TYPEDEFS*/
typedef union{
	uint32_t u32;
	float f32;
}Int32_To_F32;
/*END UNION TYPEDEFS*/

/*BEGIN STRUCT TYPEDEFS*/
typedef struct{
	CAN_HandleTypeDef* hcan;	//CAN peripheral handle pointer
	PDM_CAN_BaudRate baudRate;	//CAN bus speed

	//Bus filtering
	uint16_t enabledFilters;				//Indicate which filters from the bank should be used
	uint32_t filters[CAN_NBR_OF_FILTERS];	//Filters to be used for data reception
}PDM_CAN_Config_Struct;

typedef struct{
	int32_t data[NBR_OF_DATA_RESULTS];	//Channel value used for condition calculation

	//Data filtering and conversion
	uint8_t dataFilterNbr;	//Indicate which filter from local filters or CAN filter bank is used
	uint32_t dataFilter;	//Filter corresponding to the data's frame
	uint16_t position;		//Data field byte offset for Local and Fixed CAN data or CAN/Local data channel ID
	PDM_Data_Source source;	//Specify if data is read from ADC, Inputs or is received via CAN bus

	//Byte alignment, masking and casting
	uint16_t mask;					//AND mask for unsigned data
	PDM_Data_Alignment alignment;	//Byte alignment for 16 bit data, will always be normal for local data
	PDM_Data_Cast cast;				//Type of data to be cast into struct variable

	uint8_t inUse;	//Indicate if channel is used by functions or PWM outputs

	//Timeout info for CAN data
	uint16_t defaultVal;				//Value cast into variable if timeout is reached
	uint8_t timeout;					//Timeout in milliseconds/100
	osTimerId_t timer;					//Timer for timeout processing
	PDM_CAN_Data_Timeout timeoutFlag;	//Indicates if data was timed out
	PDM_CAN_Data_Keep keep;				//Indicate if data should be kept as last value if timeout happens
}PDM_Data_Channel_Struct;

typedef struct{
	//Output result
	int32_t result[NBR_OF_FUNC_RESULTS];	//Indicate next, current and previous function output state
	uint8_t invert;							//Set if result should be inverted
	uint8_t inUse;							//Indicate if function is used by other Functions or Outputs

	//Input processing
	int32_t consts[NBR_OF_FUNC_CONSTS];				//Constants used in specific Function types
	int32_t* inputs[NBR_OF_FUNC_INPUTS*2];			//Array of pointers to each function result used as inputs
	PDM_Function_Edge inEdge[NBR_OF_FUNC_INPUTS];	//Array indicating which edge the inputs must be to cause a change of state

	//Input configuration
	uint8_t nbrOfInputs;							//Amount of used inputs
	uint16_t inputNbr[NBR_OF_FUNC_INPUTS];			//Array indicating the position of input in it's array
	PDM_Data_Cast constCast[NBR_OF_FUNC_CONSTS];	//Indicate which type of variable a constant should be cast to
	PDM_Input_Type inputTypes[NBR_OF_FUNC_INPUTS];	//Array storing each input type

	//Function configuration
	uint16_t funcDelay[NBR_OF_FUNC_TIMES];	//Delays for result to change states or time for result state (Blink/Pulse)
	osTimerId_t funcTimer;					//Timer for change of result delay
	osMessageQueueId_t processQueue;		//Queue to send interrupt source to process Thread
	PDM_Function_Type type;					//Indicate which type of operation the function performs
}PDM_Function_Struct;

typedef struct{
	uint8_t data[8];		//Data from data frame
	uint8_t length;			//Number of data bytes
	uint32_t id;			//Frame unique identification
	PDM_Data_Source source;	//Indication if frame is from local readings or CAN bus
}PDM_Data_Queue_Struct;

typedef struct{
	uint16_t dutyCycles;		//Amount of duty cycles to get from 0 to 100%
	uint16_t turnOnTime;		//Time (ms) it takes to get from 0 to 100%
	uint16_t* dutyCycleBuffer;	//Buffer to feed DMA channel with duty cycle values
	uint32_t slope;				//Slope to calculate dutyCycleBuffer
}PDM_PWM_SoftStart_Struct;

typedef struct{
	int16_t output;
	uint16_t* inputId;
	float* weigths;
	PDM_ANN_InType* inputType;
}PDM_PWM_ANN_Struct;

typedef struct{
	uint8_t mapLengths[2];
	int32_t commandVar[2];
	//[0][i]: column; [1][j]: line
	int16_t* commandVarStep[2];
	//[column][line] or [x][y]
	uint16_t** dutyCycleMap;
}PDM_PWM_Map_Struct;

typedef struct{
	uint16_t dutyCycle;	//Output PWM Duty Cycle

	//Used for configuration
	uint16_t pwmFrequency;
	PDM_PWM_Output_Type type;
	PDM_SoftStart_Enabled softStart;

	//Hardware Timer peripheral information
	uint16_t timChannel;
	TIM_HandleTypeDef* htim;

	//Used to set specific duty cycles
	uint16_t presetDutyCycle[PWM_NBR_OF_PRESETS];
	int32_t* presetInputs[PWM_NBR_OF_PRESETS];
	PDM_Input_Type presetInType[PWM_NBR_OF_PRESETS];

	//Struct pointers for additional functionalities
	PDM_PWM_ANN_Struct* annStruct;
	PDM_PWM_Map_Struct* mapStruct;
	PDM_PWM_SoftStart_Struct* softStartStruct;
}PDM_PWM_Ctrl_Struct;

typedef struct{
	uint8_t retry;		//Maximum number of closing attempts
	uint8_t retryCount;	//Number of times attempted to close the fuse
	int16_t maxCurrent;	//Maximum current for fuse opening

	//Maximum overcurrent
	uint16_t timeout[3];	//PDM_Fuse_Time
	PDM_Fuse_Status status;
	osTimerId_t osTimer;

	//Output and Function processing
	osMessageQueueId_t* outputQueue;	//Queue handle for output process Thread
	osMessageQueueId_t* processQueue;	//Queue handle for processing Thread
	PDM_Function_Struct* funcStruct;	//Fuse Function for operations processing
}PDM_Output_Fuse_Struct;

typedef struct{
	//Output activation
	int32_t* inputFunc;				//Function result input
	GPIO_PinState outputState;		//Indicate if output pin should be high or low
	PDM_Fuse_Enabled fuseEnable;	//Specify if fuse is enabled for this output

	//Output hardware info (loaded using defines)
	uint16_t outputPin;
	GPIO_TypeDef* outputGPIO;
	PDM_Output_Hardware outputHardware;

	//Mixed types of activation
	PDM_Output_Fuse_Struct* fuseStruct;	//Pointer to fuse struct
	PDM_PWM_Ctrl_Struct* pwmStruct;		//Pointer to PWM struct
}PDM_Output_Ctrl_Struct;

typedef struct{
	uint16_t pin[NBR_OF_INPUTS];
	GPIO_TypeDef gpio[NBR_OF_INPUTS];
}PDM_Input_Struct;
/*END STRUCT TYPEDEFS*/

/*START THREAD STRUCT TYPEDEFS*/
typedef struct{
	PDM_CAN_Config_Struct* canConfig;	//Contains CAN handle, data and filter information

	osMessageQueueId_t* cfgQueueHandle;
	osMessageQueueId_t* chnQueueHandle;
	osSemaphoreId_t* semaphoreHandle;
	osSemaphoreId_t* cfgSemaphoreHandle;
	osSemaphoreId_t* chnSemaphoreHandle;
}PDM_CanRxMsg_Thread_Struct;

typedef struct{
	uint16_t* dataBuffer;
	uint16_t* dataIdBuffer;
	Data_Freq dataFreq;
	Data_Freq* dataFreqBuffer;

	CAN_HandleTypeDef* hcan;
	PDM_CAN_TxMsgType txMsgType;
	osMutexId_t* canMutexHandle;
	osMutexId_t* dataMutexHandle;
}PDM_CanTxMsg_Thread_Struct;

typedef struct{
	uint8_t nbrOfChannels;
	PDM_Data_Channel_Struct* channels;
	PDM_Function_Struct* functions;

	PDM_Output_Ctrl_Struct* outStruct;
	PDM_Input_Struct* inStruct;

	int16_t* dataBuffer;
	uint16_t* adcBuffer;
	TIM_HandleTypeDef* htim;
	osMessageQueueId_t* intQueueHandle;
	osMutexId_t* dataMutexHandle;
}PDM_Process_Thread_Struct;

typedef struct{
	uint16_t* inputPins;
	GPIO_TypeDef* inputGPIOs;
	PDM_Output_Ctrl_Struct* outStruct;
	osMessageQueueId_t* outQueueHandle;
}PDM_OutSet_Thread_Struct;

typedef struct{
	int16_t* dataBuffer;
	uint16_t* adcBuffer;
	TIM_HandleTypeDef* htim;
	osMessageQueueId_t* queueHandle;
	osMutexId_t* dataMutexHandle;
	PDM_Output_Ctrl_Struct* outStruct;
}PDM_Readings_Thread_Struct;

typedef struct{
	USBD_HandleTypeDef* husb;
	osMessageQueueId_t* freqQueueHandle;
}PDM_UsbTxMsg_Thread_Struct;

typedef struct{
	osSemaphoreId_t* canRxSemaphoreId;
	osSemaphoreId_t* outputSemaphoreId;
	osSemaphoreId_t* readingSemaphoreId;

	ADC_HandleTypeDef** hadc;
	CAN_HandleTypeDef** hcan;
	TIM_HandleTypeDef** htim;
}PDM_Config_Thread_Struct;
/*END THREAD STRUCT TYPEDEFS*/

/*BEGIN DECLARED VARIABLES*/

//RTOS QUEUES
extern osMessageQueueId_t processQueueHandle;

//RTOS SEMAPHORES
extern osSemaphoreId_t canRxSemaphoreHandle;

//CONFIGURATION
extern uint8_t usbConnectedFlag;
extern uint8_t usbVcpParameters[7];
/*END DECLARED VARIABLES*/

/*BEGIN CAN PROTOTYPES*/
//FUNCTIONS
HAL_StatusTypeDef PDM_CAN_Init(PDM_CAN_Config_Struct* fltr_str);

//THREADS
void PDM_CAN_Thread_Transmit_Data(void* threadStruct);
/*END CAN PROTOTYPES*/

/*BEGIN CONFIGURATION FUNCTION PROTOTYPES*/
void PDM_Config_Thread(void* threadStruct);
/*END CONFIGURATION FUNCTION PROTOTYPES*/

/*BEGIN CONVERSION PROTOTYPES*/
//THREADS
void PDM_Readings_Thread(void* threadStruct);
/*END CONVERSION PROTOTYPES*/

/*BEGIN HSD CONTROL FUNCTION PROTOTYPES*/
//THREADS
void PDM_Output_Thread(void* threadStruct);
/*END HSD CONTROL FUNCTION PROTOTYPES*/

/*BEGIN USB PROTOTYPES*/
//THREADS
void PDM_USB_Thread_Transmit_Data(void* threadStruct);
/*END USB PROTOTYPES*/

#endif /* INC_PDM_H_ */
