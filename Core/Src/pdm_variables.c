/*
 * pdm_variables.c
 *
 *  Created on: Mar 9, 2022
 *      Author: Rodolfo
 */

#include "pdm.h"

//CAN
uint8_t
	CAN_Baud_Rate,
	CAN_Rx_Data[8],
	CAN_Tx_Data[8];
uint32_t pTxMailbox;
CAN_RxHeaderTypeDef CAN_Rx_Message;
CAN_TxHeaderTypeDef CAN_Tx_Message;

//CONFIGURATION
uint8_t
	USB_Connected_Flag,
	USB_VCP_Parameters[7];

//CURRENT MONITORING
uint16_t
	Driver_Overcurrent_Flag,
	Driver_Safety_Flag;

//DATA
uint8_t
	Data_Conversion,
	Data_Freq_Buffer[30];
uint16_t
	Data_Buffer[30],
	Data_ID_Buffer[30];
uint16_t ADC_BUFFER[10];

//OUTPUTS
uint8_t PWM_Pin_Status;
uint16_t Input_Pin_Levels;
Output_Control_Struct Output_Pin[16];
PWM_Control_Struct PWM_Pins[4];

//TIMING
int32_t Accumulator_Delay;

uint32_t
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
