/*
 * pdm_variables.c
 *
 *  Created on: Mar 9, 2022
 *      Author: Rodolfo
 */

#include "pdm.h"

//CAN
uint8_t canRxData[8];
uint8_t canTxData[8];
uint32_t canTxMailbox;
CAN_RxHeaderTypeDef canRxMessage;
CAN_TxHeaderTypeDef canTxMessage;
PDM_CAN_Config canConfig;

//CONFIGURATION
uint8_t
	usbConnectedFlag,
	usbVcpParameters[7];

//DATA
uint8_t	dataFreqBuffer[30];
uint16_t
	dataBuffer[30],
	dataIdBuffer[30];
uint16_t adcBuffer[10];

//FLAGS
uint16_t flagDriverSafety;
PDM_Data_Read flagReading[2];

//OUTPUTS
uint8_t pwmPinStatus;
uint16_t inputLevels;
Output_Control_Struct outputStruct[16];
PWM_Control_Struct pwmOutStruct[4];

//TIMING
uint32_t
	accMsg10Hz,
	accMsg25Hz,
	accMsg50Hz,
	accMsg80Hz,
	accMsg100Hz,
	accUsbData,
	accOutputFuse[16];
