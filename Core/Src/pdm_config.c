/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "pdm_specific.h"

static void Output_Reset_State(PDM_Output_Ctrl_Struct* outStruct);
static void Output_Init(PDM_Output_Ctrl_Struct* outStruct, TIM_HandleTypeDef** htimBuffer, uint16_t* timChannelBuffer);
static uint8_t Can_Tx_Threads_Init(Data_Freq* freqBuffer, PDM_CanTxMsg_Thread_Struct* threadStruct, osThreadId_t* threadIdBuffer, osThreadAttr_t* threadAttrBuffer, osMessageQId* freqQueue);

//Initialize PDM
//Loads from EEPROM
//Initializes PWM
void PDM_Config_Thread(void* ThreadStruct)
{

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	for(;;)
	{

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

static void Output_Reset_State(PDM_Output_Ctrl_Struct* outStruct)
{
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		if(outStruct->outputHardware == Output_GPIO)	//Reset state if there is no PWM for the pin
			HAL_GPIO_WritePin(outStruct[i]->outputGPIO, outStruct[i]->outputPin, GPIO_PIN_RESET);

		else	//Reset output state by zeroing the capture compare register
			__HAL_TIM_SET_COMPARE(outStruct[i]->pwmStruct->htim, outStruct[i]->pwmStruct->timChannel, 0);
	}

	return;
}

//Initialize and set outputs to zero Volts, should be executed only once
static void Output_Init(PDM_Output_Ctrl_Struct* outStruct, TIM_HandleTypeDef** htimBuffer, uint16_t* timChannelBuffer)
{
	uint8_t acc = 0;	//Tracks position of timer buffers
	uint16_t outputPinBuffer[] = PDM_OUTPUT_PIN;
	GPIO_TypeDef outputGpioBuffer[] = PDM_OUTPUT_GPIO;
	PDM_Output_Hardware outputHardwareBuffer[] = PDM_OUT_TYPES;

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		//Attribute pins and hardware type to each output struct
		outStruct[i]->outputPin = outputPinBuffer[i];
		outStruct[i]->outputGPIO = outputGpioBuffer[i];
		outStruct[i]->outputHardware = outputHardwareBuffer[i];

		if(outputHardwareBuffer[i] == Output_GPIO)	//Reset output of it's standard GPIO
			HAL_GPIO_WritePin(outputGpioBuffer[i], outputPinBuffer[i], GPIO_PIN_RESET);

		else	//Reset output if it's PWM capable
		{
			//Attribute timer peripheral and channel to output PWM struct
			outStruct[i]->pwmStruct->htim = &htimBuffer[acc];
			outStruct[i]->pwmStruct->timChannel = timChannelBuffer[acc];

			if(outputHardwareBuffer[i] == Output_PWM)
				HAL_TIM_PWM_Start(htimBuffer[acc], timChannelBuffer[acc]);

			if(outputHardwareBuffer[i] == Output_PWMN)
				HAL_TIMEx_PWMN_Start(htimBuffer[acc], timChannelBuffer[acc]);

			__HAL_TIM_SET_COMPARE(htimBuffer[acc], timChannelBuffer[acc], 0);

			acc++;	//Increment timer and channel buffer position
		}
	}

	return;
}

//Create and start data Transmission Threads
static uint8_t Can_Tx_Threads_Init(Data_Freq* freqBuffer, PDM_CanTxMsg_Thread_Struct* threadStruct, osThreadId_t* threadIdBuffer, osThreadAttr_t* threadAttrBuffer, osMessageQId* freqQueue)
{
	uint8_t retVal = 0;

	return retVal;	//Return number of used Threads
}

static void EEPROM_Read_Pt1()
{
	return;
}

static void EEPROM_Read_Pt2()
{
	return;
}
