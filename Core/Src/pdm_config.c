/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"

static void Output_Init(PDM_Output_Ctrl_Struct* outStruct, PDM_PWM_Ctrl_Struct* pwmStruct);

/*BEGIN FUNCTIONS*/

void PDM_Output_Reset(PDM_Output_Ctrl_Struct* outStruct, PDM_PWM_Ctrl_Struct* pwmStruct)
{
	//Reset standard outputs
	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		outStruct[i].outputState = GPIO_PIN_RESET;
		if(outStruct[i].outputHardware == Output_GPIO)
			__PDM_OUT_SET_LEVEL(outStruct[i]);
	}

	//Reset PWM outputs
	for(uint8_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		pwmStruct[i].dutyCycle = PWM_MIN_DUTY_CYCLE;
		__PDM_PWM_SET_COMPARE(pwmStruct[i]);
	}

	return;
}

//Perform linear interpolation between two points
int16_t PDM_Linear_Interpolation(int32_t x, int16_t x0, int16_t x1, int16_t y0, int16_t y1)
{
	return ((((x - x0) * (y1 - y0)) / (x1 - x0)) + y0);
}

//Perform bilinear interpolation between four points by using three linear interpolations
int16_t PDM_Bilinear_Interpolation(int32_t x, int32_t y, int16_t x0, int16_t x1, int16_t y0, int16_t y1, int16_t z00, int16_t z01, int16_t z10, int16_t z11)
{
	int16_t mid[2];

	//Interpolate two midpoints
	mid[0] = PDM_Linear_Interpolation(x, x0, x1, z00, z01);
	mid[1] = PDM_Linear_Interpolation(x, x0, x1, z10, z11);

	//Get the interpolation between the two midpoints
	return PDM_Linear_Interpolation(y, y0, y1, mid[0], mid[1]);
}

/*END FUNCTIONS*/

//Initialize PDM
//Loads from EEPROM
//Initializes PWM
void PDM_Config_Thread(void* ThreadStruct)
{
	//Struct containing input and output structs and Queue handle
	PDM_OutSet_Thread_Struct* thrStr = (PDM_OutSet_Thread_Struct*) ThreadStruct;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Load output hardware data and set all levels to 0
	Output_Init(thrStr->outStruct, thrStr->pwmStruct);

	for(;;)
	{

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}

/*BEGIN STATIC FUNCTIONS*/

//Initialize and set outputs to zero Volts, should be executed only once
static void Output_Init(PDM_Output_Ctrl_Struct* outStruct, PDM_PWM_Ctrl_Struct* pwmStruct)
{
	uint16_t outPin[] = PDM_OUTPUT_PIN;
	GPIO_TypeDef* outGpio[] = PDM_OUTPUT_GPIO;
	PDM_Output_Hardware outHardware[] = PDM_OUT_TYPES;

	uint16_t pwmChannels[] = PDM_PWM_CHNS;
	TIM_HandleTypeDef* pwmTimers[] = PDM_PWM_TIMS;
	PDM_Output_Hardware pwmHardware[] = PDM_PWM_TYPES;

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		//Attribute pins and hardware type to each output struct
		outStruct[i].outputPin = outPin[i];
		outStruct[i].outputGPIO = outGpio[i];
		outStruct[i].outputHardware = outHardware[i];
	}

	for(uint8_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		//Attribute Timer, Channel and Hardware type to each PWM struct
		pwmStruct[i].timChannel = pwmChannels[i];
		pwmStruct[i].htim = pwmTimers[i];
		pwmStruct[i].outputHardware = pwmHardware[i];

		//Start PWM output
		if(pwmHardware[i] == Output_PWM)
			HAL_TIM_PWM_Start(pwmTimers[i], pwmChannels[i]);

		else
			HAL_TIMEx_PWMN_Start(pwmTimers[i], pwmChannels[i]);
	}

	PDM_Output_Reset(outStruct, pwmStruct);

	return;
}

/*END STATIC FUNCTIONS*/
