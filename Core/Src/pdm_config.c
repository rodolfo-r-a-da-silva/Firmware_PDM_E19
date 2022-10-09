/*
 * pdm_config.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Rodolfo
 */

#include "pdm.h"
#include "usbd_cdc_if.h"

static void Output_Reset_State();
static HAL_StatusTypeDef Output_Cfg_Load_From_EEPROM(I2C_HandleTypeDef* hi2c);
static HAL_StatusTypeDef Output_Cfg_Save_To_EEPROM(I2C_HandleTypeDef* hi2c);
static HAL_StatusTypeDef PWM_Output_Cfg_Load_From_EEPROM(I2C_HandleTypeDef* hi2c);
static HAL_StatusTypeDef PWM_Output_Cfg_Save_To_EEPROM(I2C_HandleTypeDef* hi2c);

//Initialize PDM
//Loads from EEPROM
//Initializes PWM
void PDM_Init(CAN_HandleTypeDef *hcan, I2C_HandleTypeDef *hi2c)
{
	//Sets all outputs to zero
	Output_Reset_State();

	//Weak function for data overwriting during initialization
	PDM_Hard_Code_Config();

	//Loads both normal output and PWM output parameters
//	Output_Cfg_Load_From_EEPROM(hi2c);
//	PWM_Output_Cfg_Load_From_EEPROM(hi2c);

	//Initializes each PWM able output
	PDM_PWM_Init(hcan, &pwmOutStruct[0], 0);
	PDM_PWM_Init(hcan, &pwmOutStruct[1], 1);
	PDM_PWM_Init(hcan, &pwmOutStruct[2], 2);
	PDM_PWM_Init(hcan, &pwmOutStruct[3], 3);

	//Checks input pin levels
	PDM_Input_Process();

	//Initializes CAN bus
	PDM_CAN_Init(hcan, &canConfig);

	//Initializes CAN ID buffer
	__PDM_ID_BUFFER_INIT();

	//Sets outputs based on input levels
	PDM_Output_Process();

	//Starts Multisense data conversion
	flagReading[0] = Data_Read_Ready;
	flagReading[1] = Data_Read_Ready;
	PDM_Data_Conversion(&htim6);

	//Initializes timers and ADC conversion
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcBuffer[5], 5);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) &adcBuffer[0], 5);
	HAL_TIM_Base_Start_IT(&htim7);

	return;
}

void PDM_USB_Process(uint8_t *Data, uint16_t Size)
{
	return;
}

//Sends data channels via USB
void PDM_USB_Transmit_Data()
{
	return;
}

HAL_StatusTypeDef PDM_PWM_Load_SoftStart_From_EEPROM(I2C_HandleTypeDef*hi2c, PWM_Control_Struct* pwm_struct, uint8_t pwm_out_number)
{
	uint8_t buffer;
	uint16_t buffer_address = 0;
	HAL_StatusTypeDef retVal = HAL_OK;

	switch(pwm_out_number)
	{
	case 0:
		buffer_address = EEPROM_PWM1_SST1_ADDRESS;
		break;

	case 1:
		buffer_address = EEPROM_PWM2_SST1_ADDRESS;
		break;

	case 2:
		buffer_address = EEPROM_PWM3_SST1_ADDRESS;
		break;

	case 3:
		buffer_address = EEPROM_PWM4_SST1_ADDRESS;
		break;
	}



	switch(pwm_struct->pwmFrequency)
	{
	case PWM_FREQ_100HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 100) / 1000;
		break;

	case PWM_FREQ_250HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 250) / 1000;
		break;

	case PWM_FREQ_500HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 500) / 1000;
		break;

	case PWM_FREQ_750HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 750) / 1000;
		break;

	case PWM_FREQ_1000HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 1000) / 1000;
		break;

	case PWM_FREQ_2500HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 2500) / 1000;
		break;

	case PWM_FREQ_5000HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 5000) / 1000;
		break;

	case PWM_FREQ_7500HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 7500) / 1000;
		break;

	case PWM_FREQ_10000HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 10000) / 1000;
		break;

	case PWM_FREQ_15000HZ:
		pwm_struct->softStartStruct->slope = (pwm_struct->softStartStruct->turnOnTime * 15000) / 1000;
		break;

	default:
		break;
	}

	return retVal;
}

HAL_StatusTypeDef PDM_PWM_Map_Load_From_EEPROM(I2C_HandleTypeDef* hi2c, PWM_Control_Struct* pwm_struct, uint16_t mem_address)
{
	uint8_t buffer[EEPROM_PWM_CFG_MAX_SIZE];
	uint16_t bufferAddress = 0;
	HAL_StatusTypeDef retVal = HAL_OK;

	if(pwm_struct->outputType != OutType_Map)
		return retVal;

	retVal = HAL_I2C_Mem_Read(hi2c, EEPROM_I2C_ADDRESS, mem_address, 2, buffer, sizeof(buffer), EEPROM_TIMEOUT);

	if(pwm_struct->pwmMapStruct != NULL)
	{
		pwm_struct->pwmMapStruct->mapLengths[0] = buffer[0];
		pwm_struct->pwmMapStruct->mapLengths[1] = buffer[1];

		bufferAddress += 2;

		if((pwm_struct->pwmMapStruct->mapLengths[0] > PWM_TABLE_MAX_SIZE)
				|| (pwm_struct->pwmMapStruct->mapLengths[1] > PWM_TABLE_MAX_SIZE))
			pwm_struct->outputType = OutType_Error;

		if(pwm_struct->outputType == OutType_Map)
		{
			pwm_struct->pwmMapStruct->dutyCycleMap = malloc(pwm_struct->pwmMapStruct->mapLengths[0] * sizeof(uint16_t*));
			pwm_struct->pwmMapStruct->commandVarStep[0] = malloc(pwm_struct->pwmMapStruct->mapLengths[0] * sizeof(int16_t));
			pwm_struct->pwmMapStruct->commandVarStep[1] = malloc(pwm_struct->pwmMapStruct->mapLengths[1] * sizeof(int16_t));

			if((pwm_struct->pwmMapStruct->dutyCycleMap == NULL)
					|| (pwm_struct->pwmMapStruct->commandVarStep[0] == NULL)
					|| (pwm_struct->pwmMapStruct->commandVarStep[1] == NULL))
				pwm_struct->outputType = OutType_Error;

			for(uint8_t i = 0; (i < pwm_struct->pwmMapStruct->mapLengths[0]) && (pwm_struct->outputType == OutType_Map); i++)
			{
				pwm_struct->pwmMapStruct->dutyCycleMap[i] = malloc(pwm_struct->pwmMapStruct->mapLengths[1] * sizeof(uint16_t));

				if(pwm_struct->pwmMapStruct->dutyCycleMap[i] == NULL)
					pwm_struct->outputType = OutType_Error;
			}
		}

		if(pwm_struct->outputType == OutType_Map)
		{
			for(uint8_t i = 0; i < pwm_struct->pwmMapStruct->mapLengths[0]; i++)
			{
				pwm_struct->pwmMapStruct->commandVarStep[0][i]  = buffer[(i * 2) + bufferAddress] << 8;
				pwm_struct->pwmMapStruct->commandVarStep[0][i] |= buffer[(i * 2) + 1 + bufferAddress];
			}

			bufferAddress += pwm_struct->pwmMapStruct->mapLengths[0] * 2;

			for(uint8_t i = 0; i < pwm_struct->pwmMapStruct->mapLengths[1]; i++)
			{
				pwm_struct->pwmMapStruct->commandVarStep[1][i]  = buffer[(i * 2) + bufferAddress] << 8;
				pwm_struct->pwmMapStruct->commandVarStep[1][i] |= buffer[(i * 2) + 1 + bufferAddress];
			}

			bufferAddress += pwm_struct->pwmMapStruct->mapLengths[1] * 2;

			for(uint8_t x = 0; x < pwm_struct->pwmMapStruct->mapLengths[0]; x++)
			{
				for(uint8_t y = 0; y < pwm_struct->pwmMapStruct->mapLengths[1]; y++)
				{
					pwm_struct->pwmMapStruct->dutyCycleMap[x][y]  = buffer[(((10 * x) + y) * 2) + bufferAddress] << 8;
					pwm_struct->pwmMapStruct->dutyCycleMap[x][y] |= buffer[(((10 * x) + y) * 2) + 1 + bufferAddress];
				}
			}
		}
	}

	else
		pwm_struct->outputType = OutType_Error;

	return retVal;
}

//Use for configuration without or with partial EEPROM data
__weak void PDM_Hard_Code_Config()
{
	//Output 1 - Bomba de Combustível
	outputStruct[0].outEnable[0] = Output_Enabled;
	outputStruct[0].inputEnable[0] = 0x0011;
	outputStruct[0].inputLevels[0] = 0x0000;
	pwmOutStruct[0].pwmFrequency = PWM_FREQ_10000HZ;
	pwmOutStruct[0].outputType = OutType_Preset;
	pwmOutStruct[0].presetEnable[0] = 0x0011;
	pwmOutStruct[0].presetInputs[0] = 0x0000;
	pwmOutStruct[0].presetDutyCycle[0] = 1000;

	//Output 2 - Ventoinha Esquerda
	outputStruct[1].outEnable[0] = Output_Enabled;
	outputStruct[1].inputEnable[0] = 0x001E;
	outputStruct[1].inputLevels[0] = 0x0002;
	pwmOutStruct[1].pwmFrequency = PWM_FREQ_10000HZ;
	pwmOutStruct[1].outputType = OutType_Preset;
	pwmOutStruct[1].presetEnable[0] = 0x001E;
	pwmOutStruct[1].presetInputs[0] = 0x0002;
	pwmOutStruct[1].presetDutyCycle[0] = 1000;

	//Output 3 - Ventoinha Direita
	outputStruct[2].outEnable[0] = Output_Enabled;
	outputStruct[2].inputEnable[0] = 0x001E;
	outputStruct[2].inputLevels[0] = 0x0002;
	pwmOutStruct[2].pwmFrequency = PWM_FREQ_10000HZ;
	pwmOutStruct[2].outputType = OutType_Preset;
	pwmOutStruct[2].presetEnable[0] = 0x001E;
	pwmOutStruct[2].presetInputs[0] = 0x0002;
	pwmOutStruct[2].presetDutyCycle[0] = 1000;

	//Output 4 - Ventoinha de Óleo -> FT
//	outputStruct[3].outEnable[0] = Output_Enabled;
//	outputStruct[3].inputEnable[0] = 0x001E;
//	outputStruct[3].inputLevels[0] = 0x0002;
//	pwmOutStruct[3].pwmFrequency = PWM_FREQ_10000HZ;
//	pwmOutStruct[3].outputType = OutType_Preset;
//	pwmOutStruct[3].presetEnable[0] = 0x001E;
//	pwmOutStruct[3].presetInputs[0] = 0x0002;
//	pwmOutStruct[3].presetDutyCycle[0] = 1000;
	outputStruct[3].outEnable[0] = Output_Enabled;
	outputStruct[3].inputEnable[0] = 0x0010;
	outputStruct[3].inputLevels[0] = 0x0000;

	//Output 5 - FT -> desabilitada
//	outputStruct[4].outEnable[0] = Output_Enabled;
	outputStruct[4].outEnable[0] = Output_Disabled;
	outputStruct[4].inputEnable[0] = 0x0010;
	outputStruct[4].inputLevels[0] = 0x0000;

	//Output 6 - Velocidade de Roda FL e FR
//	outputStruct[5].outEnable[0] = Output_Enabled;
	outputStruct[5].outEnable[0] = Output_Disabled;
	outputStruct[5].inputEnable[0] = 0x0018;
	outputStruct[5].inputLevels[0] = 0x0000;

	//Output 7 - Velocidade de Roda RL e RR
	outputStruct[6].outEnable[0] = Output_Enabled;
	outputStruct[6].inputEnable[0] = 0x0018;
	outputStruct[6].inputLevels[0] = 0x0000;

	//Output 8 - Buck DRS
	outputStruct[7].outEnable[0] = Output_Enabled;
	outputStruct[7].inputEnable[0] = 0x0008;
	outputStruct[7].inputLevels[0] = 0x0000;

	//Output 9 - ETM e CMP
	outputStruct[8].outEnable[0] = Output_Enabled;
	outputStruct[8].inputEnable[0] = 0x0010;
	outputStruct[8].inputLevels[0] = 0x0000;

	//Output 10 - P. de Óleo e P. do Shifter
	outputStruct[9].outEnable[0] = Output_Enabled;
	outputStruct[9].inputEnable[0] = 0x0010;
	outputStruct[9].inputLevels[0] = 0x0000;

	//Output 11 - Solenoides e P. de Combustível
	outputStruct[10].outEnable[0] = Output_Enabled;
	outputStruct[10].inputEnable[0] = 0x0010;
	outputStruct[10].inputLevels[0] = 0x0000;

	//Output 12 - Relay de Partida
	outputStruct[11].outEnable[0] = Output_Enabled;
	outputStruct[11].inputEnable[0] = 0;//0x001A;
	outputStruct[11].inputLevels[0] = 0;//0x0008;

	//Output 13 - Buck DAq e Brake Light
	outputStruct[12].outEnable[0] = Output_Enabled;
	outputStruct[12].inputEnable[0] = 0x0008;
	outputStruct[12].inputLevels[0] = 0x0000;

	//Output 14 - Bobina
	outputStruct[13].outEnable[0] = Output_Enabled;
	outputStruct[13].inputEnable[0] = 0x0010;
	outputStruct[13].inputLevels[0] = 0x0000;

	//Output 15 - Bico Injetor
	outputStruct[14].outEnable[0] = Output_Enabled;
	outputStruct[14].inputEnable[0] = 0x0010;
	outputStruct[14].inputLevels[0] = 0x0000;

	//Output 16 - WBO2
	outputStruct[15].outEnable[0] = Output_Enabled;
	outputStruct[15].inputEnable[0] = 0x0010;
	outputStruct[15].inputLevels[0] = 0x0000;

	canConfig.baudRate = CAN_1000kbps;

	for(uint8_t i = 0; i < 16; i++)
		dataFreqBuffer[i] = Data_Freq_50Hz;

	return;
}

static void Output_Reset_State()
{
	HAL_GPIO_WritePin(FAULTRST_GPIO_Port, FAULTRST_Pin, GPIO_PIN_SET);

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
		memset(&outputStruct[i], '\0', sizeof(Output_Control_Struct));

	//Starts PWM timers
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//Zeroes all the PWM outputs duty cycles
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

	//Sets all normal outputs to zero
	HAL_GPIO_WritePin(OUTPUT5_GPIO_Port, OUTPUT5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT6_GPIO_Port, OUTPUT6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT7_GPIO_Port, OUTPUT7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT8_GPIO_Port, OUTPUT8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT9_GPIO_Port, OUTPUT9_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT10_GPIO_Port, OUTPUT10_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT11_GPIO_Port, OUTPUT11_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT12_GPIO_Port, OUTPUT12_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT13_GPIO_Port, OUTPUT13_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT14_GPIO_Port, OUTPUT14_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT15_GPIO_Port, OUTPUT15_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUTPUT16_GPIO_Port, OUTPUT16_Pin, GPIO_PIN_RESET);

	return;
}

static HAL_StatusTypeDef Output_Cfg_Load_From_EEPROM(I2C_HandleTypeDef* hi2c)
{
	uint8_t buffer[EEPROM_OUT_CFG_BUFFER_SIZE];
	uint16_t startAddress = 0x0000;
	HAL_StatusTypeDef retVal = HAL_OK;

	retVal = HAL_I2C_Mem_Read(hi2c, EEPROM_I2C_ADDRESS, startAddress, 2, buffer, sizeof(buffer), EEPROM_TIMEOUT);

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		outputStruct[i].inputEnable[0]  = buffer[i * 6 * sizeof(uint16_t)] << 8;
		outputStruct[i].inputEnable[0] |= buffer[(i * 6 * sizeof(uint16_t)) + 1];

		outputStruct[i].inputEnable[1]  = buffer[(i * 6 * sizeof(uint16_t)) + 2] << 8;
		outputStruct[i].inputEnable[1] |= buffer[(i * 6 * sizeof(uint16_t)) + 3];

		outputStruct[i].inputLevels[0]  = buffer[(i * 6 * sizeof(uint16_t)) + 4] << 8;
		outputStruct[i].inputLevels[0] |= buffer[(i * 6 * sizeof(uint16_t)) + 5];

		outputStruct[i].inputLevels[1]  = buffer[(i * 6 * sizeof(uint16_t)) + 6] << 8;
		outputStruct[i].inputLevels[1] |= buffer[(i * 6 * sizeof(uint16_t)) + 7];

		outputStruct[i].currentThresholds  = buffer[(i * 6 * sizeof(uint16_t)) + 8] << 8;
		outputStruct[i].currentThresholds |= buffer[(i * 6 * sizeof(uint16_t)) + 8];

		outputStruct[i].timeoutOutputFuse  = buffer[(i * 6 * sizeof(uint16_t)) + 10] << 8;
		outputStruct[i].timeoutOutputFuse |= buffer[(i * 6 * sizeof(uint16_t)) + 11];
	}

	return retVal;
}

static HAL_StatusTypeDef Output_Cfg_Save_To_EEPROM(I2C_HandleTypeDef* hi2c)
{
	uint8_t buffer[EEPROM_OUT_CFG_BUFFER_SIZE];

	for(uint8_t i = 0; i < NBR_OF_OUTPUTS; i++)
	{
		buffer[i * 6 * sizeof(uint16_t)]	   = outputStruct[i].inputEnable[0] >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 1] = outputStruct[i].inputEnable[0] & 0xff;

		buffer[(i * 6 * sizeof(uint16_t)) + 2] = outputStruct[i].inputEnable[1] >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 3] = outputStruct[i].inputEnable[1] & 0xff;

		buffer[(i * 6 * sizeof(uint16_t)) + 4] = outputStruct[i].inputLevels[0] >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 5] = outputStruct[i].inputLevels[0] & 0xff;

		buffer[(i * 6 * sizeof(uint16_t)) + 6] = outputStruct[i].inputLevels[1] >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 7] = outputStruct[i].inputLevels[1] & 0xff;

		buffer[(i * 6 * sizeof(uint16_t)) + 8] = outputStruct[i].currentThresholds >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 9] = outputStruct[i].currentThresholds & 0xff;

		buffer[(i * 6 * sizeof(uint16_t)) + 10] = outputStruct[i].timeoutOutputFuse >> 8;
		buffer[(i * 6 * sizeof(uint16_t)) + 11] = outputStruct[i].timeoutOutputFuse & 0xff;
	}

	return HAL_I2C_Mem_Write(hi2c, EEPROM_I2C_ADDRESS, EEPROM_OUT_CFG_ADDRESS, 2, buffer, sizeof(buffer), EEPROM_TIMEOUT);
}

static HAL_StatusTypeDef PWM_Output_Cfg_Load_From_EEPROM(I2C_HandleTypeDef* hi2c)
{
	uint8_t buffer[EEPROM_PWM_CFG_BUFFER_SIZE];
	HAL_StatusTypeDef retVal = HAL_OK;

	retVal = HAL_I2C_Mem_Read(hi2c, EEPROM_I2C_ADDRESS, EEPROM_PWM_CFG_ADDRESS, 2, buffer, sizeof(buffer), EEPROM_TIMEOUT);

	for(uint8_t i = 0; i < NBR_OF_PWM_OUTPUTS; i++)
	{
		pwmOutStruct[i].outputType    = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t))))];

		pwmOutStruct[i].pwmFrequency  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 2];
		pwmOutStruct[i].pwmFrequency |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 3];

		pwmOutStruct[i].presetEnable[0]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 4];
		pwmOutStruct[i].presetEnable[0] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 5];

		pwmOutStruct[i].presetInputs[0]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 6];
		pwmOutStruct[i].presetInputs[0] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 7];

		pwmOutStruct[i].presetDutyCycle[0]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 8];
		pwmOutStruct[i].presetDutyCycle[0] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 9];

		pwmOutStruct[i].presetEnable[1]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 10];
		pwmOutStruct[i].presetEnable[1] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 11];

		pwmOutStruct[i].presetInputs[1]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 12];
		pwmOutStruct[i].presetInputs[1] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 13];

		pwmOutStruct[i].presetDutyCycle[1]  = buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 14];
		pwmOutStruct[i].presetDutyCycle[1] |= buffer[(i * ((2 * sizeof(uint8_t)) + (7 * sizeof(uint16_t)))) + 15];
	}

	return retVal;
}

static HAL_StatusTypeDef PWM_Output_Cfg_Save_To_EEPROM(I2C_HandleTypeDef* hi2c)
{
	return HAL_OK;
}
