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
	//Struct containing ADC, CAN and timer handles
	PDM_Config_Thread_Struct* thrdStr = (PDM_Config_Thread_Struct*) ThreadStruct;

	//Threads, Semaphores, Queues, Mutexes and Structs used for Thread initialization and communication
	//Mutex IDs
	osMutexId_t canTxMutexId;

	//Mutex Attributes
	osMutexAttr_t canTxMutexAttr = {.name = "canMutex", .attr_bits = osMutexRobust};

	//Queue IDs
	osMessageQueueId_t canRxQueueId;
	osMessageQueueId_t dataQueueId[NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS];
	osMessageQueueId_t usbQueueId;

	//Queue Attributes
	osMessageQueueAttr_t canRxQueueAttr = {.name = "canRxQueue"};
	osMessageQueueAttr_t dataQueueAttr[NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS];
	osMessageQueueAttr_t usbQueueAttr = {.name = "usbQueue"};

	//Semaphore Attributes
	osSemaphoreAttr_t canRxSemaphoreAttr = {.name = "canRxSemaphore"};
	osSemaphoreAttr_t outSemaphoreAttr = {.name = "outSemaphore"};
	osSemaphoreAttr_t readSemaphoreAttr = {.name = "readSemaphore"};

	//Thread IDs
	osThreadId_t canRxThreadId;
	osThreadId_t canTxThreadId[NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS];
	osThreadId_t outThreadId;
	osThreadId_t readThreaId;
	osThreadId_t usbRxThreadId;
	osThreadId_t usbTxThreadId;

	//Thread Attributes
	osThreadAttr_t canRxThreadAttr = {.name = "canRxThread", .stack_size = RTOS_THREAD_MEMSZ_CANRX, .priority = (osPriority_t) RTOS_THREAD_PRIO_CANRX};
	osThreadAttr_t canTxThreadAttr[NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS];
	osThreadAttr_t outThreadAttr = {.name = "outThread", .stack_size = RTOS_THREAD_MEMSZ_OUT, .priority = (osPriority_t) RTOS_THREAD_PRIO_OUT};
	osThreadAttr_t readThreadAttr = {.name = "readThread", .stack_size = RTOS_THREAD_MEMSZ_READ, .priority = (osPriority_t) RTOS_THREAD_PRIO_READ};
	osThreadAttr_t usbRxThreadAttr = {.name = "usbRxThread", .stack_size = RTOS_THREAD_MEMSZ_USBRX, .priority = (osPriority_t) RTOS_THREAD_PRIO_USBRX};
	osThreadAttr_t usbTxThreadAttr = {.name = "usbTxThread", .stack_size = RTOS_THREAD_MEMSZ_USBTX, .priority = (osPriority_t) RTOS_THREAD_PRIO_USBTX};

	//Thread Structs
	PDM_CanRxMsg_Thread_Struct canRxMsgStruct;
	PDM_CanTxMsg_Thread_Struct canTxMsgStruct = {.canMutexHandle = canTxMutexId, .freqQueueHandle = canTxFreqQueueId, .hcan = thrdStr->hcan};
	PDM_OutSet_Thread_Struct outSetStruct;
	PDM_Readings_Thread_Struct readStruct;
	PDM_UsbTxMsg_Thread_Struct usbTxStruct;

	//Variables and arrays for data parameters storage
	uint16_t adcBuffer[NBR_OF_ADC_CHANNELS];

	uint16_t dataIdBuffer[NBR_OF_DATA_CHANNELS];
	Data_Freq dataFreqBuffer[NBR_OF_DATA_CHANNELS+NBR_OF_DATA_MSGS];

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Mutex Inialization
	canTxMutexId = osMutexCreate(&canTxMutexAttr);

	//Queue Initialization
	canRxQueueId = osMessageQueueNew(RTOS_QUEUE_CANRX_COUNT, RTOS_QUEUE_CANRX_SIZE, &canRxQueueAttr);
	canTxFreqQueueId = osMessageQueueNew(RTOS_QUEUE_CANTXFREQ_COUNT, RTOS_QUEUE_CANTXFREQ_SIZE, &canTxFreqQueueAttr);
	usbQueueId = osMessageQueueNew(RTOS_QUEUE_USB_COUNT, RTOS_QUEUE_USB_SIZE, &usbQueueAttr);

	//Semaphore Initialization
	thrdStr->canRxSemaphoreId = osSemaphoreNew(RTOS_SEMAPHORE_CANRX_MAX, RTOS_SEMAPHORE_CANRX_INIT, &canRxSemaphoreAttr);
	thrdStr->outputSemaphoreId = osSemaphoreNew(RTOS_SEMAPHORE_OUT_MAX, RTOS_SEMAPHORE_OUT_INIT, &outSemaphoreAttr);
	thrdStr->readingSemaphoreId = osSemaphoreNew(RTOS_SEMAPHORE_READ_MAX, RTOS_SEMAPHORE_READ_INIT, &readSemaphoreAttr);

	for(;;)
	{
		Can_Tx_Threads_Init(dataFreqBuffer, &canTxMsgStruct, canTxThreadId, canTxThreadAttr, &canTxFreqQueueId);

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

		osThreadYield();	//Stops execution after configuring parameters
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
			outStruct[i]->pwmStruct.htim = &htimBuffer[acc];
			outStruct[i]->pwmStruct.timChannel = timChannelBuffer[acc];

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
	uint8_t acc = 0;
	uint8_t freqFlag[NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS];

	//Terminates all running CAN transmission Threads
	for(uint8_t i = 0; i < (NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS); i++)
		osThreadTerminate(threadIdBuffer[i]);

	//Check which frequencies will be used in transmission
	for(uint8_t i = 0; i <= NBR_OF_FREQ_OPTS; i++)	//Sweep each message frequency option
		for(uint8_t j = 0; j < (NBR_OF_DATA_CHANNELS+NBR_OF_DATA_MSGS); j++)	//Sweep each data frequency buffer position
			if((freqBuffer[j] == (i+1)) && (freqBuffer[j] != Data_Disabled))
				freqFlag[i] = 1;	//Set flag if any data uses the compared frequency

	//Create CAN Transmission Threads
	for(uint8_t i = 0; i < (NBR_OF_FREQ_OPTS+NBR_OF_DATA_MSGS); i++)
	{
		if(freqFlag[i] == 1)	//Create Thread only if it's used
		{
			acc++;	//Increment number of created Threads
			threadAttrBuffer[i]->priority = RTOS_THREAD_PRIO_CANTX;		//Set Thread execution priority
			threadAttrBuffer[i]->stack_size = RTOS_THREAD_MEMSZ_CANTX;	//Set Thread Stack usage size
			threadIdBuffer[i] = osThreadNew(PDM_CAN_Thread_Transmit_Data, (void*) threadStruct, threadAttrBuffer[i]);	//Create new Thread
		}
	}

	return acc;	//Return number of used Threads
}

static void EEPROM_Read_Pt1()
{
	return;
}

static void EEPROM_Read_Pt2()
{
	return;
}
