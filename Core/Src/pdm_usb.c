/*
 * pdm_usb.c
 *
 *  Created on: 11 de out de 2022
 *      Author: Rodolfo
 */

#include "pdm.h"

void PDM_USB_Thread_Transmit_Data(void* threadStruct)
{
	//Struct containing USB peripheral and frequency Queue handle
	PDM_UsbTxMsg_Thread_Struct* thrdStr = (PDM_UsbTxMsg_Thread_Struct*) threadStruct;

	//Thread execution timing
	Data_Freq frequency;	//Frequency of execution
	uint32_t periodTicks;	//Period in RTOS ticks
	uint32_t delayTick;		//Kernel timestamp for when the thread should wake up

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Receive frequency to transmit message, if failed, exits and delete the thread
	if(osMessageQueueGet(thrdStr->freqQueueHandle, (Data_Freq*) &frequency, NULL, osWaitForever) != osOK)
		osThreadExit();

	__BUFFER_TO_FREQ(frequency, periodTicks);	//Converts frequency (Data_Freq) to period (milliseconds)
	periodTicks = pdMS_TO_TICKS(periodTicks);	//Converts period in milliseconds to RTOS ticks

	delayTick = osKernelGetTickCount();	//Get kernel current timestamp

	//Infinite loop for data transmission
	for(;;)
	{
#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif

		delayTick += periodTicks;	//Add wake up period to timestamp after waking up
		osDelayUntil(delayTick);	//Put thread to sleep until messaging period is fully elapsed

	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}
