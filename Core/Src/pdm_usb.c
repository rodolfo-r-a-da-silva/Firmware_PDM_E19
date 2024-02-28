/*
 * pdm_usb.c
 *
 *  Created on: 11 de out de 2022
 *      Author: Rodolfo
 */

#include "pdm.h"

/*BEGIN GLOBAL VARIABLES*/
uint8_t usbConnectedFlag;
uint8_t usbVcpParameters[7];
/*END GLOBAL VARIABLES*/

void PDM_USB_Thread_Transmit_Data(void* threadStruct)
{
	//Struct containing USB peripheral and frequency Queue handle
//	PDM_UsbTxMsg_Thread_Struct* thrdStr = (PDM_UsbTxMsg_Thread_Struct*) threadStruct;

#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
	//Thread stack check
	uint32_t unusedStack;					//Unused size in bytes
	osThreadId_t selfId = osThreadGetId();	//Thread id for unused stack checking
	UNUSED(unusedStack);					//Removes warning telling that the variable is unused
#endif

	//Infinite loop for data transmission
	for(;;)
	{
#if(INCLUDE_uxTaskGetStackHighWaterMark == 1)
		unusedStack = osThreadGetStackSpace(selfId);	//Checks minimum unused stack size
#endif
	}

	osThreadExit(); //Exits thread if, for some reason, it reaches the end to avoid kernel errors

	return;
}
