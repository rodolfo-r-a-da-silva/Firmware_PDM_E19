/*
 * pdm_variables.c
 *
 *  Created on: Mar 9, 2022
 *      Author: Rodolfo
 */

#include "pdm.h"

/*BEGIN RTOS SEMAPHORES*/
osSemaphoreId_t canRxSemaphore;
osSemaphoreId_t outputSemaphore;
osSemaphoreId_t readingSemaphore;
/*END RTOS SEMAPHORES*/

/*BEGIN VARIABLES AND STRUCTS*/
//CONFIGURATION
uint8_t usbConnectedFlag;
uint8_t usbVcpParameters[7];
/*END VARIABLES AND STRUCTS*/
