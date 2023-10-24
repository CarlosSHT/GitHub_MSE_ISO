#include <string.h>

#include "SerialWrapper.h"
#include "usart.h"
#include <stdio.h>
#include "osKernel.h"

void serialPrint(char* buffer)
{
	osEnterCriticalSection();
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), 100);
	osExitCriticalSection();
}
