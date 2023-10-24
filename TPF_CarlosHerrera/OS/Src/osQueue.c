/*******************************************************************************
* Title                 :   ------
* Filename              :   osQueue.c
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Oct 5, 2023
* Version               :   x.0.0
* Compiler              :   ------
* Target                :   STM32XXX
* Notes                 :   
*******************************************************************************/

/******************************************************************************
* Private Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Private Includes
*******************************************************************************/
//#include "stm32f429.h"

#include "osQueue.h"
#include <stdlib.h>
#include <string.h>
//#include "core_cm4.h"
/******************************************************************************
* Private defines
*******************************************************************************/
//#define IS_inInterrupt()	(__get_IPSR() != 0U)

/******************************************************************************
* Private Typedefs
*******************************************************************************/


/******************************************************************************
* Private Macros
*******************************************************************************/


/******************************************************************************
* Public Variables
*******************************************************************************/


/******************************************************************************
* Private Variables
*******************************************************************************/


/******************************************************************************
* Private Function Prototypes
*******************************************************************************/


/******************************************************************************
* Public Function Prototypes
*******************************************************************************/


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Definitions/////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


/******************************************************************************
* Private Function Definitions
*******************************************************************************/



/******************************************************************************
* Public Function Definitions
*******************************************************************************/

/**
 * @fn bool osQueueInit(osQueueObject*, const uint32_t)
 */
bool osQueueInit(osQueueObject* queue, const uint32_t dataSize)
{


	if (queue == NULL || dataSize == 0) {
		queue->task=NULL;
		return false;
	}

	queue->count_space	= 0;
	queue->data_size	=	dataSize;
	queue->max_count	= MAX_SIZE_QUEUE / queue->data_size;
	queue->index_fill	=	0;
	queue->index_get 	=	0;
	memset(queue->data_bytes, 0, sizeof(queue->data_bytes));

	return true;
}

/**
 * @fn void osQueueSend(osQueueObject*, const void*, const uint32_t)
 */
void osQueueSend(osQueueObject* queue, const void* data, const uint32_t timeout)
{
	bool_t irq_state;

	irq_state=osIsIRQState();
	osUnsetIRQScheduling();


	if ((queue->count_space + queue->data_size)> MAX_SIZE_QUEUE)  {

		if (irq_state ) {
			return;
		}

		/// Se bloquea la tarea indefinidamente o por el timeout
		if (timeout==OS_MAX_DELAY) {
			osEnterCriticalSection();
			queue->task=osGetCurrentTask();
			osBlockTask(queue->task);
			osExitCriticalSection();
			osYieldCPU();
		}
	}
	else {
		/// haogo un copiado del dato entrante hacia el vector maximo
		memcpy(&queue->data_bytes[queue->count_space], data, queue->data_size);
		queue->count_space += queue->data_size;

		if (irq_state) {
			osSetIRQScheduling();
		}
	}


}

/**
 * @fn void osQueueReceive(osQueueObject*, void*, const uint32_t)
 */
void osQueueReceive(osQueueObject* queue, void* buffer, const uint32_t timeout)
{
	bool_t irq_state;

	irq_state=osIsIRQState();
	osUnsetIRQScheduling();

	if (queue->count_space < queue->data_size) {

		if (irq_state ) {
			return;
		}

		/// Se bloquea porque el buffer esta vacio
		if (timeout==OS_MAX_DELAY) {
			osEnterCriticalSection();
			queue->task=osGetCurrentTask();
			osBlockTask(queue->task);
			osExitCriticalSection();
			osYieldCPU();
		}
	}

	else {
		/// Devuelvo la información
		memcpy(buffer, queue->data_bytes, queue->data_size);
	    memcpy(queue->data_bytes, &queue->data_bytes[queue->data_size], queue->count_space -  queue->data_size);
		queue->count_space -= queue->data_size;
	    // Rellenar las últimas posiciones con ceros (opcional)
	    memset(queue->data_bytes,  0, MAX_SIZE_QUEUE - queue->count_space);

		if (irq_state) {
			osSetIRQScheduling();
		}
	}


}




/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
