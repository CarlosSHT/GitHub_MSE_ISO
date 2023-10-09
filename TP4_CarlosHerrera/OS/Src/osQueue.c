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
#include "osQueue.h"
#include "osKernel.h"
#include <stdlib.h>
#include <string.h>
/******************************************************************************
* Private defines
*******************************************************************************/


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
		return false;
	}
	void *ptr_queue;

	// limpia la lista de punteros de la cola
	ptr_queue = memset(queue->data_ptrs, 0, sizeof(queue->data_ptrs));
	queue->index_fill = 0;
	queue->index_get = 0;
	queue->queue_ptr = ptr_queue;
	queue->data_size = dataSize;

	return true;
}

/**
 * @fn void osQueueSend(osQueueObject*, const void*, const uint32_t)
 */
void osQueueSend(osQueueObject* queue, const void* data, const uint32_t timeout)
 {
	bool_t flag_block = true;	// La tarea se bloquea por defecto
//	osTaskObject *task_block = NULL;
	static void *data_ptr;

	data_ptr = malloc(queue->data_size);

	// Si hay espacio donde almacenar la tarea no se bloquea
	for (int var = 0; var < MAX_SIZE_QUEUE; ++var) {
		if (queue->data_ptrs[var] == NULL)
			flag_block = false;
	}

	if (!flag_block) {	// Algoritmo para llenado por copia y se almacena el puntero en la lista
		queue->data_ptrs[queue->index_fill] = memcpy(data_ptr, data, queue->data_size);
		queue->index_fill++;
		if (queue->index_fill >= MAX_SIZE_QUEUE) {
			queue->index_fill = 0;
		}
	} else {	// La tarea es bloqueada por un tiempo si esta lleno la cola de elementos
//		task_block = osGetCurrentTask();
//		osBlockTask(task_block);
		osDelay(timeout);
	}
}

/**
 * @fn void osQueueReceive(osQueueObject*, void*, const uint32_t)
 */
void osQueueReceive(osQueueObject* queue, void* buffer, const uint32_t timeout)
{
	bool_t flag_block = true;	// La tarea se bloquea por defecto
//	osTaskObject *task_block = NULL;


	// Si la cola está vacia entonces se bloquea la tarea
	for (int var = 0; var < MAX_SIZE_QUEUE; ++var) {
		if (queue->data_ptrs[var] != NULL)	// Hay al menos un dato
			flag_block = false;
	}

	if (flag_block) {	// Bloquea la tarea por un tiempo si no hay datos en la lista de elementos de la cola
//		task_block = osGetCurrentTask();
//		osBlockTask(task_block);
		osDelay(timeout);
	}
	else {
		// Realiza una copia del elemento en la cola de puntero a puntero y luego libera la memoria
		memcpy(buffer,queue->data_ptrs[queue->index_get], queue->data_size);
		free(queue->data_ptrs[queue->index_get]);
		queue->data_ptrs[queue->index_get] = NULL;
		queue->index_get++;
		if (queue->index_get >= MAX_SIZE_QUEUE) {
			queue->index_get = 0;
		}
	}
}




/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
