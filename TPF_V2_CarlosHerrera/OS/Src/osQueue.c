/*******************************************************************************
* Title                 :   ------
* Filename              :   osQueue.c
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Jan 13, 2024
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
bool osQueueInit(osQueueObject* queue, const uint32_t dataSize)
{
	memset(queue->buffer_data, 0, sizeof(queue->buffer_data));
	queue->idx_tocpy=0;
	queue->blocked_task=NULL;

	queue->datasize_bytes=dataSize;
	queue->max_ndata=MAX_SIZE_QUEUE/dataSize;
	queue->counter_data=0;



	queue->filled=false;
	return true;
}

void osQueueSend(osQueueObject* queue, const void* data, const uint32_t timeout)
{
	bool_t check_task = false;		/// Flag para verificación si una tarea se habia quedado bloquead al pedir datos de una cola vacia
	osTaskObject	*running_task;	/// Tarea a bloquear

	bool_t set_timeout=false;		/// Flag para saber si se configura un timeout

	bool_t continue_while=true;		/// Flag para salir de bucle si el timeout de la tarea llegó a 0

	/// Se intenta escribir desde una tarea a una cola llena, entonces se revisa que tarea es
	/// Si esta en un estado de interrupción salir
	/// Si existe un timeout con un tiempo finito se actualiza los valores del TIMEOUT de la tarea a bloquear
	///
	/// Permanece bloqueada si sigue lleno de datos maximos o hasta que el timeout termine (continue_while)
	if (queue->counter_data >= queue->max_ndata) {

		osEnterCriticalSection();
		running_task=osGetCurrentTask();
		osExitCriticalSection();

		if (osKernel_GetStateStep()==os_irqUpdt) {
			osErrorHook(NULL);
			return;
		}

		if (timeout!=OS_MAX_DELAY) {
			set_timeout=true;
			running_task->ticks_timeout= timeout;
			running_task->active_timeout=true;
		}

		while((queue->counter_data >= queue->max_ndata) && continue_while)
		{
			osEnterCriticalSection();
			running_task->state=OS_TASK_BLOCK;
			queue->blocked_task=running_task;
			///Si se configuro un timeout y el flag de timout de la tarea volvio a falso entonces hayu que poner
			/// la tarea en estado Ready y salir del bucle, también limpiar que tarea esta bloqueada debido la cola
			if (running_task->active_timeout==false && set_timeout==true) {
				running_task->state=OS_TASK_READY;
				continue_while=false;
				queue->blocked_task=NULL;
			}
			osExitCriticalSection();
			osKernel_YieldCPU();
		}

		/// Si dejo de estar lleno entonces se escribe un dato
		if (!(queue->counter_data >= queue->max_ndata))
		{
			queue->counter_data++;
			memcpy(&queue->buffer_data[queue->datasize_bytes * (queue->counter_data)], data, queue->datasize_bytes);
		}

	}
	else
	{
		/// Si estuvo vacio hayque verificar al final que tarea hay que liberar si existe una bloqueada
		if (queue->counter_data==0) check_task=true;

		/// Primero se llena los datos en la cola para que si hay una tarea bloqueada lo reciba sin problemas
		memcpy(&queue->buffer_data[queue->datasize_bytes * (queue->counter_data)], data, queue->datasize_bytes);
		queue->counter_data++;
//		queue->idx_tocpy = queue->idx_tocpy + queue->datasize_bytes;

		/// Se actualiza si hay una tarea bloqueada
		if(check_task)
		{
			if (queue->blocked_task->state == OS_TASK_BLOCK && queue->blocked_task!=NULL) {
				queue->blocked_task->state = OS_TASK_READY;

				if (osKernel_GetStateStep()==os_irqUpdt) {
					osKernel_SETschdlrINirq();
				}
			}
		}

	}

}

void osQueueReceive(osQueueObject* queue, void* buffer, const uint32_t timeout)
{
	osTaskObject	*running_task;
	uint8_t		temp_buffer[MAX_SIZE_QUEUE];
	bool_t set_timeout=false;		/// Flag para saber si se configura un timeout
	bool_t continue_while=true;		/// Flag para salir de bucle si el timeout de la tarea llegó a 0


	/// Si la cola tiene al menos un elemento
	/// Se extrae el valor y se hace un scheduler si esta en interrupcion
	if (queue->counter_data>0) {

		/// Como ya existe un dato en la cola entonces hay que leerla

		memcpy(buffer, queue->buffer_data, queue->datasize_bytes); //7FIJO
	    memset(temp_buffer, 0, sizeof(temp_buffer));

	    memcpy(temp_buffer, &queue->buffer_data[queue->datasize_bytes], queue->datasize_bytes * (queue->counter_data -1)  );
	    memset(queue->buffer_data, 0, sizeof(queue->buffer_data));
	    memcpy( queue->buffer_data, temp_buffer, sizeof(queue->buffer_data));
		queue->counter_data--;

		if (osKernel_GetStateStep()==os_irqUpdt) {
			osKernel_SETschdlrINirq();
		}
	}
	/// Si esta vacia lo primero que hay que hacer es salir si estamos en una interrupción
	/// Si no estamos en interrupcion hay que bloquear la tarea que intenta leer la cola
	/// hasta que haya UN elemento o TERMINE el timeout
	else {


		osEnterCriticalSection();
		running_task=osGetCurrentTask();
		osExitCriticalSection();


		if (osKernel_GetStateStep()==os_irqUpdt) {
			osErrorHook(NULL);
			return;
		}

		if (timeout!=OS_MAX_DELAY) {
			set_timeout=true;
			running_task->ticks_timeout= timeout;
			running_task->active_timeout=true;
		}

		while(queue->counter_data== 0 && continue_while)
		{
			osEnterCriticalSection();
			running_task->state=OS_TASK_BLOCK;
			queue->blocked_task=running_task;
			///Si se configuro un timeout y el flag de timout de la tarea volvio a falso entonces hay que poner
			/// la tarea en estado Ready y salir del bucle, también limpiar que tarea esta bloqueada debido la cola
			if (running_task->active_timeout==false && set_timeout==true) {
				running_task->state=OS_TASK_READY;
				continue_while=false;
				queue->blocked_task=NULL;
			}
			osExitCriticalSection();
			osKernel_YieldCPU();
		}

		if (queue->counter_data>0) {
			/// Como ya existe un dato en la cola entonces hay que leerla

			memcpy(buffer, queue->buffer_data, queue->datasize_bytes); //7FIJO
		    memset(temp_buffer, 0, sizeof(temp_buffer));

		    memcpy(temp_buffer, &queue->buffer_data[queue->datasize_bytes], queue->datasize_bytes * (queue->counter_data -1)  );
		    memset(queue->buffer_data, 0, sizeof(queue->buffer_data));
		    memcpy( queue->buffer_data, temp_buffer, sizeof(queue->buffer_data));
			queue->counter_data--;
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
