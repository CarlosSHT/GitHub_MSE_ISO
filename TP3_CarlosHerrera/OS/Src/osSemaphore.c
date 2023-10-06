/*******************************************************************************
* Title                 :   ------
* Filename              :   osSemaphore.c
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
#include "osSemaphore.h"

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
 * @fn void osSemaphoreInit(osSemaphoreObject*, const uint32_t, const uint32_t)
 * @brief 	Se valida que se inicialice al menos un semaforo binario (1 o 0) de lo
 * 			contrario devuelve un semaforo NULL
 *			Las tareas que intenten tomar un semaforo que ya no cuenta con más tomas
 *			pasan a una lista de tareas bloqueadas
 * @param semaphore
 * @param maxCount
 * @param count
 */
void osSemaphoreInit(osSemaphoreObject* semaphore, const uint32_t maxCount, const uint32_t count)
{
	if (maxCount<1) {
		semaphore=NULL;
	}

	semaphore->max_counter=maxCount;
	semaphore->counter=count;
	semaphore->bloqued_counter = 0;

	for (uint8_t var = 0; var < MAX_NUMBER_TASK; ++var) {
		semaphore->list_tasks[var]=NULL;
		semaphore->list_priority[var]=MAX_NUMBER_PRIORITY + 1;
	}
}

/**
 * @fn bool osSemaphoreTake(osSemaphoreObject*)
 * @brief 	Mientras el numero de tareas (counter) que pueden tomar el semaforo
 * 			sea mayor a 0 devuelve semaforo tomado.Caso contrario, almacena la
 * 			tarea actual que se está ejecutando (osGetCurrentTask)y guarda su
 * 			puntero en una lista y la prioridad de la tarea en otra lista para
 * 			realizar un desbloqueo por nivel de prioridades
 * 			Por ultimo bloquea la tarea con la función (osBlockTask)
 *
 * @param semaphore
 * @return
 */
bool osSemaphoreTake(osSemaphoreObject* semaphore)
{
	if (semaphore == NULL) {
		return false;
	}

	uint8_t index_task;
	bool_t ret = false;
	osTaskObject* task_block = NULL;

	// Cuando no hay más tareas que puedan tomar el semaforo , este llegó a 0
	if (semaphore->counter == 0) {

		// Busca la posición vacia donde se llena con la tarea a bloquear
		// Y tambien se guarda la prioridad de dicha tarea
		{
		for (index_task = 0; index_task < MAX_NUMBER_TASK; ++index_task) {
			if (semaphore->list_tasks[index_task] == NULL) {
				break;
			}
		}
		semaphore->list_tasks[index_task] = osGetCurrentTask();
		task_block = semaphore->list_tasks[index_task];
//		osBlockTask(semaphore->list_tasks[index_task]);
		semaphore->list_priority[index_task] = semaphore->list_tasks[index_task]->priority;
		}

		semaphore->bloqued_counter++;	// se incrementa el numero de tareas bloqueadas
		ret = false;
		osBlockTask(task_block); // Se bloquea la tarea llama internamente a un cambio
														// de contexto forzado
	} else {
		semaphore->counter--;
		ret = true;
	}
	return ret;
}

/**
 * @fn void osSemaphoreGive(osSemaphoreObject*)
 * @brief 	Se realliza un desbloqueo por prioridades FIFO, es decir,
 * 			se desbloquea (osUnblockTask) la primera tarea de mas
 * 			alta prioridad que llego
 *
 * @param semaphore
 */
void osSemaphoreGive(osSemaphoreObject* semaphore)
 {

	uint8_t index_task;
	uint8_t index_unblock = 0;
	uint8_t high_priority = MAX_NUMBER_PRIORITY + 1;
	osTaskObject *task_unblock = NULL;

	if (semaphore != NULL) {

		// si no hay tareas bloqueadas se incrementa el contador (counter) hasta
		// el máximo de tareas permitido
		if (semaphore->bloqued_counter == 0) {
			if (semaphore->counter < semaphore->max_counter) {
				semaphore->counter++;
			}
		} else {
			// buscar tarea con mayor prioridad que este bloqueada
			for (index_task = 0; index_task < MAX_NUMBER_TASK; ++index_task) {
				//			if (semaphore->list_tasks[index_task] != NULL) {
				if (semaphore->list_priority[index_task] < high_priority) {
					high_priority = semaphore->list_priority[index_task];
					task_unblock = semaphore->list_tasks[index_task];
					index_unblock = index_task;
				}
				//			}
			}

			// Caso exista una tarea a desbloquear se procede con su desbloqueo
			if (task_unblock != NULL) {

				// La tarea a desbloquear (task_unblock) es eliminada de la lista de tareas bloqueadas
				{
					for (index_task = index_unblock + 1;
							index_task < MAX_NUMBER_TASK; ++index_task) {
						semaphore->list_tasks[index_task - 1] = semaphore->list_tasks[index_task];
						semaphore->list_priority[index_task - 1] = semaphore->list_priority[index_task];
					}
					index_task--;
					semaphore->list_tasks[index_task] = NULL;
					semaphore->list_priority[index_task] = MAX_NUMBER_PRIORITY
							+ 1;
				}

				semaphore->bloqued_counter--;// se decrementa el numero de tareas bloqueadas
				osUnblockTask(task_unblock);// se libera la tarea bloqueada (pasa a estado ready)
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
