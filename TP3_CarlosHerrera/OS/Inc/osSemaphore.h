/******************************************************************************
* Title                 :   ------
* Filename              :   osSemaphore.h
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Oct 5, 2023
* Version               :   x.0.0
* Compiler              :   ------
* Target                :   STM32XXX
* Notes                 :   
*******************************************************************************/


/******************************************************************************
* Define to Prevent Recursive Inclusion
*******************************************************************************/
#ifndef INC_OSSEMAPHORE_H_
#define INC_OSSEMAPHORE_H_

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
* Private Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Private Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "osKernel.h"
/******************************************************************************
* Public defines
*******************************************************************************/


/******************************************************************************
* Exported Typedefs
*******************************************************************************/
typedef bool bool_t;

typedef struct
{
	uint8_t			counter;			//	contador del semaforo (usado como contador inicial)
//	uint8_t			start_counter;
	uint8_t			max_counter;		//	maximo numero de cuenta
	uint8_t			bloqued_counter;	//	contador de tareas bloqueadas

	osTaskObject*	list_tasks[MAX_NUMBER_TASK];	//	lista de tareas bloqueadas por semaforo
	osPriorityType	list_priority[MAX_NUMBER_TASK];	//	lista de prioridades de las tareas bloqueadas
}osSemaphoreObject;

/******************************************************************************
* Exported constants
*******************************************************************************/

	
/******************************************************************************
* Exported macro
*******************************************************************************/


/******************************************************************************
* Exported functions prototypes
*******************************************************************************/

/**
 * @brief Initializes semaphore binary or not.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 * @param[in]       maxCount    Maximum count value that can be reached.
 * @param[in]       count       The count value assigned to the semaphore when it is created.
 */
void osSemaphoreInit(osSemaphoreObject* semaphore, const uint32_t maxCount, const uint32_t count);

/**
 * @brief Take semaphore.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 *
 * @return Returns true if the semaphore could be taken.
 */
bool osSemaphoreTake(osSemaphoreObject* semaphore);

/**
 * @brief Give semaphore.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 */
void osSemaphoreGive(osSemaphoreObject* semaphore);


/*******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_OSSEMAPHORE_H_ */

/*** End of File **************************************************************/
