/******************************************************************************
* Title                 :   ------
* Filename              :   osSemaphore.h
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Jan 13, 2024
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
#include <stdint.h>
#include <stdbool.h>
#include <osKernel.h>

/******************************************************************************
* Private Includes
*******************************************************************************/


/******************************************************************************
* Public defines
*******************************************************************************/


/******************************************************************************
* Exported Typedefs
*******************************************************************************/
typedef bool bool_t;


typedef struct
{
	uint32_t		max_counter;		// Maximo numero de cuenta
	uint32_t		init_counter;		// Numero de cuentas tomadas, puede tomar valores mayor igual o menor a max counter siempre >0
	int32_t			counter;			// Valor actual de cuenta
	uint32_t		blocked_counter;	// tareas bloquedas

	osTaskObject		*tasks_blckd[MAX_NUMBER_TASK];	//	lista de tareas bloqueadas por semaforo
	bool_t 				state_blckd[MAX_NUMBER_TASK];	//	lista de tareas bloqueadas por semaforo

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
 * @brief Take semaphore. Mayormente llamado por Tareas
 *
 * @param[in,out]   semaphore   Semaphore handler.
 *
 * @return Returns true if the semaphore could be taken.
 */
bool osSemaphoreTake(osSemaphoreObject* semaphore);

/**
 * @brief Give semaphore. Mayormente utilizado por IRQs
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
