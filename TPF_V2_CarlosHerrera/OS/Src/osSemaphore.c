/*******************************************************************************
* Title                 :   ------
* Filename              :   osSemaphore.c
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
#include "osSemaphore.h"
#include <stddef.h>

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
 * @brief Initializes semaphore binary or not.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 * @param[in]       maxCount    Maximum count value that can be reached.
 * @param[in]       count       The count value assigned to the semaphore when it is created.
 */
void osSemaphoreInit(osSemaphoreObject* semaphore, const uint32_t maxCount, const uint32_t count)
{
	semaphore->max_counter = maxCount;
	semaphore->init_counter = count;
	semaphore->counter = (int32_t)semaphore->max_counter -(int32_t) semaphore->init_counter;

	for (int var = 0; var < MAX_NUMBER_TASK; ++var) {
		semaphore->state_blckd[var]=false;
		semaphore->tasks_blckd[var]=NULL;
	}
}

/**
 * @brief Take semaphore.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 *
 * @return Returns true if the semaphore could be taken.
 */
bool osSemaphoreTake(osSemaphoreObject* semaphore)
{
	osTaskObject* task2block = NULL;
	uint32_t	min;
	uint32_t	idx_task;

	semaphore->counter--;
	if (semaphore->counter<0) {

		osEnterCriticalSection();
		task2block = osGetCurrentTask();
		task2block->state=OS_TASK_BLOCK;
		osExitCriticalSection();
		semaphore->blocked_counter++;
		if (semaphore->max_counter<MAX_NUMBER_TASK) {
			min=semaphore->max_counter;
		}
		else
		{
			min=MAX_NUMBER_TASK;
		}

		for (idx_task = 0; idx_task < min; ++idx_task) {
			if (semaphore->tasks_blckd[idx_task]==NULL) {
				semaphore->tasks_blckd[idx_task]=task2block;
				semaphore->state_blckd[idx_task]=true;
				break;
			}
		}

		osKernel_YieldCPU();
//		while(semaphore->list_tasks[idx_task]->blockedbysemph)
//		{
//
//		}

	}

	return true;
}

/**
 * @brief Give semaphore.
 *
 * @param[in,out]   semaphore   Semaphore handler.
 */
void osSemaphoreGive(osSemaphoreObject* semaphore)
{
	osTaskObject* task2free = NULL;
	bool_t do_scheduler =false;

	if (semaphore->counter < semaphore->max_counter) {
		semaphore->counter++;
	}

	if (semaphore->counter<=0) {

		for (int nprior = 0; nprior < MAX_NUMBER_PRIORITY; ++nprior) {

			for (int ntask = 0; ntask < MAX_NUMBER_TASK; ++ntask) {
				if (semaphore->tasks_blckd[ntask]->priority==nprior) {
					semaphore->tasks_blckd[ntask]->state=OS_TASK_READY;
					semaphore->state_blckd[ntask]=false;
					semaphore->tasks_blckd[ntask]=NULL;
					do_scheduler= true;
					break;
				}
			}

			if (do_scheduler) {
				if (osKernel_GetStateStep()==os_irqUpdt) osKernel_SETschdlrINirq();
				break;
			}
		}
	}

}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
