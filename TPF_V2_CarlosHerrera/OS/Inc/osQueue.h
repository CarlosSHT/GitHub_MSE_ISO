/******************************************************************************
* Title                 :   ------
* Filename              :   osQueue.h
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
#ifndef INC_OSQUEUE_H_
#define INC_OSQUEUE_H_

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
#include "osKernel.h"
/******************************************************************************
* Public defines
*******************************************************************************/

#define MAX_SIZE_QUEUE  128         // Maximum buffer size of the queue
#define OS_MAX_DELAY    0xFFFFFFFF  // Macro where the queue is locked forever. It ignores the timeout variable in the implementation.



/******************************************************************************
* Exported Typedefs
*******************************************************************************/
/**
 * @brief Data structure queue.
 */
typedef struct
{
	uint8_t		buffer_data[MAX_SIZE_QUEUE];
	uint32_t	idx_tocpy;
	uint32_t	idx_todel;
	osTaskObject	*blocked_task;


	uint32_t	datasize_bytes;
	uint32_t	max_ndata;
	uint32_t	counter_data;
	bool_t		filled;
}osQueueObject;

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
 * @brief Initialize the queue.
 *
 * @param[in, out]  queue       Queue object.
 * @param[in]       dataSize    Data size of the queue.
 *
 * @return Returns true if was success in otherwise false.
 */
bool osQueueInit(osQueueObject* queue, const uint32_t dataSize);

/**
 * @brief Send data to the queue.
 *
 * @param[in, out]  queue   Queue object.
 * @param[in, out]  data    Data sent to the queue.
 * @param[in]       timeout Number of ticks to wait before blocking the task..
 *
 * @return Returns true if it could be put in the queue
 * in otherwise false.
 */
void osQueueSend(osQueueObject* queue, const void* data, const uint32_t timeout);

/**
 * @brief Receive data to the queue.
 *
 * @param[in, out]  queue   Queue object.
 * @param[in, out]  buffer  Buffer to  save the data read from the queue.
 * @param[in]       timeout Number of ticks to wait before blocking the task..
 *
 * @return Returns true if it was possible to take it out in the queue
 * in otherwise false.
 */
void osQueueReceive(osQueueObject* queue, void* buffer, const uint32_t timeout);


/*******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_OSQUEUE_H_ */

/*** End of File **************************************************************/
