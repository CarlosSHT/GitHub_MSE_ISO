/******************************************************************************
* Title                 :   ------
* Filename              :   osQueue.h
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

/******************************************************************************
* Public defines
*******************************************************************************/
#define MAX_SIZE_QUEUE  128     // Maximum buffer size of the queue

/******************************************************************************
* Exported Typedefs
*******************************************************************************/
/**
 * @brief Data structure queue.
 */
typedef struct
{
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
