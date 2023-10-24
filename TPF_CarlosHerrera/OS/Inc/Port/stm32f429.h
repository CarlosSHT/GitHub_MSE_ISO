/******************************************************************************
* Title                 :   ------
* Filename              :   stm32f429.h
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Oct 8, 2023
* Version               :   x.0.0
* Compiler              :   ------
* Target                :   STM32XXX
* Notes                 :   
*******************************************************************************/


/******************************************************************************
* Define to Prevent Recursive Inclusion
*******************************************************************************/
#ifndef INC_PORT_STM32F429_H_
#define INC_PORT_STM32F429_H_

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
* Private Preprocessor Constants
*******************************************************************************/
#ifdef STM32F429

/******************************************************************************
* Private Includes
*******************************************************************************/

/**
 * @note THe macros, types definition, variables and method of this file must be used internaly in the OS.
 */
#include "stm32f429xx.h"

/******************************************************************************
* Public defines
*******************************************************************************/
#define IRQ_NUMBER      91                  /* Number of interrupts supported by the MCU */

/******************************************************************************
* Exported Typedefs
*******************************************************************************/
typedef IRQn_Type   osIRQnType;             /* STM32F4XX interrupt number definition */
typedef void (*IRQHandler)(void* data);     /* Protorype of function */

// Data type of IRQ vector.
typedef struct
{
	IRQHandler  handler;    // Function served by the IRQ.
	void*       data;		// Data that is passed to the function that services the IRQ.
}osIRQVector;

/******************************************************************************
* Exported constants
*******************************************************************************/
extern osIRQVector irqVector[IRQ_NUMBER];

/******************************************************************************
* Exported macro
*******************************************************************************/


/******************************************************************************
* Exported functions prototypes
*******************************************************************************/


/**
 * @brief Function used to execute the interrupt logic and
 * clear the interrupt trigger bit.
 *
 * @param[in]	irqType		IRQ number on the interrupts vector.
 */
void osIRQHandler(osIRQnType irqType);


/*******************************************************************************/
#ifdef __cplusplus
}
#endif


#endif // STM32F429

#endif /* INC_PORT_STM32F429_H_ */

/*** End of File **************************************************************/
