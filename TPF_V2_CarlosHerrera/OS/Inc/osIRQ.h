/******************************************************************************
* Title                 :   ------
* Filename              :   osIRQ.h
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
#ifndef INC_OSIRQ_H_
#define INC_OSIRQ_H_

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
* Private Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Private Includes
*******************************************************************************/
#include <stdbool.h>

#ifdef STM32F429
#include "stm32f429.h"
#endif

#ifdef LPC4337
#include "lpc4337.h"
#endif
/******************************************************************************
* Public defines
*******************************************************************************/


/******************************************************************************
* Exported Typedefs
*******************************************************************************/
typedef bool bool_t;

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
 * @brief Registering the callback in the os interrupt vector and enabling the interrupt.
 *
 * @param[in]	irqType		IRQ number on the interrupts vector.
 * @param[in]	function    Logic to be executed in the interruption.
 * 							El USUARIO debe limpiar las interrupciones del periferico
 * 							dentro de la funcion Callback
 * @param[in]	data		Data used by the logic performed in the interrupt.
 *
 * @return Returns true if the operation was successful otherwise false.
 */
bool osRegisterIRQ(osIRQnType irqType, IRQHandler function, void *data);

/**
 * @brief Clears the callback register in the OS interrupt vector and disables the interrupt.
 *
 * @param[in]	irqType		IRQ number on the interrupts vector.
 *
 * @return Returns true if the operation was successful otherwise false.
 */
bool osUnregisterIRQ(osIRQnType irqType);

/*******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_OSIRQ_H_ */

/*** End of File **************************************************************/
