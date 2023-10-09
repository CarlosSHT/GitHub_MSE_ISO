#ifndef INC_LPC4337_H_
#define INC_LPC4337_H_

#ifdef LPC4337

#include "cmsis_43xx.h"

#define IRQ_NUMBER      53                  /* Number of interrupts supported by the MCU */

typedef LPC43XX_IRQn_Type   osIRQnType;             /* STM32F4XX interrupt number definition */
typedef void (*IRQHandler)(void* data);     /* Protorype of function */

// Data type of IRQ vector.
typedef struct
{
	IRQHandler  handler;    // Function served by the IRQ.
	void*       data;		// Data that is passed to the function that services the IRQ.
}osIRQVector;


extern osIRQVector irqVector[IRQ_NUMBER];

/**
 * @brief Function used to execute the interrupt logic and
 * clear the interrupt trigger bit.
 *
 * @param[in]	irqType		IRQ number on the interrupts vector.
 */
void osIRQHandler(osIRQnType irqType);

#endif // LPC4337
#endif // INC_LPC4337_H_

