/*******************************************************************************
* Title                 :   ------
* Filename              :   osIRQ.c
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Oct 8, 2023
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
#include "osIRQ.h"
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
 * @fn bool osRegisterIRQ(osIRQnType, IRQHandler, void*)
 */
bool osRegisterIRQ(osIRQnType irqType, IRQHandler function, void *data)
{
	bool_t ret = false;

	if (irqVector[irqType].handler == NULL) {	/// verifica este vacio el puntero a funcion
		irqVector[irqType].handler = function;	/// asigna la funcion
		irqVector[irqType].data = data;			/// asigna el argumento a la funcion
		NVIC_ClearPendingIRQ(irqType);
		NVIC_EnableIRQ(irqType);
		ret = true;
	}

    return ret;
}

/**
 * @fn bool osUnregisterIRQ(osIRQnType)
 */
bool osUnregisterIRQ(osIRQnType irqType)
{
	bool_t ret = false;

	if (irqVector[irqType].handler != NULL) {	/// Verifica tenga un puntero a funcion
		irqVector[irqType].handler = NULL;		/// Limpia la funcion
		irqVector[irqType].data = NULL;			/// Limpia el argumento
		NVIC_ClearPendingIRQ(irqType);
		NVIC_EnableIRQ(irqType);
		ret = true;
	}
    return ret;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
