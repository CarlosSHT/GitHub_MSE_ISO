/*
 * os_Kernel.h
 *
 *      Author: CARLOS
 */

#ifndef INC_OS_KERNEL_H_
#define INC_OS_KERNEL_H_

//Stack Frame

#include <stdint.h>

#define SYSTICK_PERIOD_MS	1U // milliseconds
#define MAX_NUM_TASK		8U
#define STACK_SIZE			256U
//#define SIZE_STACK_FRAME	8U	// AUTOMATIC
#define SIZE_STACK_FRAME	17U


#define XPSRT_VAL	1<<24
#define LR_IRQ_VAL	0xFFFFFFF9
#define XPSRT_POS	1U
#define PC_POS	2U
#define LR_POS	3U
#define R0_POS	8U
#define LR_IRQ_POS	9U


//Una región de memoria que actuará como el stack de esa tarea.

typedef enum
{
    StateTask_Ready,
	StateTask_Running
}osTaskStatus;

typedef struct
{
	uint32_t	stack_mem[STACK_SIZE/4];	// Longitud stack

	uint32_t	stack_ptr;
	uint32_t	entry_point;
	uint32_t	task_return_hook;
	void*		arg;

    uint8_t		id;
    osTaskStatus	state;
}osTaskObj;



//Inicialización del stack frame para la primera vez que esa tarea es llamada.
void osTaskCreate(osTaskObj *task, void* func_cb, void *arg);


#endif /* INC_OS_KERNEL_H_ */
