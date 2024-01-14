/******************************************************************************
* Title                 :   ------
* Filename              :   osKernel.h
* Author                :   Carlos Herrera Trujillo
* Origin Date           :   Jan 11, 2024
* Version               :   x.0.0
* Compiler              :   ------
* Target                :   STM32XXX
* Notes                 :   
*******************************************************************************/


/******************************************************************************
* Define to Prevent Recursive Inclusion
*******************************************************************************/
#ifndef INC_OSKERNEL_H_
#define INC_OSKERNEL_H_

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
#define SYSTICK_PERIOD_MS		1U          // Systick period time in mili-second.
#define MAX_NUMBER_PRIORITY		4U          // Defines the maximum amount of priority.
#define REAL_NUMBER_PRIORITY	MAX_NUMBER_PRIORITY + 1U	// Defines maximum task we could create.

#define MAX_NUMBER_TASK			8U          // Defines maximum task we could create.
#define MAX_STACK_SIZE          512U        // Defines maximum stack size for a task.
#define SIZE_STACK_FRAME        17U         // Size stack frame
#define ZERO_TICKS				0U


#define XPSR_VALUE              1 << 24     // xPSR.T = 1
#define EXEC_RETURN_VALUE       0xFFFFFFF9  // EXEC_RETURN value. Return to thread mode with MSP, not use FPU
#define XPSR_REG_POSITION       1U
#define PC_REG_POSTION          2U
#define LR_REG_POSTION          3U
#define R12_REG_POSTION         4U
#define R3_REG_POSTION          5U
#define R2_REG_POSTION          6U
#define R1_REG_POSTION          7U
#define R0_REG_POSTION          8U
#define LR_PREV_VALUE_POSTION   9U
#define R4_REG_POSTION          10U
#define R5_REG_POSTION          11U
#define R6_REG_POSTION          12U
#define R7_REG_POSTION          13U
#define R8_REG_POSTION          14U
#define R9_REG_POSTION          15U
#define R10_REG_POSTION         16U
#define R11_REG_POSTION         17U
/******************************************************************************
* Exported Typedefs
*******************************************************************************/
typedef bool bool_t;
typedef enum _os_state
{
	os_fromReset = 0,
	os_normalRun,
	os_scheduling,
	os_irqUpdt,
} os_State;

typedef enum
{
	OS_VERYHIGH_PRIORITY,
	OS_HIGH_PRIORITY,
	OS_NORMAL_PRIORITY,
	OS_LOW_PRIORITY
} osPriorityType;

typedef enum
{
	OS_TASK_READY = 0,
	OS_TASK_RUNNING,
	OS_TASK_BLOCK,
	OS_TASK_SUSPEND,
} osTaskStatusType;

typedef struct _osTaskObject
{
	uint32_t memory[MAX_STACK_SIZE / 4];   // Memory stack
	uint32_t stackPointer;               // Stack pointer of task
	void *entryPoint;                 // Callback executed on task
	void *task_return_hook;           // Callback executed on task

	osPriorityType		priority;
	osTaskStatusType 	state;
	uint32_t 			ticks_blocked;
	bool_t 				active_delay;

	uint32_t 			ticks_timeout;
	bool_t 				active_timeout;
	struct _osTaskObject *next_task;
} osTaskObject;
/******************************************************************************
* Exported constants
*******************************************************************************/

	
/******************************************************************************
* Exported macro
*******************************************************************************/


/******************************************************************************
* Exported functions prototypes
*******************************************************************************/

bool osTaskCreate(osTaskObject* handler, osPriorityType priority, void* callback);

void osStart(void);

void osDelay(const uint32_t n_tick);
void osReturnTaskHook(void);

void osIdleTask(void);


/**
 * @brief Function used if user would like to do something on systick hander interrupt.
 * It has a default implementation that do anything.
 *
 * @warning The function used to perform operations on each Systick in the system. It
 * be as short as possible because it is called in the Systick interrupt handler.
 *
 * @warning The function shouldn't call an OS API in any case because a new scheduler
 * could occur.
 */
void osSysTickHook(void);

/**
 * @brief Function used when happen error on OS
 *
 * @param[in]   caller  Function pointer where error happened.
 */
void osErrorHook(void* caller);

/**
 * @fn void osEnterCriticalSection(void)
 * @brief Declare the beginning of the critical section.
 * 			Permite la creacion de secciones criticas anidadas
 *
 */
void osEnterCriticalSection(void);

/**
 * @brief Declare the end of the critical section.
 * 			Permite finalizar un anidado de seccion critica
 * 			Se recomienda implementar el retorno de valor BOOL
 * 			a fin de saber si se rehabilitó las interrupciones
 */
void osExitCriticalSection(void);


void osKernel_YieldCPU(void);

os_State osKernel_GetStateStep(void);

void osKernel_SETschdlrINirq(void);

void osKernel_UNSETschdlrINirq(void);
bool_t osKernel_GETschdlrINirq(void);
osTaskObject* osGetCurrentTask(void);

void osKernel_SaveState(void);

void osKernel_RecoverState(void);


void osSetRESETState();
void osSetNORMALState();

void osSetSCHEDULINGState();
void osSetIRQState();
/*******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_OSKERNEL_H_ */

/*** End of File **************************************************************/
