/******************************************************************************
* Title                 :   ------
* Filename              :   osKernel.h
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
#define MAX_NUMBER_TASK         8U          // Defines maximum task we could create.
#define MAX_NUMBER_PRIORITY     4U          // Defines the maximum amount of priority.
#define MAX_STACK_SIZE          256U        // Defines maximum stack size for a task.
#define SYSTICK_PERIOD_MS       1U          // Systick period time in mili-second.
#define SIZE_STACK_FRAME        17U         // Size stack frame



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

/* Possible task status */
typedef enum
{
    OS_TASK_NOSTATE	= 0,			// NO state
    OS_TASK_READY	= 0b0001,		// Ready state
    OS_TASK_RUNNING	= 0b0010,		// Running state
    OS_TASK_BLOCK	= 0b0100,		// Blocked state
    OS_TASK_SUSPEND	= 0b1000		// Suspended state
}osTaskStatusType;

typedef enum
{
    OS_VERYHIGH_PRIORITY,
    OS_HIGH_PRIORITY,
    OS_NORMAL_PRIORITY,
    OS_LOW_PRIORITY
}osPriorityType;

typedef struct
{
    uint32_t            memory[MAX_STACK_SIZE/4];   // Memory stack
    uint32_t            stackPointer;               // Stack pointer of task
    void*               entryPoint;                 // Callback executed on task
    uint8_t             id;                         // Task ID, it is a number started in 0
    osPriorityType      priority;                   // Task priority.
    osTaskStatusType    state;                     // Status task.

    bool_t				active_block;			// Flag de activar delay => "wait"
    bool_t				active_terminate;		// Flag de activar suspender => "terminate"

    uint32_t			delay_time;				// Delay en ticks del sistema
    bool_t				active_delay;			// Flag de delay enlazado con estado bloqueado
}osTaskObject;

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
 * @brief Create task.
 *
 * @param[in,out]   handler     Data structure of task.
 * @param[in]       priority    Task priority level.
 * @param[in]       callback    Function executed on task
 *
 * @return Return true if task was success or false in otherwise.
 */
bool osTaskCreate(osTaskObject* handler, osPriorityType priority, void* callback);

/**
 * @brief Initialization pendSV exception with lowest priority possible.
 */
void osStart(void);

/**
 * @brief Execute a delay for the current task.
 *
 * @param[in]   tick Number ticks delayed.
 */
void osDelay(const uint32_t tick);

/**
 * @brief Function used as default when some task return for a problem.
 */

void osReturnTaskHook(void);

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
 * @brief Idle task of the operation system.
 */
void osIdleTask(void);

/**
 * @fn osTaskObject osGetCurrentTask*(void)
 * @brief 	Obtiene la actual tarea que se encuentra en Running
 *
 * @return	Retorna el puntero de la actual tarea en Running
 */
osTaskObject* osGetCurrentTask(void);

/**
 * @fn void osBlockTask(osTaskObject*)
 * @brief 	Realiza el bloqueo de una tarea
 *
 * @param task_handler: puntero de la tarea a bloquear
 */
void osBlockTask(osTaskObject* task_handler);

/**
 * @fn void osUnblockTask(osTaskObject*)
 * @brief	Desbloquea una tarea
 *
 * @param task_handler:	Puntero de la tarea a desbloquear
 */
void osUnblockTask(osTaskObject* task_handler);


/*******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* INC_OSKERNEL_H_ */

/*** End of File **************************************************************/
