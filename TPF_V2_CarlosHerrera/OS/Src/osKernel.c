/*******************************************************************************
 * Title                 :   ------
 * Filename              :   osKernel.c
 * Author                :   Carlos Herrera Trujillo
 * Origin Date           :   Jan 11, 2024
 * Version               :   x.0.0
 * Compiler              :   ------
 * Target                :   STM32XXX
 * Notes                 :
 *******************************************************************************/

/******************************************************************************
 * Private Preprocessor Constants
 *******************************************************************************/
#include <stddef.h>
#include "osKernel.h"
#include "system_stm32f4xx.h"
#include "stm32f429xx.h"
#include "core_cm4.h"
#include "cmsis_gcc.h"

/******************************************************************************
 * Private Includes
 *******************************************************************************/

/******************************************************************************
 * Private defines
 *******************************************************************************/

/******************************************************************************
 * Private Typedefs
 *******************************************************************************/
typedef bool bool_t;



typedef struct _osKernelObject
{
	osTaskObject *firstptr_tasks[REAL_NUMBER_PRIORITY];
	osTaskObject *lastptr_tasks[REAL_NUMBER_PRIORITY];
	os_State current_osState;
	os_State previous_osState;
	uint32_t number_tasks;

	osTaskObject *current_task;
	osTaskObject *next_task;

	bool_t updateContext;
	osTaskObject *next_taskfind[REAL_NUMBER_PRIORITY];
	uint32_t tasks_perprior[REAL_NUMBER_PRIORITY];
	int8_t critical_counter;						//Contador de secciones criticas solicitadas

	bool_t	schdlrINirq;
} osKernelObject;

/******************************************************************************
 * Private Macros
 *******************************************************************************/

/******************************************************************************
 * Public Variables
 *******************************************************************************/

/******************************************************************************
 * Private Variables
 *******************************************************************************/
osKernelObject osKernel =
{ 0 };
static osTaskObject Idle_Task;

/******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/
static uint32_t getNextContext(uint32_t currentStaskPointer);

static void osKernel_setPendSV(void);
static void osKernel_UPDTtaskdelays(void);
/******************************************************************************
 * Public Function Prototypes
 *******************************************************************************/

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Definitions/////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * Private Function Definitions
 *******************************************************************************/
static uint32_t getNextContext(uint32_t currentStaskPointer)
{
	osKernel.current_task->stackPointer = currentStaskPointer;

	if (osKernel.current_task->state == OS_TASK_RUNNING)
	{
		osKernel.current_task->state = OS_TASK_READY;
	}

	osKernel.current_task = osKernel.next_task;
	osKernel.current_task->state = OS_TASK_RUNNING;

	osKernel.current_osState = os_normalRun;

	return osKernel.current_task->stackPointer;
}

static void oskernel_createIdleTask(void)
{
	osTaskCreate(&Idle_Task, MAX_NUMBER_PRIORITY, osIdleTask);
	osKernel.number_tasks--;
}

static void osKernel_Scheduler(void)
{
	int max_qtypprior = 0;
	bool_t onetask_running = false;
	if (osKernel.current_osState == os_fromReset)
	{
		osKernel.current_task = &Idle_Task;
		osKernel.current_osState = os_normalRun;
	}

	if (osKernel.current_osState == os_scheduling)
		return;

	osKernel.current_osState = os_scheduling;

	/// Aca se busca la siguiente tareea a pasar a estado running

	for (int idx_prior = 0; idx_prior < REAL_NUMBER_PRIORITY; ++idx_prior)
	{
		if (osKernel.firstptr_tasks[idx_prior] != NULL)
		{
			max_qtypprior = osKernel.tasks_perprior[idx_prior];
			onetask_running = false;
			for (int i_qtypprior = 0; i_qtypprior < max_qtypprior; ++i_qtypprior)
			{

				if (osKernel.next_taskfind[idx_prior]->state == OS_TASK_RUNNING)
				{
					onetask_running = true;
				}
				if (osKernel.next_taskfind[idx_prior]->state == OS_TASK_READY)
				{
					osKernel.updateContext = true;
					osKernel.next_task = osKernel.next_taskfind[idx_prior];
					break;
				}
				else
				{
					osKernel.next_taskfind[idx_prior] = osKernel.next_taskfind[idx_prior]->next_task;
				}
			}
		}

		if (osKernel.updateContext == true)
			break;

		if (onetask_running == true)
			break;
	}
	///

	osKernel.current_osState = os_normalRun;

	if (osKernel.updateContext == true)
	{
		osKernel_setPendSV();
	}
}

static void osKernel_setPendSV(void)
{

	osKernel.updateContext = false;
	/*
	 * Set up bit corresponding exception PendSV
	 */
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

	/*
	 * Instruction Synchronization Barrier; flushes the pipeline and ensures that
	 * all previous instructions are completed before executing new instructions
	 */
	__ISB();

	/*
	 * Data Synchronization Barrier; ensures that all memory accesses are
	 * completed before next instruction is executed
	 */
	__DSB();
}

/**
 * @fn void countTaskDelay(void)
 * @brief Actualiza los contadores de todas las tareas
 *
 */
static void osKernel_UPDTtaskdelays(void)
{
	osTaskObject *checkTask;
	int max_qtypprior = 0;
	for (int idx_prior = 0; idx_prior < MAX_NUMBER_PRIORITY; ++idx_prior)
	{
		if (osKernel.firstptr_tasks[idx_prior] != NULL)
		{
			max_qtypprior=osKernel.tasks_perprior[idx_prior];
			checkTask=osKernel.firstptr_tasks[idx_prior];
			for (int i_qtypprior = 0; i_qtypprior < max_qtypprior; ++i_qtypprior) {
				if (checkTask->active_delay) {
					if (checkTask->ticks_blocked>0) {
						checkTask->ticks_blocked--;
					}
					if (checkTask->state == OS_TASK_BLOCK && checkTask->ticks_blocked == 0) {
						checkTask->state = OS_TASK_READY;
						checkTask->active_delay=false;
					}
				}
				else if (checkTask->active_timeout) {
					if (checkTask->ticks_timeout>0) {
						checkTask->ticks_timeout--;
					}
					if (checkTask->state == OS_TASK_BLOCK && checkTask->ticks_timeout == 0) {
						checkTask->state = OS_TASK_READY;
						checkTask->active_timeout=false;
					}
				}
				checkTask = checkTask->next_task;
			}
		}
	}
}
/******************************************************************************
 * Public Function Definitions
 *******************************************************************************/

bool osTaskCreate(osTaskObject *handler, osPriorityType priority, void *callback)
{
	/// Validates the total number of tasks in the RTOS
	if (!(osKernel.number_tasks < MAX_NUMBER_TASK))
	{
		return false;
	}

	/// xPSR value with 24 bit on one (Thumb mode).
	handler->memory[MAX_STACK_SIZE / 4 - XPSR_REG_POSITION] = XPSR_VALUE;
	/// Program pointer (PC) points to function used by the task.
	handler->memory[MAX_STACK_SIZE / 4 - PC_REG_POSTION] = (uint32_t) callback;
	/// If occur some problem and task executed return so after that execute this function.

	handler->memory[MAX_STACK_SIZE / 4 - LR_REG_POSTION] = (uint32_t) osReturnTaskHook;
	/*
	 * The previous value of LR (which is EXEC_RETURN in this case) is necessary because,
	 * in this implementation, a function is called from within the PendSV handler,
	 * causing the LR value to be modified to the return address by the time
	 * getContextoSiguiente finishes executing.
	 */
	handler->memory[MAX_STACK_SIZE / 4 - LR_PREV_VALUE_POSTION] = EXEC_RETURN_VALUE;

	handler->stackPointer = (uint32_t) (handler->memory + MAX_STACK_SIZE / 4 - SIZE_STACK_FRAME);
	handler->entryPoint = callback;
	handler->task_return_hook = NULL;	/// not used, but an error can be configured for each task
	handler->priority = priority;
	handler->state = OS_TASK_READY;
	handler->ticks_blocked = ZERO_TICKS;

	handler->active_delay=false;
	handler->active_timeout=false;
	handler->ticks_timeout= ZERO_TICKS;

	osKernel.tasks_perprior[priority]++;
	osKernel.number_tasks++;

	/// Allows creating a circular linked list for each priority level
	if (osKernel.firstptr_tasks[handler->priority] == NULL)
	{
		osKernel.firstptr_tasks[handler->priority] = handler;
	}

	handler->next_task = osKernel.firstptr_tasks[handler->priority];
	if (osKernel.lastptr_tasks[handler->priority] != NULL)
	{
		osKernel.lastptr_tasks[handler->priority]->next_task = handler;
	}
	osKernel.lastptr_tasks[handler->priority] = handler;

	return true;
}

void osStart(void)
{
	NVIC_DisableIRQ(SysTick_IRQn);
	NVIC_DisableIRQ(PendSV_IRQn);

	oskernel_createIdleTask();

	osKernel.current_osState = os_fromReset;
	osKernel.current_task = NULL;
	osKernel.next_task = NULL;

	for (int var = 0; var < REAL_NUMBER_PRIORITY; ++var)
	{
		osKernel.next_taskfind[var] = osKernel.firstptr_tasks[var];
	}
	/*
	 * All interrupts has priority 0 (maximum) at start execution. For that don't happen fault
	 * condition, we have to less priotity of NVIC. This math calculation showing take lowest
	 * priority possible.
	 */
	NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	/* Activate and configure the time of Systick exception */
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / (1000U * SYSTICK_PERIOD_MS));

	NVIC_EnableIRQ(PendSV_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}

void osDelay(const uint32_t n_tick)
{
	// Puntero a la tarea que se encuentra en running actualmente
	osTaskObject *running_task;

	// Si se encuentra en ISR devuelve a error
//	if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0) {
//		osErrorHook(NULL);
//	}
	if (osKernel.current_osState == os_irqUpdt)
	{
		osErrorHook(NULL);
	}

	if (n_tick != 0)
	{
		osEnterCriticalSection();
		running_task = osKernel.current_task;
		running_task->ticks_blocked = n_tick;
		running_task->active_delay=true;
		osExitCriticalSection();

		if (running_task == &Idle_Task || running_task == NULL)
		{
			osErrorHook(NULL);
		}

		running_task->state = OS_TASK_BLOCK;
		osKernel_YieldCPU();
	}

}

/* ========== Processor Interruption and Exception Handlers ========= */

void SysTick_Handler(void)
{
	osKernel_UPDTtaskdelays();

	osKernel_Scheduler();

	osSysTickHook();

}

/**
 * @fn void osEnterCriticalSection(void)
 */
void osEnterCriticalSection(void)
{
	__disable_irq();			/// Deshabilita todas las interrupciones
	osKernel.critical_counter++;	/// Incrementa el contador de anidacion critica
}

/**
 * @fn void osExitCriticalSection(void)
 */
void osExitCriticalSection(void)
{
	osKernel.critical_counter--;			///	El contador de anidaciones disminuye
	if (osKernel.critical_counter < 1)
	{	///	y verifica si ya no hay mas anidacion
		osKernel.critical_counter = 0;		///	para vovler a activar las interrupciones
		__enable_irq();
	}
}

void osKernel_YieldCPU(void)
{
	osKernel_Scheduler();
}

os_State osKernel_GetStateStep(void)
{
	return osKernel.current_osState;
}

void osKernel_SETschdlrINirq(void)
{
	osKernel.schdlrINirq=true;
}

void osKernel_UNSETschdlrINirq(void)
{
	osKernel.schdlrINirq=false;
}

bool_t osKernel_GETschdlrINirq(void)
{
	return osKernel.schdlrINirq;
}
/**
 * @fn osTaskObject osGetCurrentTask*(void)
 */
osTaskObject* osGetCurrentTask(void)
{
	return osKernel.current_task;
}

void osKernel_SaveState(void)
{
	osKernel.previous_osState=osKernel.current_osState;
}

void osKernel_RecoverState(void)
{
	osKernel.current_osState=osKernel.previous_osState;
}

void osSetRESETState()
{
	osKernel.current_osState=os_fromReset;
}
void osSetNORMALState()
{
	osKernel.current_osState=os_normalRun;
}

void osSetSCHEDULINGState()
{
	osKernel.current_osState=os_scheduling;
}
void osSetIRQState()
{
	osKernel.current_osState=os_irqUpdt;
}



__attribute__ ((naked)) void PendSV_Handler(void)
{
	// Se entra a la seccion critica y se deshabilita las interrupciones.
	__ASM volatile ("cpsid i");

	/**
	 * Implementación de stacking para FPU:
	 *
	 * Las tres primeras corresponden a un testeo del bit EXEC_RETURN[4]. La instruccion TST hace un
	 * AND estilo bitwise (bit a bit) entre el registro LR y el literal inmediato. El resultado de esta
	 * operacion no se guarda y los bits N y Z son actualizados. En este caso, si el bit EXEC_RETURN[4] = 0
	 * el resultado de la operacion sera cero, y la bandera Z = 1, por lo que se da la condicion EQ y
	 * se hace el push de los registros de FPU restantes
	 */
	__ASM volatile ("tst lr, 0x10");
	__ASM volatile ("it eq");
	__ASM volatile ("vpusheq {s16-s31}");

	/**
	 * Cuando se ingresa al handler de PendSV lo primero que se ejecuta es un push para
	 * guardar los registros R4-R11 y el valor de LR, que en este punto es EXEC_RETURN
	 * El push se hace al reves de como se escribe en la instruccion, por lo que LR
	 * se guarda en la posicion 9 (luego del stack frame). Como la funcion getNextContext
	 * se llama con un branch con link, el valor del LR es modificado guardando la direccion
	 * de retorno una vez se complete la ejecucion de la funcion
	 * El pasaje de argumentos a getContextoSiguiente se hace como especifica el AAPCS siendo
	 * el unico argumento pasado por RO, y el valor de retorno tambien se almacena en R0
	 *
	 * NOTA: El primer ingreso a este handler (luego del reset) implica que el push se hace sobre el
	 * stack inicial, ese stack se pierde porque no hay seguimiento del MSP en el primer ingreso
	 */
	__ASM volatile ("push {r4-r11, lr}");
	__ASM volatile ("mrs r0, msp");
	__ASM volatile ("bl %0" :: "i"(getNextContext));
	__ASM volatile ("msr msp, r0");
	__ASM volatile ("pop {r4-r11, lr}");
	//Recuperados todos los valores de registros

	/**
	 * Implementación de unstacking para FPU:
	 *
	 * Habiendo hecho el cambio de contexto y recuperado los valores de los registros, es necesario
	 * determinar si el contexto tiene guardados registros correspondientes a la FPU. si este es el caso
	 * se hace el unstacking de los que se hizo PUSH manualmente.
	 */
	__ASM volatile ("tst lr,0x10");
	__ASM volatile ("it eq");
	__ASM volatile ("vpopeq {s16-s31}");

	// Se sale de la seccion critica y se habilita las interrupciones.
	__ASM volatile ("cpsie i");

	/* Se hace un branch indirect con el valor de LR que es nuevamente EXEC_RETURN */
	__ASM volatile ("bx lr");
}

__attribute__((weak)) void osReturnTaskHook(void)
{
	while (1)
	{
		__WFI();
	}
}

__attribute__((weak)) void osIdleTask(void)
{
	while (1)
	{
	}
}

__attribute__((weak)) void osSysTickHook(void)
{
	__ASM volatile ("nop");
}

__attribute__((weak)) void osErrorHook(void *caller)
{
	while (1)
	{
	}
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*************** END OF FUNCTIONS *********************************************/
