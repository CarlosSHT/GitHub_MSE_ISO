/*******************************************************************************
 * Title                 :   ------
 * Filename              :   osKernel.c
 * Author                :   Carlos Herrera Trujillo
 * Origin Date           :   Oct 5, 2023
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
#include <stddef.h>
#include "osKernel.h"
#include "system_stm32f4xx.h"
#include "stm32f429xx.h"
#include "core_cm4.h"
#include "cmsis_gcc.h"

/******************************************************************************
 * Private defines
 *******************************************************************************/
#define FIRST_INDEX_TASK_PRIORITY     0U

/******************************************************************************
 * Private Typedefs
 *******************************************************************************/
typedef struct {
//    osTaskObject*   listTask[MAX_NUMBER_TASK];  // Task list.
	osTaskObject *currentTask;		// Current handler task running.
	osTaskObject *nextTask;			// Next handler task will be run.
	uint8_t countTask;				// Number of task created.
	bool running;					// Status task, if it is running true in otherwise false.

	// Matriz de prioridad con los punteros de las tareas
	osTaskObject *priority_matrix[MAX_NUMBER_PRIORITY + 1][MAX_NUMBER_TASK];
	uint8_t ntask_priority[MAX_NUMBER_PRIORITY + 1];
	uint8_t ntask_idxoffset[MAX_NUMBER_PRIORITY + 1];

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
static osKernelObject osKernel = { 0 };
static osTaskObject Idle_Task;

/******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/
static void scheduler(void);
static uint32_t getNextContext(uint32_t currentStaskPointer);

static void TaskFSMstate(osTaskStatusType *state, bool_t wait, bool_t terminate);
static void CreateIdleTask(void);
static osTaskObject* getRunningTask(void);
static void countTaskDelay(void);
static void SysShieldCore(void);

/******************************************************************************
 * Public Function Prototypes
 *******************************************************************************/

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Definitions/////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * Private Function Definitions
 *******************************************************************************/

/**
 * @brief Get the task that must be run.
 *
 * @return Returns true if a new task to be executed.
 */
static void scheduler(void)
{
    uint8_t	n_task = 0, n_priority = 0;
	uint8_t abs_idx = 0;
    uint8_t bin_state = 0;

    bool_t one_exist = false, go_idle = true;

    // Se recorre cada fila de prioridad
    for (n_priority = 0; n_priority < MAX_NUMBER_PRIORITY; ++n_priority) {

        // hago un recorrido agrupando los estados en un resultado
        for (n_task = 0; n_task < MAX_NUMBER_TASK; ++n_task) {

//        	if (osKernel.priority_matrix[n_priority][n_task]->state == OS_TASK_BLOCK && !osKernel.priority_matrix[n_priority][n_task]->active_block) {
//    	        TaskFSMstate(&(osKernel.priority_matrix[n_priority][n_task]->state), osKernel.priority_matrix[n_priority][n_task]->active_block, osKernel.priority_matrix[n_priority][n_task]->active_terminate);
//			}

        	bin_state = bin_state | osKernel.priority_matrix[n_priority][n_task]->state;
    	}

        // Verifico si hay alguna tarea ready o running
        if ((bin_state & 0b0011) !=0) {

        	// Como hay al menos una tarea running o ready ya no se va al idle
        	go_idle = false;

			// Termina busqueda de ready
			one_exist = true;

        	// Busco el primer ready de acuerdo al offset de indice (el indice es relativo)
        	for (n_task = osKernel.ntask_idxoffset[n_priority]; n_task < MAX_NUMBER_TASK + osKernel.ntask_idxoffset[n_priority]; ++n_task)
        	{
        		// Indice absoluto del numero de tarea
        		abs_idx=n_task % MAX_NUMBER_TASK;

        		// Verifico que sea una tarea
    			if (osKernel.priority_matrix[n_priority][abs_idx]!=NULL && osKernel.priority_matrix[n_priority][abs_idx]->state == OS_TASK_READY)
    			{
					// Cargo la siguiente tarea que debe ir a running
					osKernel.nextTask = osKernel.priority_matrix[n_priority][abs_idx];
    				// Guardo el nuevo indice relativo de tarea para la fila prioridad
					osKernel.ntask_idxoffset[n_priority] = (abs_idx + 1) %  MAX_NUMBER_TASK;


					// break del for
					break;
    			}
			}
    	}
        if (one_exist) break;
	}

    // Siguiente tarea es Idle
    if (go_idle) osKernel.nextTask = &Idle_Task;

    // Si es primera vez entonces carga la siguiente tarea como la actual sea tarea o sea idle
	if (!osKernel.running && osKernel.currentTask == NULL) {
		osKernel.currentTask = osKernel.nextTask;
	}
}

/**
 * @brief Get next context task.
 *
 * @param[in] currentStaskPointer Stack pointer of current task.
 *
 * @return Return stack pointer of new task to execute.
 */
static uint32_t getNextContext(uint32_t currentStaskPointer)
{
	// Si es primera iteracion entonces el sistema corre y el estado de la tarea paa de ready a running
	// Ninguna tarea nace en running
    if (!osKernel.running)
    {
			TaskFSMstate(&(osKernel.currentTask->state), osKernel.currentTask->active_block, osKernel.currentTask->active_terminate);
			osKernel.running = true;
    }
    else
    {
    	// Guardo el stackpointer de la tarea que esta corriendo en estado Running y pasa
    	// a estado Ready o Suspend o Block de acuerdo a sus flags internos
		osKernel.currentTask->stackPointer  = currentStaskPointer;
		if (osKernel.currentTask->state == OS_TASK_RUNNING) {
			TaskFSMstate(&(osKernel.currentTask->state), osKernel.currentTask->active_block, osKernel.currentTask->active_terminate);
		}

		// La siguiente tarea definida (Estado Ready) se actualiza como tarea actual
		// Y se cambia el estado de la tarea con la maquina de estados
		osKernel.currentTask            = osKernel.nextTask;
		TaskFSMstate(&(osKernel.currentTask->state), osKernel.currentTask->active_block, osKernel.currentTask->active_terminate);
	}

    return osKernel.currentTask->stackPointer;
}


/**
 * @fn void SysShieldCore(void)
 * @brief Se utiliza para una ejecucion forzada de cambio de contexto
 *
 */
static void SysShieldCore(void)
{
    scheduler();
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
 * @fn void TaskFSMstate(osTaskStatusType*, bool_t, bool_t)
 * @brief Maquina de estados FSM que actualiza el estado de una tarea utilizando los flags
 * de bloquear y suspender
 *
 * @param state
 * @param wait
 * @param terminate
 */
static void TaskFSMstate(osTaskStatusType *state, bool_t wait, bool_t terminate)
{
	switch (*state) {
		case OS_TASK_READY:
			*state=OS_TASK_RUNNING;
			break;
		case OS_TASK_RUNNING:
			*state=OS_TASK_READY;
			if (wait) {
				*state=OS_TASK_BLOCK;
				break;
			}
			if (terminate) {
				*state=OS_TASK_SUSPEND;
				break;
			}
			break;
		case OS_TASK_BLOCK:
			if (!wait) {
			*state=OS_TASK_READY;
			}
			break;
		case OS_TASK_SUSPEND:
				*state=OS_TASK_READY;
			break;
		default:
			break;
	}
}


/**
 * @fn void CreateIdleTask(void)
 * @brief Crea la tarea Idle
 *
 */
static void CreateIdleTask(void)
{
   osTaskCreate(&Idle_Task, OS_LOW_PRIORITY + 1, osIdleTask);
}


/**
 * @fn osTaskObject getRunningTask*(void)
 * @brief Obtiene la tarea actual que esta en estado running
 * Se considera si debe ser Idle la tarea en running
 *
 * @return
 */
static osTaskObject* getRunningTask(void)
 {
	osTaskObject *runningtask = NULL;

	for (int var = 0; var < MAX_NUMBER_PRIORITY + 1; ++var) {
		for (int var2a = 0; var2a < MAX_NUMBER_TASK; ++var2a) {
			if (osKernel.priority_matrix[var][var2a] != NULL
					&& osKernel.priority_matrix[var][var2a]->state
							== OS_TASK_RUNNING) {
				runningtask = osKernel.priority_matrix[var][var2a];
			}
		}
	}

	return runningtask;
}



/**
 * @fn void countTaskDelay(void)
 * @brief Actualiza los contadores de todas las tareas
 *
 */
static void countTaskDelay(void)
{
	// Recorre todas las tareas con 2 for
	for (int var = 0; var < MAX_NUMBER_PRIORITY; ++var) {
		for (int var2a = 0; var2a < MAX_NUMBER_TASK; ++var2a) {

			// Verifica si la tarea tiene un delay activo y reduce la cuenta si es mayor a 0
			if (osKernel.priority_matrix[var][var2a]!=NULL && osKernel.priority_matrix[var][var2a]->active_delay && osKernel.priority_matrix[var][var2a]->delay_time > 0)
			{
				osKernel.priority_matrix[var][var2a]->delay_time--;
			}

			// Si llega a 0 y está bloqueado entonces actualiza la tarea a estado ready y desactiva el delay y borra flag bloqueado
			if (osKernel.priority_matrix[var][var2a]->delay_time == 0 && osKernel.priority_matrix[var][var2a]->state==OS_TASK_BLOCK && osKernel.priority_matrix[var][var2a]->active_delay) {
		        osKernel.priority_matrix[var][var2a]->active_delay=false;
		        osKernel.priority_matrix[var][var2a]->active_block=false;
//		        osKernel.priority_matrix[var][var2a]->state = OS_TASK_READY;
				TaskFSMstate(&(osKernel.priority_matrix[var][var2a]->state), osKernel.priority_matrix[var][var2a]->active_block, osKernel.priority_matrix[var][var2a]->active_terminate);

			}
		}
	}
}
/******************************************************************************
 * Public Function Definitions
 *******************************************************************************/

/**
 * @fn bool osTaskCreate(osTaskObject*, osPriorityType, void*)
 */
bool osTaskCreate(osTaskObject *handler, osPriorityType priority,
		void *callback) {
	// Valida la cantidad de tareas por cada prioridad
	if (osKernel.ntask_priority[priority] >= MAX_NUMBER_TASK) {
		return false;
	}
//    if (osKernel.countTask >= MAX_NUMBER_TASK)
//    {
//        return false;
//    }

	// xPSR value with 24 bit on one (Thumb mode).
	handler->memory[MAX_STACK_SIZE / 4 - XPSR_REG_POSITION] = XPSR_VALUE;
	// Program pointer (PC) points to function used by the task.
	handler->memory[MAX_STACK_SIZE / 4 - PC_REG_POSTION] = (uint32_t) callback;
	// If occur some problem and task executed return so after that execute this function.
	handler->memory[MAX_STACK_SIZE / 4 - LR_REG_POSTION] =
			(uint32_t) osReturnTaskHook;

	/*
	 * Previous Link register (LR) value because handler pendSV call function inside exception
	 * and LR is overwrite with the return value of getNextContext.
	 */
	handler->memory[MAX_STACK_SIZE / 4 - LR_PREV_VALUE_POSTION] =
			EXEC_RETURN_VALUE;

	// Pointer function of task.
	handler->entryPoint = callback;
	handler->id = osKernel.countTask;
	handler->state = OS_TASK_READY;
	handler->priority = priority;
	handler->stackPointer = (uint32_t) (handler->memory + MAX_STACK_SIZE / 4
			- SIZE_STACK_FRAME);

	// Ninguna tarea nace bloqueada o suspendida
	handler->active_block = false;
	handler->active_terminate = false;
	handler->active_delay = false;

	// Se llena la matriz de prioridades
	osKernel.priority_matrix[priority][osKernel.ntask_priority[priority]] =
			handler;
	osKernel.ntask_priority[priority]++;
//    // Fill controls OS structure
//    osKernel.listTask[osKernel.countTask] = handler;
//    osKernel.countTask++;
//
//    // Ask to avoid overflow memory when fill element vector
//    if (osKernel.countTask < MAX_NUMBER_TASK)
//    {
//        osKernel.listTask[osKernel.countTask] = NULL;
//    }

	return true;
}

/**
 * @fn void osStart(void)
 */
void osStart(void) {
	NVIC_DisableIRQ(SysTick_IRQn);
	NVIC_DisableIRQ(PendSV_IRQn);

	osKernel.running = false;
	osKernel.currentTask = NULL;
	osKernel.nextTask = NULL;

	CreateIdleTask();
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

/**
 * @fn void osDelay(const uint32_t)
 */
void osDelay(const uint32_t tick) {
	// Puntero a la tarea que se encuentra en running actualmente
	osTaskObject *running_task;

	// Si se encuentra en ISR devuelve a error
	if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0) {
		osErrorHook(NULL);
	}

	// Funcion que obtiene la tarea que esa en estado running actualmente
//	running_task=getRunningTask();
	running_task = osKernel.currentTask;

	// El S.O. no hace nada si el delay es aplicado a la tarea IDLE
	if (running_task != NULL && running_task != &Idle_Task) {

		// Actualiza el estado de la tarea en el cambio de contexto
		if (!running_task->active_delay) {
			running_task->active_delay = true;
			running_task->active_block = true;
			running_task->delay_time = tick;
//			running_task->state = OS_TASK_BLOCK;
			// Actualiza la tarea de Running a Blocked
			TaskFSMstate(&(running_task->state), running_task->active_block,
					running_task->active_terminate);

			SysShieldCore();
		}

	}
}

/**
 * @fn void osReturnTaskHook(void)
 */
__attribute__((weak)) void osReturnTaskHook(void)
{
    while(1)
    {
        __WFI();
    }
}

/**
 * @fn void osSysTickHook(void)
 */
__attribute__((weak)) void osSysTickHook(void)
{
    __ASM volatile ("nop");
}

__attribute__((weak)) void osErrorHook(void* caller)
{
    while(1)
    {
    }
}

/**
 * @fn void osIdleTask(void)
 */
__attribute__((weak)) void osIdleTask(void)
{
    while(1)
    {
    }
}

/**
 * @fn osTaskObject osGetCurrentTask*(void)
 */
osTaskObject* osGetCurrentTask(void)
{
	return osKernel.currentTask;
}

/**
 * @fn void osBlockTask(osTaskObject*)
 */
void osBlockTask(osTaskObject* task_handler)
{
	task_handler->active_block = true;
	task_handler->state = OS_TASK_BLOCK;
	SysShieldCore();
}

/**
 * @fn void osUnblockTask(osTaskObject*)
 */
void osUnblockTask(osTaskObject* task_handler)
{
	task_handler->active_block = false;
	task_handler->state = OS_TASK_READY;
	SysShieldCore();
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
    scheduler();
    osSysTickHook();

    countTaskDelay();
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

__attribute__ ((naked)) void PendSV_Handler(void)
{
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
    __ASM volatile ("pop {r4-r11, lr}");    //Recuperados todos los valores de registros

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

    /* Se hace un branch indirect con el valor de LR que es nuevamente EXEC_RETURN */
    __ASM volatile ("bx lr");
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


/*************** END OF FUNCTIONS *********************************************/
