/*
 * os_Kernel.c
 *
 *      Author: CARLOS
 */


#include "os_Kernel.h"
#include "system_stm32f4xx.h"
#include "stm32f429xx.h"
#include "core_cm4.h"
#include "cmsis_gcc.h"
#include <stdbool.h>
#include <stddef.h>



bool kernel_running=false;
uint8_t kernel_task_id=0;
osTaskObj* list_task[MAX_NUM_TASK];
osTaskObj* current_task;
osTaskObj* next_task;


static uint32_t getNextContext(uint32_t currentStaskPointer);


void osInit(void)
{
    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_DisableIRQ(PendSV_IRQn);

    kernel_running=false;
    kernel_task_id=0;
    current_task= NULL;
    next_task= NULL;

    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS)-1);

    NVIC_EnableIRQ(PendSV_IRQn);
    NVIC_EnableIRQ(SysTick_IRQn);
}


//1. Inicialización del stack frame para la primera vez que esa tarea es llamada.
void osTaskCreate(osTaskObj *task, void* func_cb, void *arg)
{
	task->stack_mem[STACK_SIZE/4-XPSRT_POS] = XPSRT_VAL;				/* xPSR.T = 1 */
	task->stack_mem[STACK_SIZE/4-PC_POS] = (uint32_t)func_cb;	/* PC */
//	task->stack_mem[STACK_SIZE/4-LR_POS] = 0;					/* LR */
	task->stack_mem[STACK_SIZE/4-R0_POS] = (uint32_t)arg;		/* R0 */
	task->stack_mem[STACK_SIZE/4-LR_IRQ_POS] = LR_IRQ_VAL;


	task->stack_ptr=(uint32_t)(task->stack_mem+STACK_SIZE/4-SIZE_STACK_FRAME);
	task->entry_point=(uint32_t)func_cb;
//	task->task_return_hook=0;
	task->arg=arg;/* LR IRQ */


	list_task[kernel_task_id]=task;
	kernel_task_id++;
	task->id=kernel_task_id;
}


static void sys_scheduler(void)
{
    uint8_t index = 0;

    // Is the first time that operating system execute? Yes, so I start with Task1
    if (!kernel_running) {
        current_task=list_task[0];
    }
    else
    {
        index = current_task->id;

        // If is the last task so I start againt with first.
        if (index > MAX_NUM_TASK || NULL == list_task[index])
        {
            index = 1;
        }
        next_task=list_task[index-1];
    }
}

static uint32_t getNextContext(uint32_t currentStaskPointer)
{
    if (!kernel_running)
    {
    	current_task->state = StateTask_Running;
    	kernel_running = true;
    }
    else
    {
    	current_task->stack_ptr = currentStaskPointer;
    	current_task->state = StateTask_Ready;

        current_task=next_task;
        current_task->state = StateTask_Running;
    }

    return current_task->stack_ptr;
}



// Cada vez que ocurre una excepción (PendSV) realiza un PUSH automático de un stack frame
__attribute__ ((naked)) void PendSV_Handler(void)
{
    __ASM volatile ("push {r4-r11, lr}");

    __ASM volatile ("mrs r0, msp");	// pasamos el valor del stack pointer a r0
    __ASM volatile ("bl %0" :: "i"(getNextContext));
    __ASM volatile ("msr msp, r0");

    __ASM volatile ("pop {r4-r11, lr}");    //Recuperados todos los valores de registros
    /* Se hace un branch indirect con el valor de LR que es nuevamente EXEC_RETURN */
    __ASM volatile ("bx lr");
}



void SysTick_Handler(void)
{
	sys_scheduler();

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
