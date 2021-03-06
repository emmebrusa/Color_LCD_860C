#include "fault.h"

/** from https://electronics.stackexchange.com/questions/293772/finding-the-source-of-a-hard-fault-using-extended-hardfault-handler
 * HardFault_HandlerAsm:
 * Alternative Hard Fault handler to help debug the reason for a fault.
 * To use, edit the vector table to reference this function in the HardFault vector
 * This code is suitable for Cortex-M3 and Cortex-M0 cores
 */

// Use the 'naked' attribute so that C stacking is not used.
__attribute__((naked))
void HardFault_Handler(void){
        /*
         * Get the appropriate stack pointer, depending on our mode,
         * and use it as the parameter to the C handler. This function
         * will never return
         */

        __asm(  ".syntax unified\n"
                        "MOVS   R0, #4  \n"
                        "MOV    R1, LR  \n"
                        "TST    R0, R1  \n"
                        "BEQ    _MSP    \n"
                        "MRS    R0, PSP \n"
                        "B      HardFault_HandlerC      \n"
                "_MSP:  \n"
                        "MRS    R0, MSP \n"
                        "B      HardFault_HandlerC      \n"
                ".syntax divided\n") ;
}

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void HardFault_HandlerC(unsigned long *hardfault_args){

	volatile unsigned long stacked_pc = ((unsigned long)hardfault_args[6]) ;
#if 0
	volatile unsigned long stacked_r0 = ((unsigned long)hardfault_args[0]) ;
	volatile unsigned long stacked_r1 = ((unsigned long)hardfault_args[1]) ;
	volatile unsigned long stacked_r2 = ((unsigned long)hardfault_args[2]) ;
	volatile unsigned long stacked_r3 = ((unsigned long)hardfault_args[3]) ;
	volatile unsigned long stacked_r12 = ((unsigned long)hardfault_args[4]) ;
	volatile unsigned long stacked_lr = ((unsigned long)hardfault_args[5]) ;
	volatile unsigned long stacked_psr = ((unsigned long)hardfault_args[7]) ;
	volatile unsigned long _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
	volatile unsigned long _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
	volatile unsigned long _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
	volatile unsigned long _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
	volatile unsigned long _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
	volatile unsigned long _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
#endif

	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR

	// Hard Fault Status Register

	// Debug Fault Status Register

	// Auxiliary Fault Status Register

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	//_MMAR
	// Bus Fault Address Register
	//_BFAR

	app_error_fault_handler(FAULT_HARDFAULT, stacked_pc, 0);
}
