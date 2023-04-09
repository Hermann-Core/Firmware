/************************************************************************************* 
 * @file 	   boot_jump.cpp
 * @date     April, 02 2023
 * @author   AWATSA HERMANN
 * @brief	   bootloader jump source file
 * 
 *           Contains the functions used to jump to the main application
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.02   |    1      |  0         |

*************************************************************************************/


/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "boot_jump.hpp"
#include "hardware_core.h"
#include "peripherals_defs.h"



/************************************************************************************#
|                                       DEFINES                                      |
#************************************************************************************/
#define SRAM_BASE          0x20000000

#define UNUSED             __attribute__((unused))
#define NAKED              __attribute__((naked, noreturn)) static
#define NO_RETURN          __attribute__((noreturn))

#if defined (STM32F303)
#define MAX_IRQ_NUMBERS    81
#elif defined (STM32G473)
#define MAX_IRQ_NUMBERS    101
#endif



/************************************************************************************#
|                              FUNCTIONS DECLARATIONS                                |
#************************************************************************************/
NAKED void JumpASM(UNUSED u32 SP, UNUSED u32 PC);



/************************************************************************************#
|                              FUNCTIONS DEFINITIONS                                 |
#************************************************************************************/

/**
 * \brief Used to jump to the application from bootloader
 * 
 * \param appAddress the starting address of the application
 */
NO_RETURN void JumpToApp(const u32 *appVector)
{
    __disable_irq();                  /* Disable global interrupts */
    __disable_fault_irq();            /* Disable fault exceptions handlers*/
    // TODO! Deinit all peripherals (which may trigger an interrupt) and clear all irq pendings flags
    for (u8 i = 0; i < MAX_IRQ_NUMBERS; i++)
    {
      IRQ_Disable((IRQn_t)i);         /* Disable all interrupts */
      IRQ_ClearPending((IRQn_t)i);    /* Clear all pendings requests in NVIC */
    }

    /* Disable the systick and clear its exception pending */
    SysTick->CTRL = 0x0;
    SysTick->LOAD = 0x0;
    SysTick->VAL  = 0x0;
    SCB->ICSR    |= SCB_ICSR_PENDSTCLR_Msk;

    /* Set the vector table address for the application */
    SCB->VTOR = SRAM_BASE;

    /* Set the stack pointer and jump to the application */
    JumpASM(appVector[0], appVector[1]);
}


/**
 * \brief Set the stack pointer and branch to an address
 * 
 * \param SP the stack pointer
 * \param PC the program counter
 */
NAKED void JumpASM(UNUSED u32 SP, UNUSED u32 PC)
{
  asm volatile("MSR  MSP,r0");
  asm volatile("BX   r1");
}


/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
