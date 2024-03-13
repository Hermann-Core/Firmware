/************************************************************************************* 
 * @file 	 boot_jump.cpp
 * @date   April, 02 2023
 * @author Awatsa Hermann
 * @brief	 bootloader jump source file
 * 
 *         Contains the functions used to jump to the main application
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.02   |    1      |  0         |

*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "boot_jump.hpp"
#include "hw_core.hpp"



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define unused             __attribute__((unused))
#define naked              __attribute__((naked, noreturn))
#define no_return          __attribute__((noreturn))


/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

/**
 * \brief Used to jump to the application from bootloader
 * 
 * \param appAddress the vector address of the application
 */
no_return void boot_jump::jumpToApp(const u32* appVector)
{
    extern const u32 APP_ADDRESS;   /* defined in the linker script */
#if defined (STM32F303)
    constexpr u8 MAX_IRQ_NUMBERS = 81;
#elif defined (STM32G474)
    constexpr u8 MAX_IRQ_NUMBERS = 101;
#endif

    /* Disable global and faults interrupts */
    hw_core::disable_all_irq();
    hw_core::disable_fault_irq();

    for (u8 i = 0; i < MAX_IRQ_NUMBERS; i++)
    {
      /* Disable and clear all pending interrupts */
      hw_core::irq_disable((IRQn_t)i);
      hw_core::irq_clearPending((IRQn_t)i);
    }

    /* Disable the systick and clear its exception pending */
    SysTick->CTRL   =   0x0;
    SysTick->LOAD   =   0x0;
    SysTick->VAL    =   0x0;
    SCB->ICSR      |=   SCB_ICSR_PENDSTCLR_Msk;

    //! deinit all the configured peripherals

    /* Set the vector table address for the application */
    SCB->VTOR = (u32)&APP_ADDRESS;

    /* Set the stack pointer and jump to the application */
    jumpASM(appVector[0], appVector[1]);

    while(true);
}


/**
 * \brief Set the stack pointer and branch to a function
 * 
 * \param SP the stack pointer
 * \param PC the program counter
 */
naked void boot_jump::jumpASM(unused u32 SP, unused u32 PC)
{
  asm volatile("MSR  MSP,r0");
  asm volatile("BX   r1");
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
