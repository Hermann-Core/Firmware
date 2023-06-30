/************************************************************************************* 
 * @file 	 peripherals_irqs.c
 * @date   05, March 2023
 * @author AWATSA HERMANN
 * @brief	 Peripherals interrupts source file
 * 
 *         Contains the definitions of the MCU
 *         peripherals interrupts handlers
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.05   |    1      |  0         |

*************************************************************************************/

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "peripherals_irqs.h"
#include "peripherals_defs.h"



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define NORETURN_CCMRAM      __attribute__((noreturn, section(".ccmram")))
#define SECTION_CCMRAM       __attribute__((section(".ccmram")))



/*====================================================================================
|                        Cortex m4 specific exceptions handlers                      |
====================================================================================*/

/**
  * @brief This function handles the non maskable interrupts
  */
NORETURN_CCMRAM void NMI_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the hardfault exception
  */
NORETURN_CCMRAM void HardFault_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the memory manage exception
  */
NORETURN_CCMRAM void MemManage_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the busfault exception
  */
NORETURN_CCMRAM void BusFault_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the usage fault exception
  */
NORETURN_CCMRAM void UsageFault_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the supervisor call exception
  */
NORETURN_CCMRAM void SVC_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the debug monitor exception
  */
NORETURN_CCMRAM void DebugMon_Handler(void)
{
  while (1);
}

/**
  * @brief This function is used to handle the Pending supervisor exception
  */
NORETURN_CCMRAM void PendSV_Handler(void)
{
  while (1);
}

/**
  * @brief This function handles the systick timer.
  */
SECTION_CCMRAM void SysTick_Handler(void)
{
  /* will be implement later */
}


/*====================================================================================
|                    STM32G4 and F303 specific interrupts handlers                   |
====================================================================================*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
