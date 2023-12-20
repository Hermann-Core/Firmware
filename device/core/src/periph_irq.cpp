/************************************************************************************* 
 * @file 	 periph_irq.c
 * @date   05, March 2023
 * @author Awatsa Hermann
 * @brief	 Peripherals interrupts handlers
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.05   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/**
 * \defgroup peripheralIRQ Peripherals IRQs
 * \ingroup core
 * Provides handlers for Cortex M4 specific exceptions and STM32G4/F303 specific
 * interrupts.
 * 
 * @{
 */
/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "periph_irq.h"



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define noreturn_ccmram      __attribute__((noreturn, section(".ccmram")))
#define section_ccmram       __attribute__((section(".ccmram")))



/*====================================================================================
|                        Cortex m4 specific exceptions handlers                      |
====================================================================================*/

/**
  * \brief This function handles the non maskable interrupts
  */
noreturn_ccmram void NMI_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the hardfault exception
  */
noreturn_ccmram void HardFault_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the memory manage exception
  */
noreturn_ccmram void MemManage_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the busfault exception
  */
noreturn_ccmram void BusFault_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the usage fault exception
  */
noreturn_ccmram void UsageFault_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the supervisor call exception
  */
noreturn_ccmram void SVC_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the debug monitor exception
  */
noreturn_ccmram void DebugMon_Handler(void)
{
  while (true);
}

/**
  * \brief This function is used to handle the Pending supervisor exception
  */
noreturn_ccmram void PendSV_Handler(void)
{
  while (true);
}

/**
  * \brief This function handles the systick timer.
  */
section_ccmram void SysTick_Handler(void)
{
  /* will be implement later */
}


/*====================================================================================
|                    STM32G4 and F303 specific interrupts handlers                   |
====================================================================================*/

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
