/************************************************************************************* 
 * @file 	 hw_core.c
 * @date   March, 12 2023
 * @author Awatsa Hermann
 * @brief	 Interface to access the core hardware functionalities
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "hw_core.hpp"


/**
 * \defgroup core Core
 * \brief Core functionalities and utilities. These functionalities are
 * designed to provide common services and access to core hardware features
 * 
 * @{
 * 
 * \defgroup hardwareCore Hardware Core Features
 * \ingroup core
 * Interface to access the core hardware functionalities
 * 
 * @{
 */

/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

/**
 * \brief Enable an interrupt
 * 
 * \param [in] IRQn : interrupt number
 */
void hw_core::irq_enable(IRQn_t IRQn)
{
    if (!__NVIC_GetEnableIRQ(IRQn))
    {
      __NVIC_EnableIRQ(IRQn);
    }
}


/**
 * \brief Disable an interrupt
 * 
 * \param [in] IRQn : interrupt number
 */
void hw_core::irq_disable(IRQn_t IRQn)
{
    while (__NVIC_GetActive(IRQn))
    {
      /* wait till the interrupt release the active state */
    }
    
    __NVIC_DisableIRQ(IRQn);
}


/**
 * \brief Set an interrupt priority
 * 
 * \param [in] IRQn : interrupt number
 * \param [in] u8Priority : interrupt priority
 */
void hw_core::irq_setPriority(IRQn_t IRQn, u8 priority)
{
  NVIC_SetPriority(IRQn, (u32)priority);
}


/**
 * \brief Clear a pending interrupt
 * 
 * \param [in] IRQn : interrupt number
 */
void hw_core::irq_clearPending(IRQn_t IRQn)
{
    if (__NVIC_GetPendingIRQ(IRQn))
    {
      __NVIC_ClearPendingIRQ(IRQn);
    }
}


/**
 * \brief disable all global interrupts
 */
void hw_core::disable_all_irq(void)
{
    __disable_irq();
}


/**
 * \brief disable the faults exceptions
 */
void hw_core::disable_fault_irq(void)
{
    __disable_fault_irq();
}


/**
 * \brief Trigger a system reset
 */
__attribute__((noreturn)) void hw_core::systemReset(void)
{
  __NVIC_SystemReset();
}

void hw_core::initTicks(u32 ticks)
{
    SysTick_Config(ticks);
}

/** @} */

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
