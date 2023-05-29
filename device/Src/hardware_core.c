/************************************************************************************* 
 * @file 	   hardware_core.c
 * @date     March, 12 2023
 * @author   AWATSA HERMANN
 * @brief	   Hardware core source file
 * 
 *           Contains the definitions of
 *           the core hardware functions
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.12   |    1      |  0         |

*************************************************************************************/

/************************************************************************************/
/*                                     INCLUDES                                     */
/************************************************************************************/
#include "hardware_core.h"


/************************************************************************************/
/*                             FUNCTIONS DEFINITIONS                                */
/************************************************************************************/

void IRQ_Enable(IRQn_t IRQn)
{
    if (!__NVIC_GetEnableIRQ(IRQn))
    {
      NVIC_EnableIRQ(IRQn);
    }
}


void IRQ_SetPriority(IRQn_t IRQn, u8 priority)
{
    NVIC_SetPriority(IRQn, (u32)priority);
}


void IRQ_ClearPending(IRQn_t IRQn)
{
    if (__NVIC_GetPendingIRQ(IRQn))
    {
      NVIC_ClearPendingIRQ(IRQn);
    }
}


void IRQ_Disable(IRQn_t IRQn)
{
    while (__NVIC_GetActive(IRQn))
    {
      /* wait till the interrupt quit active state */
    }
    
    NVIC_DisableIRQ(IRQn);
}


__attribute__((noreturn)) void SystemReset(void)
{
    __NVIC_SystemReset();
}


/************************************************************************************/
/*                                   END OF FILE                                    */
/************************************************************************************/
