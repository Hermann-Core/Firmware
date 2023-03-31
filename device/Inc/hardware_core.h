/************************************************************************************* 
 * @file 	   hardware_core.h
 * @date       March, 12 2023
 * @author     AWATSA HERMANN
 * @brief	   Core hardware header file
 * 
 *             Contains the declarations of
               the core hardware functions
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.12   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _HARDWARE_CORE_
#define _HARDWARE_CORE_

/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "peripherals_defs.h"



/************************************************************************************#
|                              FUNCTIONS DECLARATIONS                                |
#************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Enable an interrupt
 * 
 * @param IRQn interrupt number
 */
void IRQ_Enable(IRQn_t IRQn);


/**
 * @brief Set an interrupt priority
 * 
 * @param IRQn interrupt number
 * @param u8Priority interrupt priority
 */
void IRQ_SetPriority(IRQn_t IRQn, u8 priority);


/**
 * @brief Clear a pending interrupt
 * 
 * @param IRQn interrupt number
 */
void IRQ_ClearPending(IRQn_t IRQn);


/**
 * @brief Disable an interrupt
 * 
 * @param IRQn interrupt number
 */
void IRQ_Disable(IRQn_t IRQn);

#ifdef __cplusplus
}
#endif


#endif      /* _HARDWARE_CORE_ */

/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
