/************************************************************************************* 
 * @file   common.hpp
 * @date   Nov, 28 2023
 * @author Awatsa Hermann
 * @brief  common types and functions
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.28.11   |    1      |  0         |

*************************************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

/**
 * @class common types and functions class
 */
class common
{
    public:

        inline static void set_reg_bits(volatile u32& reg, u32 value, u32 pos)
        {
            if (!(reg & (value << pos)))
            {
                reg |= (value << pos);
            }
        }

        inline static void set_reg_bits(volatile u32& reg, u32 mask)
        {
            if (!(reg & mask))
            {
                reg |= mask;
            }
        }

        inline static void reset_reg_bits(volatile u32& reg, u32 value, u32 pos)
        {
            reg &= ~(value << pos);
        }

        inline static void reset_reg_bits(volatile u32& reg, u32 mask)
        {
            reg &= ~mask;
        }
};


#endif      /* _COMMON_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
