/************************************************************************************* 
 * @file   operators.hpp
 * @date   Nov, 27 2023
 * @author Awatsa Hermann
 * @brief  operators declarations file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.27.11   |    1      |  0         |

*************************************************************************************/

#ifndef _OPERATORS_H_
#define _OPERATORS_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"


/*==================================================================================
|                             FUNCTIONS DECLARATIONS                                
===================================================================================*/

constexpr u32 operator"" _mhz(uint64_t val)
{
    return static_cast<uint32_t>(val * 1'000'000);
}

constexpr u32 operator"" _khz(uint64_t val)
{
    return static_cast<uint32_t>(val * 1'000);
}

constexpr u32 operator"" _hz(uint64_t val)
{
    return static_cast<uint32_t>(val);
}


#endif      /* _OPERATORS_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
