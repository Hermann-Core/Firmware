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
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

#ifndef _OPERATORS_H_
#define _OPERATORS_H_

/**
 * \defgroup operator Operators
 * \ingroup common
 * Declarations of operators
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"


/*==================================================================================
|                             FUNCTIONS DECLARATIONS                                
===================================================================================*/

/**
 * \brief Converts a value to megahertz.
 *
 * \param [in] val : Value to be converted.
 * \return Converted value in megahertz.
 */
constexpr u32 operator"" _mhz(uint64_t val)
{
    return static_cast<uint32_t>(val * 1'000'000);
}

/**
 * \brief Converts a value to kilohertz.
 *
 * \param [in] val : Value to be converted.
 * \return Converted value in kilohertz.
 */
constexpr u32 operator"" _khz(uint64_t val)
{
    return static_cast<uint32_t>(val * 1'000);
}

/**
 * \brief Converts a value to hertz.
 *
 * \param [in] val : Value to be converted.
 * \return Converted value in hertz.
 */
constexpr u32 operator"" _hz(uint64_t val)
{
    return static_cast<uint32_t>(val);
}

/** @} */

#endif      /* _OPERATORS_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
