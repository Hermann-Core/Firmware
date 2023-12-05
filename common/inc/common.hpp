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
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

/**
 * \defgroup commonUtilities Common Functions & Utilities
 * \brief provides a set of common utility functions for handling various operations.
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "assert.h"
#include "features.h"
#include "checker.hpp"
#include "operators.hpp"
#include "common_types.h"
#include "drivers_const.hpp"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace common
{

    /**
     * \brief Set specific bits in a register at a given position.
     *
     * \param [in] reg   : Reference to the register.
     * \param [in] value : Value to set in the register.
     * \param [in] pos   : Position of the bits to set.
     */
    constexpr void set_reg_bits(volatile u32& reg, u32 value, u32 pos)
    {
        if (!(reg & (value << pos)))
        {
            reg |= (value << pos);
        }
    }

    /**
     * \brief Set bits in a register based on a provided mask.
     *
     * \param [in] reg  : Reference to the register.
     * \param [in] mask : Mask indicating the bits to set.
     */
    constexpr void set_reg_bits(volatile u32& reg, u32 mask)
    {
        if (!(reg & mask))
        {
            reg |= mask;
        }
    }

    /**
     * \brief Reset specific bits in a register at a given position.
     *
     * \param [in] reg   : Reference to the register.
     * \param [in] value : Value indicating the bits to reset.
     * \param [in] pos   : Position of the bits to reset.
     */
    constexpr void reset_reg_bits(volatile u32& reg, u32 value, u32 pos)
    {
        reg &= ~(value << pos);
    }

    /**
     * \brief Reset bits in a register based on a provided mask.
     *
     * \param [in] reg  : Reference to the register.
     * \param [in] mask : Mask indicating the bits to reset.
     */
    constexpr void reset_reg_bits(volatile u32& reg, u32 mask)
    {
        reg &= ~mask;
    }

    /**
     * @brief Fills a range with a constant value.
     *
     * @tparam Iterator  : Type of the iterators defining the range.
     * @tparam T         : Type of the value to be assigned.
     * @param [in] first : Iterator pointing to the beginning of the range.
     * @param [in] last  : Iterator pointing to the end of the range.
     * @param [in] value : The value to be assigned to each element in the range.
     */
    template <typename Iterator, typename T>
    constexpr void fill(Iterator first, Iterator last, const T& value)
    {
        while (first != last) {
            *first++ = value;
        }
    }

    /**
     * @brief Fills a specified number of elements in a range with a constant value.
     *
     * @tparam Iterator : Type of the iterator defining the start of the range.
     * @tparam Size     : Type representing the number of elements to be filled.
     * @tparam T        : Type of the value to be assigned.
     * @param [in] first : Iterator pointing to the beginning of the range.
     * @param [in] count : Number of elements to be filled with the value.
     * @param [in] value : The value to be assigned to each element in the range.
     */
    template <typename Iterator, typename Size, typename T>
    constexpr void fill(Iterator first, Size count, const T& value)
    {
        while (count != 0) {
            *first++ = value;
            --count;
        }
    }
};

/** @} */

#endif      /* _COMMON_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
