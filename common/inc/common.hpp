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
 * \addtogroup common
 * 
 * @{
 * 
 * \defgroup commonUtilities Common Functions & Utilities
 * \brief provides a set of common utility functions for handling various operations.
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "assert.h"
#include "types.h"
#include "const.hpp"
#include "features.h"
#include "checker.hpp"
#include "operators.hpp"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace common
{

    /**
     * \brief Set specific bits in a register at a given position.
     *
     * \param [in] reg   : reference to the register.
     * \param [in] value : value to set in the register.
     * \param [in] pos   : position of the bits to set.
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
     * \param [in] reg  : reference to the register.
     * \param [in] mask : mask indicating the bits to set.
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
     * \param [in] reg   : reference to the register.
     * \param [in] value : value indicating the bits to reset.
     * \param [in] pos   : position of the bits to reset.
     */
    constexpr void reset_reg_bits(volatile u32& reg, u32 value, u32 pos)
    {
        reg &= ~(value << pos);
    }

    /**
     * \brief Reset bits in a register based on a provided mask.
     *
     * \param [in] reg  : reference to the register.
     * \param [in] mask : mask indicating the bits to reset.
     */
    constexpr void reset_reg_bits(volatile u32& reg, u32 mask)
    {
        reg &= ~mask;
    }

    /**
     * \brief Read specific bit(s) in a register at a given position.
     *
     * \param [in] reg   : reference to the register.
     * \param [in] value : value indicating the bit(s) to read.
     * \param [in] pos   : position of the bits to read.
     * \return result of the read operation
     */
    constexpr u32 read_reg_bits(volatile u32 const& reg, u32 value, u32 pos)
    {
        return (reg & (value << pos));
    }

    /**
     * \brief Read bit(s) in a register based on a provided mask.
     *
     * \param [in] reg  : reference to the register.
     * \param [in] mask : mask indicating the bit(s) to read.
     * \return result of the read operation
     */
    constexpr u32 read_reg_bits(volatile u32 const& reg, u32 mask)
    {
        return (reg & mask);
    }

    /**
     * \brief Retrieve specific bit(s) from a register.
     *
     * \param [in] reg   : reference to the register.
     * \param [in] pos   : starting position of the bit(s) to retrieve
     * \param [in] numOfBits : number of bits to retrieve
     * \return retrieved bit(s)
     */
    constexpr u32 get_reg_bits(volatile u32 const& reg, u32 pos, u32 numOfBits)
    {
        return ((reg & ((1U << numOfBits) - 1U)) >> pos);
    }

    /**
     * @brief Fills a range with a constant value.
     *
     * @tparam Iterator  : type of the iterators defining the range.
     * @tparam T         : type of the value to be assigned.
     * @param [in] first : iterator pointing to the beginning of the range.
     * @param [in] last  : iterator pointing to the end of the range.
     * @param [in] value : the value to be assigned to each element in the range.
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
     * @tparam Iterator : type of the iterator defining the start of the range.
     * @tparam Size     : type representing the number of elements to be filled.
     * @tparam T        : type of the value to be assigned.
     * @param [in] first : iterator pointing to the beginning of the range.
     * @param [in] count : number of elements to be filled with the value.
     * @param [in] value : the value to be assigned to each element in the range.
     */
    template <typename Iterator, typename Size, typename T>
    constexpr void fill(Iterator first, Size count, const T& value)
    {
        while (count != 0) {
            *first++ = value;
            --count;
        }
    }

    /**
     * \brief Count the number of digits in an integer
     * 
     * \param [in] value : integer value
     * \return number of digits 
     */
    inline size_t count_digits(int value)
    {
        u8 count = 0;
        do
        {
            value /= 10;
            count++;
        } while (value != 0);

        return count;
    }
};

/** @} */

/** @} */

#endif      /* _COMMON_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
