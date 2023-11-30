/************************************************************************************* 
 * @file   checker.hpp
 * @date   Nov, 28 2023
 * @author Awatsa Hermann
 * @brief  this file contains the checker functions
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

#ifndef _CHECKER_H_
#define _CHECKER_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

template <typename T = u32>
class checker
{
    public:

        /**
         * @brief check if a value is within a specified range 
         * 
         * @param [in] value     : value
         * @param [in] max_value : max accepted value
         */
        inline static void check_range(T value, T max_value)
        {
            static_assert(value <= max_value, "the value is out of range");
        }

        /**
         * @brief check if an overrun occured and then roll the value to a specified value
         * 
         * @param [in] value     : value
         * @param [in] max_value : max permitted value
         * @param [in] ovr_value : roll value
         */
        inline static void check_overrun(T& value, T max_value, T ovr_value)
        {
            if (value > max_value)
            {
                value = ovr_value;
            }
        }

        /**
         * @brief check if a pointer is not null
         * 
         * @param [in] pointer : pointer of any data type
         */
        
        inline static void check_pointer(const T* pointer)
        {
            static_assert(pointer != nullptr, "the pointer is a null pointer");
        }

        /**
         * @brief check if a value is out of bound
         * 
         * @param [in] value : value
         * @param [in] bound : bound value
         */
        inline static void check_out_of_bound(T value, T bound)
        {
            static_assert(value < bound, "the value is out of bound");
        }
};


#endif      /* _CHECKER_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
