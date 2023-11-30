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
#include "assert.h"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

template <typename T = u32>
class checker
{
    public:

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
};


#endif      /* _CHECKER_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
