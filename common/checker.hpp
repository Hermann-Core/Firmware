/************************************************************************************* 
 * @file   checker.hpp
 * @date   Nov, 28 2023
 * @author Awatsa Hermann
 * @brief  various checkers interface
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
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

#ifndef _CHECKER_H_
#define _CHECKER_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/


/**
 * \addtogroup common
 * Various checkers used accross the project
 * 
 * @{
 */

/*==================================================================================
|                                NAMESPACE                                 
===================================================================================*/

namespace common
{
    /**
     * \brief check if an overrun occured and then roll the value to the limit
     * 
     * \tparam T template parameter
     * \param [in] value     : value
     * \param [in] max_value : max permitted value
     * \param [in] ovr_value : roll value
     */
    template <typename T>
    constexpr void handle_overrun(T& value, T max_value, T ovr_value)
    {
        if (value > max_value)
        {
            value = ovr_value;
        }
    }
};

/** @} */

#endif      /* _CHECKER_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
