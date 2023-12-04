/************************************************************************************* 
 * @file   array.hpp
 * @date   Dec, 01 2023
 * @author Awatsa Hermann
 * @brief  interface used to handle the static objects container.
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.01.12   |    1      |  0         |

 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
*************************************************************************************/

#ifndef _ARRAY_H_
#define _ARRAY_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

/**
 * @defgroup library library
 * @details This group includes several modules and classes providing shared functionality 
 * 
 */

/**
 * @class static object container class
 */
template <typename T, size_t _size>
class array
{
    public:

        using value_type        = T;
        using size_type         = size_t;
        using reference         = T&;
        using const_reference   = const T&;
        using pointer           = T*;
        using const_pointer     = const T*;

    private:

        value_type buffer_[_size];
};



#endif      /* _ARRAY_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
