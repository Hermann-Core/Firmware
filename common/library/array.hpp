/************************************************************************************* 
 * @file   array.hpp
 * @date   Nov, 29 2023
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
 # 2023.29.11   |    1      |  0         |

*************************************************************************************/

#ifndef _ARRAY_H_
#define _ARRAY_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include <cstddef>
#include "common_types.h"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

/**
 * @class static object container class
 */
template <typename T, std::size_t _SIZE>
class array
{
    public:

    private:
};


#endif      /* _ARRAY_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
