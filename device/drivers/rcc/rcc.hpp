/************************************************************************************* 
 * @file   rcc.hpp
 * @date   Nov, 26 2023
 * @author Awatsa Hermann
 * @brief  clock interface header file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.26.11   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

#ifndef _RCC_H_
#define _RCC_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "operators.hpp"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace drivers
{
    class rcc
    {
        public:

            static void enableClock(const u32 periphID);
            static void disableClock(const u32 periphID);
            static void resetPeriph(const u32 periphID);
            static u32  getClockFrequency();

        private:
        
            static constexpr u32 PERIPH_ID_MAX = 143;
        #if defined (STM32F303)
            static constexpr u32 SYSCLK = 72_mhz;
        #elif defined (STM32G473)
            static constexpr u32 SYSCLK = 170_mhz;
        #endif
    };
}


#endif      /* _RCC_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
