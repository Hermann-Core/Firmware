/************************************************************************************* 
 * @file   flash.hpp
 * @date   Dec, 21 2023
 * @author Awatsa Hermann
 * @brief  flash memory interface header file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.12.21   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

// Include guard
#ifndef _FLASH_H_
#define _FLASH_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "types.h"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace driver
{
    class flash
    {
        public:

            static bool erase(const u32 address);
            static bool erase(const u32 address, const size_t nbPages);
            static bool massErase();
            static bool read(const u32 address, u32* buffer, const size_t size);
            static u32  read(const u32 address);
            template<typename T>
            static bool program(const u32 address, const T* buffer, const size_t size);
            template<typename T>
            static bool program(const u32 address, const T value);
            static bool protect(const u32 startAddress, const size_t size);
        
        private:
            
            static void IRQ_Handler();
            #if defined (STM32G474)
            static constexpr auto PAGE_SIZE = 0x1000;
            #elif defined (STM32F303)
            static constexpr auto PAGE_SIZE = 0x800;
            #endif
    };
}


#endif      /* _FLASH_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
