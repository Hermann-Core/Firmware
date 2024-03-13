/************************************************************************************* 
 * @file   gpio.hpp
 * @date   Dec, 20 2023
 * @author Awatsa Hermann
 * @brief  gpio driver interface
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.12.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "array.hpp"


/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace drivers
{
    class gpio
    {
        public:
            using cbk_f = void (*)(void);
            
            explicit gpio(u32 gpioID);
            void init(GPIOCfg config, u32 pinMask);
            void init(GPIOCfg config, u16 pin);
            void set(u16 pin);
            void reset(u16 pin);
            void toggle(u16 pin);
            bool read(u16 pin);
            bool lock(u16 pin);
            void enableIrq(u16 pin, edge edge, cbk_f callback);
            void disableIrq(u16 pin) const;
            void enableEvent(u16 pin, edge edge);
            void disableEvent(u16 pin) const;

        private:
            void clearCfg(u16 pin);
            void cfgOutput(u16 config, u16 pin);
            void cfgInput(u16 config, u16 pin);
            void cfgAlt(u16 config, u16 pin);
            GPIO_HW* getPort(u32 gpioID);

            static constexpr auto MAX_PINS = static_cast<u16>(16);
            static constexpr auto HIGH_SPEED = static_cast<u32>(3);
            u32  portNum;                    // GPIO port number
            GPIO_HW* const locGPIO;          // Local GPIO peripheral pointer
            array<bool,  MAX_PINS> inFlag;   // Input config flags
            array<bool,  MAX_PINS> outFlag;  // Output config flags

        public:
            static array<cbk_f, MAX_PINS> cbkArray; // Array of callback functions
    };
}


#endif      /* _GPIO_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
