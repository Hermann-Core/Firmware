/************************************************************************************* 
 * @file   bootloader_main.cpp
 * @date   june, 30 2023
 * @author Awatsa Hermann
 * @brief  Bootloader entry point. Comprises the main function of the bootlader
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.06.30   |    1      |  1         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 *
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "main.hpp"
#include "boot_jump.hpp"
#include "const.hpp"
#include "gpio.hpp"

// Todo! Set bootloader version (major, minor, patch)

/*==================================================================================
|                                   DEFINES                                
===================================================================================*/ 



/*==================================================================================
|                            FUNCTIONS DECLARATIONS                                
===================================================================================*/



/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

int main (void)
{
    extern const u32 APP_ADDRESS;   /* defined in the linker script */

    driver::gpio gpioTest(common::GPIOD_ID);

    gpioTest.init(common::GPIOCfg::PULLDWN, common::GPIO_PIN10);
    gpioTest.enableIrq(common::GPIO_PIN10, common::edge::RISING, nullptr);

    // boot_jump::jumpToApp(&APP_ADDRESS);     /* jump to the application */

    while (true);
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
