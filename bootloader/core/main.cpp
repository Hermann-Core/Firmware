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
#include "common.hpp"
#include "boot_jump.hpp"
#include "rcc.hpp"

// Todo! Set bootloader version (major, minor, patch)

/*==================================================================================
|                                   DEFINES                                
===================================================================================*/ 



/*==================================================================================
|                            VARIABLES DECLARATIONS                                
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

    driver::rcc::enableClock(common::periphID::I2C1_ID);

    I2C1->CR1 = 10512154;
    I2C1->OAR1 = 14124655;
    I2C1->OAR2 = 52614;

    driver::rcc::resetPeriph(common::periphID::I2C1_ID);

    boot_jump::jumpToApp(&APP_ADDRESS);     /* jump to the application */

    while (true);
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
