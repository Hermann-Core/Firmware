/************************************************************************************* 
 * @file   bootloader_main.cpp
 * @date   june, 30 2023
 * @author Awatsa Hermann
 * @brief  main bootloader source file
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

*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "bootloader_main.hpp"
#include "features.h"
#include "boot_jump.hpp"
#include "drivers_const.hpp"
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
    extern const u32 APP_ADDRESS;   /* Symbol defined in the linker script */

    // boot_jump::jumpToApp(&APP_ADDRESS);

    u32 temp = driver::rcc::getClockFrequency(200);

    while (true)
    {
        
    }
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
