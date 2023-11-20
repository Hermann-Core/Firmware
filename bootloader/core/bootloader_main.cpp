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

// Todo! Set bootloader version (major, minor, patch)

/*==================================================================================
|                                   DEFINES                                
===================================================================================*/ 



/*==================================================================================
|                            VARIABLES DECLARATIONS                                
===================================================================================*/
extern const u32 APP_ADDRESS;   /* Symbol defines in the linker script */



/*==================================================================================
|                            FUNCTIONS DECLARATIONS                                
===================================================================================*/



/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

int main (void)
{
    RTT_WriteString(0, "\n\nIci c'est le bootloader\n");

    boot_jump::jumpToApp(&APP_ADDRESS);

    while (true)
    {
        RTT_WriteString(0, "\n\nOn est entré dans la boucle\n");
    }
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/