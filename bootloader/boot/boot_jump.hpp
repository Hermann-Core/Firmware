/************************************************************************************* 
 * @file   boot_jump.hpp
 * @date   June, 27 2023
 * @author Awatsa Hermann
 * @brief  bootloader jump header file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.02   |    1      |  1         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _BOOT_JUMP_H_
#define _BOOT_JUMP_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common.hpp"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

class boot_jump
{
    public:
        static void jumpToApp(const u32* appVector);

    private:
        static void jumpASM(u32 SP, u32 PC);
};


#endif  /* _BOOT_JUMP_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
