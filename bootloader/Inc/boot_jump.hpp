/************************************************************************************* 
 * @file 	 boot_jump.hpp
 * @date     April, 02 2023
 * @author   AWATSA HERMANN
 * @brief	 bootloader jump header file
 * 
 *           Contains the functions used to jump to the main application
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.02   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _BOOT_JUMP_H_
#define _BOOT_JUMP_H_


#define UNUSED             __attribute__((unused))


/************************************************************************************/
/*                                      INCLUDES                                    */
/************************************************************************************/
#include "peripherals_defs.h"



/************************************************************************************/
/*                               CLASSES DECLARATIONS                               */
/************************************************************************************/

class jump
{
    public:
        static void JumpToApp(const u32 *appAddress);

    private:
        static void JumpASM(UNUSED u32 SP, UNUSED u32 PC);
};


#endif  /* _BOOT_JUMP_H_ */

/************************************************************************************/
/*                                   END OF FILE                                    */
/************************************************************************************/
