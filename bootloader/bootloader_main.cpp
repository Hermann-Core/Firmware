/************************************************************************************* 
 * @file 	 bootloader_main.cpp
 * @date     April, 02 2023
 * @author   AWATSA HERMANN
 * @brief	 main bootloader source file
 * 
 *           All the main bootloader related calls will be done here
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


/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "features.h"



/************************************************************************************#
|                                       DEFINES                                      |
#************************************************************************************/



/************************************************************************************#
|                              VARIABLES DECLARATIONS                                |
#************************************************************************************/
const u16 a = 3;
extern const u32 APP_ADDRESS[];



/************************************************************************************#
|                              FUNCTIONS DEFINITIONS                                 |
#************************************************************************************/

int main (void)
{
    SysTick_Config(72000);
    
    while (true)
    {
        RTT_WriteString(0, "\n\nFirst test\n");
        RTT_printf(0, "\nLe double de 3 est : %02u", a*2);
    }
}


static void JumpToApp(u32 *appAddress)
{
    
}
