/************************************************************************************* 
 * @file   main_app.cpp
 * @date   April, 02 2023
 * @author Awatsa Hermann
 * @brief  Main application entry point. The main function is defined here. 
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.02   |    1      |  0         |
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



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/



/*==================================================================================
|                            VARIABLES DECLARATIONS                                
===================================================================================*/



/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

int main (void)
{
    SysTick_Config(72000);

    while (true)
    {
        
    }
}


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
