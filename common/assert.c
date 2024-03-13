/************************************************************************************* 
 * @file   hardware_init.c
 * @date   Nov, 29 2023
 * @author Awatsa Hermann
 * @brief  assertion handler file
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.30.11   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "assert.h"
#include "swo.h"


/**
 * \defgroup common Common
 * Collection of common functionalities and utilities that are used across the project.
 * 
 * @{
 * 
 * \defgroup assert Assertions Handler
 * Assertion handler for error checking
 * 
 * @{
 */

/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

/**
 * \brief handle the assertion
 * 
 * \param [in] condition : condition to check
 * \param [in] message : custom message to print at assertion failed
 * \param [in] file : file name
 * \param [in] line : line number
 */
void assert_handler(bool condition, const char* message,
                    const char* file, int line)
{
    if (!condition)
    {
        while (1)
        {
            swo_printf("ASSERTION FAILED in \"%s\" at line %d : ", file, line);
            swo_printf("%s\n\n", message);

            asm("bkpt");    /* we halted the cpu */
        }
    }
}

/** @} */

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
