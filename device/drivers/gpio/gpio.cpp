/************************************************************************************* 
 * \file   gpio.cpp
 * \date   Dec, 12 2023
 * \author Awatsa Hermann
 * \brief  gpio driver interface
 * 
 * ***********************************************************************************
 * \attention
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


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "gpio.hpp"



/**
 * \defgroup gpio GPIO
 * \ingroup drivers
 * Provides functions to initialize and control GPIO pins. This driver supports basic
 * GPIO operations such as pin initialization, setting/clearing pin values, reading
 * pin states and attach an interrupt to a pin state change.
 * 
 * @{
 */

/*==================================================================================
|                                PRIVATE FUNCTIONS                                
===================================================================================*/



/*==================================================================================
|                                PUBLIC FUNCTIONS                                
===================================================================================*/

using namespace driver;



/**@}*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
