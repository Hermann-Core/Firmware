/************************************************************************************* 
 * @file   print.h
 * @date   Dec, 11 2023
 * @author Awatsa Hermann
 * @brief  formated string print
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.11.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _PRINT_H_
#define _PRINT_H_


/*====================================================================================
|                     INCLUDES                      
====================================================================================*/
#include <stdarg.h>

#include "types.h"



/*====================================================================================
|                     FUNCTIONS DECLARATIONS                      
====================================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

size_t _sprintf(char buffer[], const char* format, va_list args);
size_t print(char buffer[], const char* format, ...);

#ifdef __cplusplus
}
#endif


#endif      /* _PRINT_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
