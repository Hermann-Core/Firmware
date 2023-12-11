/************************************************************************************* 
 * \file   swo.c
 * \date   Dec, 01 2023
 * \author Awatsa Hermann
 * \brief  serial wire output interface
 * ***********************************************************************************
 * \attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.01.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/



/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include <stdarg.h>

#include "print.h"
#include "types.h"
#include "swo.h"



/**
 * @defgroup logging Logging
 * \brief Data output functionalities. This provide ways for printing data outside
 * the MCU for various purposes including debugging, logging and so on
 * 
 * @{
 * 
 * \defgroup swo Serial Wire Output
 * \ingroup logging
 * This file contains an implementation for serial wire output interface functions
 * that allow printing data to the ITM (Instrumentation Trace Macrocell) of STM32F303
 * and STM32G473 MCUs.
 * 
 * @{
 */

/*==================================================================================
|                                 DEFINES                                
===================================================================================*/
#define ITM_STIM_U8            (*(volatile     char*)0xE0000000) // STIM Byte access
#define ITM_ENA                (*(volatile uint32_t*)0xE0000E00) // ITM Enable Register
#define ITM_TCR                (*(volatile uint32_t*)0xE0000E80) // ITM Trace Control Register

#define PRINTF_BUFFER_SIZE     (u32)256


/*==================================================================================
|                               PRIVATE DATA                                
===================================================================================*/



/*==================================================================================
|                             PRIVATE FUNCTIONS                                
===================================================================================*/



/*==================================================================================
|                              PUBLIC FUNCTIONS                                
===================================================================================*/

/**
 * \brief send character through the ITM
 * 
 * \param [in] c : character to send
 * \return print caracter 
 */
char swo_putchar(char c)
{
    /* Check if the ITM_TCR.ITMENA bit is set */
    if ((ITM_TCR & 1) == 0) {
        return 0;
    }

    /* Check if stimulus port is enabled */
    if ((ITM_ENA & 1) == 0) {
        return 0;
    }
    
    /* Wait until STIMx is ready */
    while ((ITM_STIM_U8 & 1) == 0){/**/}
    ITM_STIM_U8 = c;    /* send data */

    return c;
}


/**
 * \brief print a string to the ITM
 * 
 * \param [in] s : string to print
 * \return number of printed characters 
 */
u32 swo_puts(const char* s)
{
    if (s == NULL) {
        return 0;
    }
    
    u32 count = 0;
    do
    {
        swo_putchar(*s++);
        count++;
        
    } while (*s != '\0');

    return count;
}


/**
 * \brief print a format string to the ITM
 * 
 * \param [in] format : string to print
 * \return number of printed characters 
 */
size_t swo_printf(const char* format, ...)
{
    if (format == NULL) {
        return 0;
    }

    char buffer[PRINTF_BUFFER_SIZE] = {0};
    va_list args;
    va_start(args, format);

    u32 count = _sprintf(buffer, format, args);
    swo_puts(buffer);

    va_end(args);
    return count;
}


/** @} */

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
