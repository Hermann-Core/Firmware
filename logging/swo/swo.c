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
#include "common_types.h"
#include "swo.h"



/**
 * @defgroup logging Logging
 * \brief Data output functionalities. This provide ways for printing data outside
 * the MCU for various purposes including debugging, logging and so on
 * 
 * \defgroup swo Serial Wire Output
 * \ingroup logging
 * This file contains an implementation for serial wire output interface functions
 * that allow printing data to the ITM (Instrumentation Trace Macrocell) of STM32F303
 * and STM32G473 MCUs using a lighweight version of the printf function.
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

/**
 * \brief count the number of digits in an integer
 * 
 * \param [in] value : integer value
 * \return number of digits 
 */
static u8 count_digits(int value)
{
    u8 count = 0;
    do
    {
        value /= 10;
        count++;
    } while (value != 0);

    return count;
}

/**
 * \brief store the digits in the buffer
 * 
 * \param [in] digit  : the decimal digit
 * \param [in] buffer : the buffer
 * \param [in] pos    : the buffer position
 */
static void store_digits(int digit, char buffer[], u32 *pos)
{
    if (digit >= 10)
    {
        store_digits(digit/10, buffer, pos);
    }
    buffer[*pos] = (char)((digit % 10) + '0');
    (*pos)++;
}



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
 * \param [in] s : string to print
 * \return number of printed characters 
 */
u32 swo_printf(const char* format, ...)
{
    if (format == NULL) {
        return 0;
    }

    char buffer[PRINTF_BUFFER_SIZE] = {0};
    u32 count = 0;
    va_list args;
    va_start(args, format);

    do
    {
        if (*format == '%')
        {
            format++;
            u8 width = 0;

            while (*format >= '0' && *format <= '9')
            {
                width = (u8)(width * 10 + (*format - '0'));
                format++;
            }

            switch (*format)
            {
                case 'd':
                {
                    int value = va_arg(args, int);
                    /* print the sign if necessary */
                    if (value < 0)
                    {
                        buffer[count] = '-';
                        count++;
                        value = -value;
                    }
                    /* pad with leading zeros if applicable */
                    for (u8 i = 0; i < (width - count_digits(value)); i++)
                    {
                        buffer[count] = '0';
                        count++;
                    }
                    /* store the actual value */
                    store_digits(value, buffer, &count);
                }    
                    break;

                case 'u':
                {
                    uint32_t value = va_arg(args, uint32_t);
                    /* pad with leading zeros if applicable */
                    for (u8 i = 0; i < (width - count_digits(value)); i++)
                    {
                        buffer[count] = '0';
                        count++;
                    }
                    /* store the actual value */
                    store_digits(value, buffer, &count);
                }
                    break;

                case 's':
                {
                    const char* str = va_arg(args, char*);

                    while (*str != '\0')
                    {
                        buffer[count] = *str++;
                        count++;
                    }
                }
                    break;
                
                default:
                    buffer[count] = *format;
                    count++;
            }
        }
        else
        {
            /* the character is not a specifier */
            buffer[count] = *format;
            count++;
        }

        format++;
        
    } while (*format != '\0');

    count = swo_puts(buffer);   /* print the formated string */

    va_end(args);
    return count;
}

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
