/************************************************************************************* 
 * @file   print.c
 * @date   Dec, 12 2023
 * @author Awatsa Hermann
 * @brief  formated string print
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
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


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "print.h"


/**
 * \addtogroup common
 * 
 * @{
 * 
 * \defgroup print Print Functions
 * \brief Formated string handler. Provide functions for handling and outputs formated
 * strings in buffer using a lighweight version of the printf function.
 * 
 * @{
 * 
 */

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
 * \brief Store a formated string in the buffer
 * 
 * \param [in] buffer : buffer
 * \param [in] format : formated string
 * \param [in] args   : va_list arguments
 * \return number of printed characters
 */
size_t _sprintf(char *buffer, const char *format, va_list args)
{
    if (buffer == NULL || format == NULL) {
        return 0;
    }

    size_t count = 0;

    for (; *format != '\0'; ++format)
    {
        if (*format != '%'){
            buffer[count++] = *format;
            continue;
        }
        ++format;

        // parse format specifier
        u8 width = 0;
        while (*format >= '0' && *format <= '9')
        {
            width = (u8)(width * 10 + (*format - '0'));
            ++format;
        }

        // handle specifier cases
        switch (*format)
        {
            case 'd':
            {
                int value = va_arg(args, int);
                // print the sign if applicable
                if (value < 0)
                {
                    buffer[count++] = '-';
                    value = -value;
                }
                // pad with leading zeros if applicable
                for (u8 i = 0; i < (width - count_digits(value)); i++){
                    buffer[count++] = '0';
                }
                store_digits(value, buffer, &count);
                break;
            }
            case 'u':
            {
                u32 value = va_arg(args, u32);
                // pad with leading zeros if applicable
                for (u8 i = 0; i < (width - count_digits(value)); i++) {
                    buffer[count++] = '0';
                }
                store_digits(value, buffer, &count);
                break;
            }
            case 's':
            {
                const char* str = va_arg(args, const char*);
                if (str == NULL) {
                    buffer[count++] = '\0';
                    break;
                }
                while (*str != '\0') {
                    buffer[count++] = *str++;
                }
                break;
            }
            default:
                buffer[count++] = *format;
                break;
        }
    }
    buffer[count] = '\0';  // null terminate the buffer
    return count;
}


/**
 * \brief Store a formated string in the buffer
 * 
 * \param [in] buffer : buffer
 * \param [in] format : formated string
 * \return number of printed characters
 */
size_t print(char buffer[], const char* format, ...)
{
    if (format == NULL) {
        return 0;
    }

    va_list args;
    va_start(args, format);

    size_t count = _sprintf(buffer, format, args);

    va_end(args);
    return count;
}


/** @} */

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
