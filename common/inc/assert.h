/************************************************************************************* 
 * @file   assert.h
 * @date   Nov, 30 2023
 * @author Awatsa Hermann
 * @brief  assertion handler interface
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.30.11   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _ASSERT_H_
#define _ASSERT_H_


/*====================================================================================
|                     INCLUDES                      
====================================================================================*/
#include <stdbool.h>



/*====================================================================================
|                     FUNCTIONS DECLARATIONS                      
====================================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef NDEBUG

    #define assert(expression, message) ((void)0)

#else
    void assert_handler(bool condition,
                        const char* message,
                        const char* file,
                        int line);

    #define assert(expression, message) (void)(                              \
                                        assert_handler(expression, message,  \
                                        __FILE__, (unsigned)(__LINE__)))

#endif

#ifdef __cplusplus
}
#endif


#endif      /* _ASSERT_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
