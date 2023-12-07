/************************************************************************************* 
 * @file   types.h
 * @date   03, March 2023
 * @author Awatsa Hermann
 * @brief  common C/C++ types redefinition
 * 
 *         Redefinition of the common C/C++ types
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.03   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _COMMON_TYPES_H_
#define _COMMON_TYPES_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include <stdint.h>
#include <stddef.h>



/*==================================================================================
|                             TYPES DEFINITIONS                                
===================================================================================*/
#define _vol        volatile

#ifdef __cplusplus
extern "C" {
#endif
typedef int8_t     i8;
typedef uint8_t    u8;
typedef int16_t    i16;
typedef uint16_t   u16;
typedef int32_t    i32;
typedef uint32_t   u32;
typedef int64_t    i64;
typedef uint64_t   u64;
#ifdef __cplusplus
}
#endif


#endif      /* _COMMON_TYPES_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
