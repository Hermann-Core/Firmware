/************************************************************************************* 
 * @file 	   features.h
 * @date       05, March 2023
 * @author     AWATSA HERMANN
 * @brief	   This file contains some feature to
 *             extend the functionalities of the MCU
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.05   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _FEATURES_
#define _FEATURES_


/* Enable the Arm standard portability level */
#define _AEABI_PORTABILITY_LEVEL   1

/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "peripherals_defs.h"


/* Iindicate whether we use the Realtime trace feature or not */
#ifdef _USE_RTTI
 #include "SEGGER_RTT.h"
#endif

/* Indicate whether we use the arm dsp library or not */
#ifdef _DSP_LIB
 #include "arm_math.h"
#endif

/* Defined the use of interrupts or not */
#ifdef _ENABLE_IRQ
 #include "hardware_core.h"
#endif


#endif      /* _FEATURES_ */

/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
