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
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _FEATURES_H_
#define _FEATURES_H_


/* Enable the Arm standard portability level */
#define _AEABI_PORTABILITY_LEVEL   1

/************************************************************************************/
/*                                     INCLUDES                                     */
/************************************************************************************/
#include "periph_def.h"


/* Iindicate whether we use the Realtime trace feature or not */
#ifdef _USE_RTT
 #include "SEGGER_RTT.h"
#endif

/* Indicate whether we use the arm dsp library or not */
#ifdef _DSP_LIB
 #include "arm_math.h"
#endif


#endif      /* _FEATURES_H_ */

/************************************************************************************/
/*                                   END OF FILE                                    */
/************************************************************************************/
