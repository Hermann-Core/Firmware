/************************************************************************************* 
 * @file 	   peripherals_irqs.h
 * @date       05, March 2023
 * @author     AWATSA HERMANN
 * @brief	   Peripherals interrupts header file
 * 
 *             Contains the declarations of the MCU
               peripherals interrupts handlers
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.05   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _PERIPHERALS_IRQS_
#define _PERIPHERALS_IRQS_

#ifdef __cplusplus
extern "C"
{
#endif

/*====================================================================================
|                        Cortex m4 specific exceptions handlers                      |
====================================================================================*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/*====================================================================================
|                    STM32G4 and F303 specific interrupts handlers                   |
====================================================================================*/

#ifdef __cplusplus
}
#endif


#endif      /* _PERIPHERALS_IRQS_ */

/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
