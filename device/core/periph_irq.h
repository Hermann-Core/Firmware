/************************************************************************************* 
 * @file   periph_irq.h
 * @date   05, March 2023
 * @author Awatsa Hermann
 * @brief  Peripherals interrupts header file
 * 
 *         Contains the declarations of the MCU
 *         peripherals interrupts handlers
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
#ifndef _PERIPHERALS_IRQS_H_
#define _PERIPHERALS_IRQS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*====================================================================================
|                     Cortex m4 specific exceptions handlers                      
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
|                         STM32 specific interrupts handlers                   
====================================================================================*/

#ifdef __cplusplus
}
#endif


#endif      /* _PERIPHERALS_IRQS_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
