/************************************************************************************* 
 * @file   common-types.h
 * @date   Nov, 27 2023
 * @author Awatsa Hermann
 * @brief  this file containts the constants used
 *         for writing the peripherals drivers
 * 
 * ***********************************************************************************
 * @attention
 * 
 #   date       |  Version  | revision   |
 -----------------------------------------
 # 2023.27.11   |    1      |  0         |

*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _DRIVERS_CONST_H_
#define _DRIVERS_CONST_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

class periphID
{
    public:
#if defined (STM32G473)
        /*====================== AHB1 ========================*/
        static constexpr u32 DMA1_ID    =  static_cast<u32>(0);
        static constexpr u32 DMA2_ID    =  static_cast<u32>(1);
        static constexpr u32 DMAMUX_ID  =  static_cast<u32>(3);
        static constexpr u32 CORDIC_ID  =  static_cast<u32>(4);
        static constexpr u32 FLASH_ID   =  static_cast<u32>(8);
        static constexpr u32 CRC_ID     =  static_cast<u32>(12);

        /*====================== AHB2 ========================*/
        static constexpr u32 GPIOA_ID   =  static_cast<u32>(32);
        static constexpr u32 GPIOB_ID   =  static_cast<u32>(33);
        static constexpr u32 GPIOC_ID   =  static_cast<u32>(34);
        static constexpr u32 GPIOD_ID   =  static_cast<u32>(35);
        static constexpr u32 GPIOE_ID   =  static_cast<u32>(36);
        static constexpr u32 GPIOF_ID   =  static_cast<u32>(37);
        static constexpr u32 GPIOG_ID   =  static_cast<u32>(38);
        static constexpr u32 ADC12_ID   =  static_cast<u32>(45);
        static constexpr u32 ADC345_ID  =  static_cast<u32>(46);
        static constexpr u32 DAC1_ID    =  static_cast<u32>(48);
        static constexpr u32 DAC2_ID    =  static_cast<u32>(49);
        static constexpr u32 DAC3_ID    =  static_cast<u32>(50);
        static constexpr u32 DAC4_ID    =  static_cast<u32>(51);

        /*====================== BDCR ========================*/
        static constexpr u32 RTC_ID    =  static_cast<u32>(143);

#elif defined (STM32F303)
        /*====================== AHB1 ========================*/
        static constexpr u32 DMA1_ID    =  static_cast<u32>(0);
        static constexpr u32 DMA2_ID    =  static_cast<u32>(1);
        static constexpr u32 FLASH_ID   =  static_cast<u32>(4);
        static constexpr u32 CRC_ID     =  static_cast<u32>(6);
        static constexpr u32 GPIOA_ID   =  static_cast<u32>(17);
        static constexpr u32 GPIOB_ID   =  static_cast<u32>(18);
        static constexpr u32 GPIOC_ID   =  static_cast<u32>(19);
        static constexpr u32 GPIOD_ID   =  static_cast<u32>(20);
        static constexpr u32 GPIOE_ID   =  static_cast<u32>(21);
        static constexpr u32 GPIOF_ID   =  static_cast<u32>(22);
        static constexpr u32 GPIOG_ID   =  static_cast<u32>(23);
        static constexpr u32 ADC12_ID   =  static_cast<u32>(28);
        static constexpr u32 ADC34_ID   =  static_cast<u32>(29);

#endif
        /*====================== APB1 ========================*/
        static constexpr u32 TIM2_ID    =  static_cast<u32>(64);
        static constexpr u32 TIM3_ID    =  static_cast<u32>(65);
        static constexpr u32 TIM4_ID    =  static_cast<u32>(66);
        static constexpr u32 RTCAPB_ID  =  static_cast<u32>(74);
        static constexpr u32 WWDG_ID    =  static_cast<u32>(75);
        static constexpr u32 USART3_ID  =  static_cast<u32>(82);
        static constexpr u32 I2C1_ID    =  static_cast<u32>(85);

        /*====================== APB2 ========================*/
        static constexpr u32 SYSCFG_ID  =  static_cast<u32>(96);
        static constexpr u32 TIM1_ID    =  static_cast<u32>(107);
        static constexpr u32 TIM8_ID    =  static_cast<u32>(109);
};     



#endif      /* _DRIVERS_CONST_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
