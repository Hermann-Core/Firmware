/************************************************************************************* 
 * @file   const.hpp
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
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _CONST_H_
#define _CONST_H_


/**
 * \defgroup driversConst Drivers Constants
 * \ingroup common
 * \brief Defined constants used for the peripherals drivers
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "types.h"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

namespace common
{
    /**
     * \brief Common constants definitions
     */
    inline namespace common_const
    {
        constexpr auto SET         = 0x1;
        constexpr auto RESET       = 0x0;
        constexpr auto CLEAR_FLAGS = 0xFFFFFFFFU;
        constexpr auto CLEAR_REG   = static_cast<u32>(0x00);
    }

    /**
     * \brief peripherals ID constants definitions
     */
    inline namespace periphID
    {
        #if defined (STM32G473)
        /*====================== AHB1 ========================*/
        static constexpr auto DMA1_ID    =  static_cast<u32>(0);
        static constexpr auto DMA2_ID    =  static_cast<u32>(1);
        static constexpr auto DMAMUX_ID  =  static_cast<u32>(3);
        static constexpr auto CORDIC_ID  =  static_cast<u32>(4);
        static constexpr auto FLASH_ID   =  static_cast<u32>(8);
        static constexpr auto CRC_ID     =  static_cast<u32>(12);

        /*====================== AHB2 ========================*/
        static constexpr auto GPIOA_ID   =  static_cast<u32>(32);
        static constexpr auto GPIOB_ID   =  static_cast<u32>(33);
        static constexpr auto GPIOC_ID   =  static_cast<u32>(34);
        static constexpr auto GPIOD_ID   =  static_cast<u32>(35);
        static constexpr auto GPIOE_ID   =  static_cast<u32>(36);
        static constexpr auto GPIOF_ID   =  static_cast<u32>(37);
        static constexpr auto GPIOG_ID   =  static_cast<u32>(38);
        static constexpr auto ADC12_ID   =  static_cast<u32>(45);
        static constexpr auto ADC345_ID  =  static_cast<u32>(46);
        static constexpr auto DAC1_ID    =  static_cast<u32>(48);
        static constexpr auto DAC2_ID    =  static_cast<u32>(49);
        static constexpr auto DAC3_ID    =  static_cast<u32>(50);
        static constexpr auto DAC4_ID    =  static_cast<u32>(51);

        /*====================== BDCR ========================*/
        static constexpr u32 RTC_ID    =  static_cast<u32>(143);

        #elif defined (STM32F303)
        /*====================== AHB1 ========================*/
        static constexpr auto DMA1_ID    =  static_cast<u32>(0);
        static constexpr auto DMA2_ID    =  static_cast<u32>(1);
        static constexpr auto FLASH_ID   =  static_cast<u32>(4);
        static constexpr auto CRC_ID     =  static_cast<u32>(6);
        static constexpr auto GPIOA_ID   =  static_cast<u32>(17);
        static constexpr auto GPIOB_ID   =  static_cast<u32>(18);
        static constexpr auto GPIOC_ID   =  static_cast<u32>(19);
        static constexpr auto GPIOD_ID   =  static_cast<u32>(20);
        static constexpr auto GPIOE_ID   =  static_cast<u32>(21);
        static constexpr auto GPIOF_ID   =  static_cast<u32>(22);
        static constexpr auto GPIOG_ID   =  static_cast<u32>(23);
        static constexpr auto ADC12_ID   =  static_cast<u32>(28);
        static constexpr auto ADC34_ID   =  static_cast<u32>(29);
        #endif
        /*====================== APB1 ========================*/
        static constexpr auto TIM2_ID    =  static_cast<u32>(64);
        static constexpr auto TIM3_ID    =  static_cast<u32>(65);
        static constexpr auto TIM4_ID    =  static_cast<u32>(66);
        static constexpr auto RTCAPB_ID  =  static_cast<u32>(74);
        static constexpr auto WWDG_ID    =  static_cast<u32>(75);
        static constexpr auto USART3_ID  =  static_cast<u32>(82);
        static constexpr auto I2C1_ID    =  static_cast<u32>(85);
        static constexpr auto PWR_ID     =  static_cast<u32>(92);

        /*====================== APB2 ========================*/
        static constexpr auto SYSCFG_ID  =  static_cast<u32>(96);
        static constexpr auto TIM1_ID    =  static_cast<u32>(107);
        static constexpr auto TIM8_ID    =  static_cast<u32>(109);
    };

    /**
     * \brief GPIO specific constants definitions
     */
    inline namespace const_gpio
    {
        /**
         * \brief GPIO pin configuration type
         */
        enum class GPIOCfg
        {
            FLOAT   ,   PULLUP  ,   PULLDWN ,
            ANALOG  ,   PUSHPULL,   OP_DRAIN,
            ALTER_0 ,   ALTER_1 ,   ALTER_2 ,
            ALTER_3 ,   ALTER_4 ,   ALTER_5 ,
            ALTER_6 ,   ALTER_7 ,   ALTER_8 ,
            ALTER_9 ,   ALTER_10,   ALTER_11,
            ALTER_12,   ALTER_13,   ALTER_14
        };

        /**
         * \brief Tigger type of the GPIO pin
         */
        enum class edge{ RISING, FALLING, BOTH };

        /*================= GPIO pin numbers =================*/
        constexpr auto GPIO_PIN0  = static_cast<u16>(0x0);
        constexpr auto GPIO_PIN1  = static_cast<u16>(0x1);
        constexpr auto GPIO_PIN2  = static_cast<u16>(0x2);
        constexpr auto GPIO_PIN3  = static_cast<u16>(0x3);
        constexpr auto GPIO_PIN4  = static_cast<u16>(0x4);
        constexpr auto GPIO_PIN5  = static_cast<u16>(0x5);
        constexpr auto GPIO_PIN6  = static_cast<u16>(0x6);
        constexpr auto GPIO_PIN7  = static_cast<u16>(0x7);
        constexpr auto GPIO_PIN8  = static_cast<u16>(0x8);
        constexpr auto GPIO_PIN9  = static_cast<u16>(0x9);
        constexpr auto GPIO_PIN10 = static_cast<u16>(0xA);
        constexpr auto GPIO_PIN11 = static_cast<u16>(0xB);
        constexpr auto GPIO_PIN12 = static_cast<u16>(0xC);
        constexpr auto GPIO_PIN13 = static_cast<u16>(0xD);
        constexpr auto GPIO_PIN14 = static_cast<u16>(0xE);
        constexpr auto GPIO_PIN15 = static_cast<u16>(0xF);

        /*=============== Registers bits mask =================*/
        constexpr auto LOCK_MASK  = static_cast<u32>(0x1UL << 16U);
    }

    /**
     * \brief Flash specific constants definitions
     */
    inline namespace flash_const
    {
        enum class pgmSize { HALF, DOUBLE };
    }
};   

/** @} */


#endif      /* _CONST_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
