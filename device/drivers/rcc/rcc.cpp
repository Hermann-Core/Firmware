/************************************************************************************* 
 * \file   rcc.cpp
 * \date   Nov, 27 2023
 * \author Awatsa Hermann
 * \brief  Interface designed for managing the rcc
 * 
 * ***********************************************************************************
 * \attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.26.11   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "rcc.hpp"
#include "common.hpp"



/**
 * \defgroup drivers Drivers 
 * \brief Set of low-level drivers designed to interface with hardware peripherals
 * on the microcontroller. They provide an abstraction layer for interacting with
 * various hardware peripherals such as UART, I2C, DMA, GPIO, Timers, RCC and so on.
 * 
 * \defgroup rcc Reset & Clock Control
 * \ingroup drivers
 * Handle interactions with the Reset and Clock Control (RCC) module. Provides an
 * interface for enabling/disabling peripheral clocks, managing reset states as well
 * as providing peripheral clocks frequencies.
 * 
 * @{
 */

/*==================================================================================
|                         PRIVATE FUNCTIONS DEFINITIONS                                
===================================================================================*/



/*==================================================================================
|                         PUBLIC FUNCTIONS DEFINITIONS                                
===================================================================================*/

using namespace driver;

/**
 * \brief enable the clock for the specified peripheral
 * 
 * \param [in] periphID : peripheral ID 
 */
void rcc::enableClock(const u32& periphID)
{
#if defined (STM32G473)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
#elif defined (STM32F303)
    constexpr u32 AHB1_MAX = common::periphID::ADC34_ID;
#endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX)
    {
        common::set_reg_bits(RCC->AHB1ENR, (0x1UL << periphID));
    }
#if defined (STM32G473)
    else if (periphID <= common::periphID::DAC4_ID)
    {
        common::set_reg_bits(RCC->AHB2ENR, (0x1UL << (periphID%32)));
    }
#endif  /* STM32G473 */
    else if (periphID <= common::periphID::I2C1_ID)
    {
        common::set_reg_bits(RCC->APB1ENR, (0x1UL << (periphID%64)));
    }
    else if (periphID <= common::periphID::TIM8_ID)
    {
        common::set_reg_bits(RCC->APB2ENR, (0x1UL << (periphID%96)));
    }
    else
    {
        RCC->BDCR_b.RTCEN = 1;
    }
}


/**
 * \brief disable the clock for the specified peripheral
 * 
 * \param [in] periphID : peripheral ID 
 */
void rcc::disableClock(const u32& periphID)
{
    #if defined (STM32G473)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
#elif defined (STM32F303)
    constexpr u32 AHB1_MAX = common::periphID::ADC34_ID;
#endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX)
    {
        common::reset_reg_bits(RCC->AHB1ENR, (0x1UL << periphID));
    }
#if defined (STM32G473)
    else if (periphID <= common::periphID::DAC4_ID)
    {
        common::reset_reg_bits(RCC->AHB2ENR, (0x1UL << (periphID%32)));
    }
#endif  /* STM32G473 */
    else if (periphID <= common::periphID::I2C1_ID)
    {
        common::reset_reg_bits(RCC->APB1ENR, (0x1UL << (periphID%64)));
    }
    else if (periphID <= common::periphID::TIM8_ID)
    {
        common::reset_reg_bits(RCC->APB2ENR, (0x1UL << (periphID%96)));
    }
    else
    {
        RCC->BDCR_b.RTCEN = 0;
    }
}


/**
 * \brief reset the specified peripheral
 * 
 * \param [in] periphID : peripheral ID 
 */
void rcc::resetPeriph(const u32& periphID)
{
    #if defined (STM32G473)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
#elif defined (STM32F303)
    constexpr u32 AHB1_MAX = common::periphID::ADC34_ID;
#endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX)
    {
        common::set_reg_bits(RCC->AHB1RSTR, (0x1UL << periphID));
        common::reset_reg_bits(RCC->AHB1RSTR, (0x1UL << periphID));
    }
#if defined (STM32G473)
    else if (periphID <= periphID::DAC4_ID)
    {
        common::set_reg_bits(RCC->AHB2RSTR, (0x1UL << (periphID%32)));
        common::reset_reg_bits(RCC->AHB2RSTR, (0x1UL << (periphID%32)));
    }
#endif  /* STM32G473 */
    else if (periphID <= common::periphID::I2C1_ID)
    {
        common::set_reg_bits(RCC->APB1RSTR, (0x1UL << (periphID%64)));
        common::reset_reg_bits(RCC->APB1RSTR, (0x1UL << (periphID%64)));
    }
    else if (periphID <= common::periphID::TIM8_ID)
    {
        common::set_reg_bits(RCC->APB2RSTR, (0x1UL << (periphID%96)));
        common::reset_reg_bits(RCC->APB2RSTR, (0x1UL << (periphID%96)));
    }
}


/**
 * \brief get the clock frequency of the specified peripheral
 * 
 * \param [in] periphID : peripheral ID 
 * \return the clock frequency of the peripheral
 */
u32 rcc::getClockFrequency(const u32& periphID)
{
    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    return SYSCLK;
}

/**@}*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
