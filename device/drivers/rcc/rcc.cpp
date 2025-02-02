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
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
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
#include "common.hpp"
#include "rcc.hpp"



/**
 * \defgroup drivers Drivers 
 * \brief Set of low-level drivers designed to interface with hardware peripherals
 * on the microcontroller. They provide an abstraction layer for interacting with
 * various hardware peripherals such as UART, I2C, DMA, GPIO, Timers, RCC and so on.
 * 
 * @{
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

using namespace drivers;

/**
 * \brief enable the clock for the specified peripheral
 * 
 * \param [in] periphID : peripheral identifier
 */
void rcc::enableClock(const u32 periphID)
{
    _vol u32 temp;  /* temporary variable */

    #if defined (STM32G474)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
    #elif defined (STM32F303)
    constexpr u32 AHB1_MAX = periphID::ADC34_ID;
    #endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX) {
        if (!common::read_reg_bits(RCC->AHB1ENR, SET, periphID))
        {
            common::set_reg_bit(RCC->AHB1ENR, SET, periphID);
            /* dummy read in order to set a delay after enabling the clock */
            temp = common::read_reg_bits(RCC->AHB1ENR, SET, periphID);
        }
    }

    #if defined (STM32G474)
    else if (periphID <= common::periphID::DAC4_ID) {
        if (!common::read_reg_bits(RCC->AHB2ENR, common::SET,(periphID-32U))
        {
            common::set_reg_bits(RCC->AHB2ENR, common::SET, periphID-32U);
            /* dummy read in order to set a delay after enabling the clock */
            temp = common::read_reg_bits(RCC->AHB2ENR, common::SET, periphID-32U);
        }
    }
    #endif  /* STM32G474 */

    else if (periphID <= periphID::PWR_ID) {
        if (!common::read_reg_bits(RCC->APB1ENR, SET, periphID-64U))
        {
            common::set_reg_bit(RCC->APB1ENR, SET, periphID-64U);
            /* dummy read in order to set a delay after enabling the clock */
            temp = common::read_reg_bits(RCC->APB1ENR, SET, periphID-64U);
        }
    }
    else if (periphID <= periphID::TIM8_ID) {
        if (!common::read_reg_bits(RCC->APB2ENR, SET, periphID-96U))
        {
            common::set_reg_bit(RCC->APB2ENR, SET, periphID-96U);
            /* dummy read in order to set a delay after enabling the clock */
            temp = common::read_reg_bits(RCC->APB2ENR, SET, periphID-96U);
        }
    }
    else {
        RCC->BDCR_b.RTCEN = SET;
    }
    (void)temp;
}


/**
 * \brief disable the clock for the specified peripheral
 * 
 * \param [in] periphID : peripheral identifier
 */
void rcc::disableClock(const u32 periphID)
{
    #if defined (STM32G474)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
    #elif defined (STM32F303)
    constexpr u32 AHB1_MAX = periphID::ADC34_ID;
    #endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX) {
        if (common::read_reg_bits(RCC->AHB1ENR, SET, periphID)) {
            common::reset_reg_bits(RCC->AHB1ENR, SET, periphID);
        }
    }

    #if defined (STM32G474)
    else if (periphID <= common::periphID::DAC4_ID) {
        if (common::read_reg_bits(RCC->AHB2ENR, common::SET, periphID-32U)) {
            common::reset_reg_bits(RCC->AHB2ENR, common::SET, periphID-32U);
        }
    }
    #endif  /* STM32G474 */

    else if (periphID <= periphID::PWR_ID) {
        if (common::read_reg_bits(RCC->APB1ENR, SET, periphID-64U)) {
            common::reset_reg_bits(RCC->APB1ENR, SET, periphID-64U);
        }
    }
    else if (periphID <= periphID::TIM8_ID) {
        if (common::read_reg_bits(RCC->APB2ENR, SET, periphID-96U)) {
            common::reset_reg_bits(RCC->APB2ENR, SET, periphID-96U);
        }
    }
    else {
        RCC->BDCR_b.RTCEN = RESET;
    }
}


/**
 * \brief reset the specified peripheral
 * 
 * \param [in] periphID : peripheral identifier
 */
void rcc::resetPeriph(const u32 periphID)
{
    #if defined (STM32G474)
    constexpr u32 AHB1_MAX = common::periphID::CRC_ID;
    #elif defined (STM32F303)
    constexpr u32 AHB1_MAX = periphID::ADC34_ID;
    #endif

    assert(periphID <= PERIPH_ID_MAX, "the peripheral ID is out of range");

    if (periphID <= AHB1_MAX)
    {
        common::set_reg_bit(RCC->AHB1RSTR, SET, periphID);
        common::reset_reg_bits(RCC->AHB1RSTR, SET, periphID);
    }
    #if defined (STM32G474)
    else if (periphID <= periphID::DAC4_ID)
    {
        common::set_reg_bits(RCC->AHB2RSTR, common::SET, periphID%32);
        common::reset_reg_bits(RCC->AHB2RSTR, common::SET, periphID%32);
    }
    #endif  /* STM32G474 */
    else if (periphID <= periphID::PWR_ID)
    {
        common::set_reg_bit(RCC->APB1RSTR, SET, periphID%64);
        common::reset_reg_bits(RCC->APB1RSTR, SET, periphID%64);
    }
    else if (periphID <= periphID::TIM8_ID)
    {
        common::set_reg_bit(RCC->APB2RSTR, SET, periphID%96);
        common::reset_reg_bits(RCC->APB2RSTR, SET, periphID%96);
    }
}


/**
 * \brief get the clock frequency of the specified peripheral
 * 
 * \return the clock frequency of the peripheral
 */
u32 rcc::getClockFrequency()
{
    return SYSCLK;
}

/**@}*/

/**@}*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
