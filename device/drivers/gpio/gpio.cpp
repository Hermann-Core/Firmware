/************************************************************************************* 
 * \file   gpio.cpp
 * \date   Dec, 20 2023
 * \author Awatsa Hermann
 * \brief  gpio driver interface
 * 
 * ***********************************************************************************
 * \attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.12.12   |    1      |  0         |
 * 
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "gpio.hpp"
#include "common.hpp"
#include "const.hpp"
#include "periph_def.h"
#include "rcc.hpp"
#include "hw_core.hpp"


#define section_ccmram       __attribute__((section(".ccmram")))

/**
 * \defgroup gpio GPIO
 * \ingroup drivers
 * Provides functions to initialize and control GPIO pins. This driver supports basic
 * GPIO operations such as pin initialization, setting/clearing pin values, reading
 * pin states and attach an interrupt to a pin state change.
 * 
 * @{
 */

/*==================================================================================
|                                PRIVATE FUNCTIONS                                
===================================================================================*/
using namespace drivers;

array<gpio::cbk_f, gpio::MAX_PINS> gpio::cbkArray;

/**
 * \brief Clear the configuration of the GPIO pin
 * 
 * \param [in] pin : pin number
 */
void gpio::clearCfg(u16 pin)
{
    assert((pin < MAX_PINS), "Invalid pin number.");

    // Reset the pin configuration
    common::reset_reg_bits(locGPIO->MODER, 0b11, pin*2);
    common::reset_reg_bits(locGPIO->OTYPER, SET, pin);
    common::reset_reg_bits(locGPIO->OSPEEDR, 0b11, pin*2);
    common::reset_reg_bits(locGPIO->PUPDR, 0b11, pin*2);
    common::reset_reg_bits((locGPIO->AFR[pin>>3]), 0xF, (pin%8)*4);
}

/**
 * \brief Configure the pin as input pin with
 *        the given configuratin
 * 
 * \param [in] config : pin configuration setting
 * \param [in] pin    : pin number
 */
void gpio::cfgInput(u16 config, u16 pin)
{
    this->clearCfg(pin);    // Clear the pin configuration
    switch (config)
    {
        case 1: case 2:
            // Set the pin as input pull-up or pull-down pin
            common::modify_reg(locGPIO->PUPDR, 2, config, pin*2);
            break;
        default:
            // Set the pin as analog or input floating pin
            common::modify_reg(locGPIO->MODER, 2, config, pin*2);
            break;
    }
    // Set the GPIO speed to the highest speed
    common::modify_reg(locGPIO->OSPEEDR, 2, HIGH_SPEED, pin*2);
    inFlag.at(pin) = true;  // Set the input flag configuation
}

/**
 * \brief Configure the pin as output pin with
 *        the given configuratin
 * 
 * \param [in] config : pin configuration setting
 * \param [in] pin    : pin number
 */
void gpio::cfgOutput(u16 config, u16 pin)
{
    this->clearCfg(pin);    // Clear the pin configuration
    // Set the pin as general purpose output pin
    common::set_reg_bit(locGPIO->MODER, (0x1UL << (pin*2)));
    // Set the output mode (push-pull or opain drain)
    common::modify_reg(locGPIO->OTYPER, 2, config, pin*2);
    // Set the GPIO speed to the highest speed
    common::modify_reg(locGPIO->OSPEEDR, 2, HIGH_SPEED, pin*2);
    outFlag.at(pin) = true; // Set the output flag configuration
}

/**
 * \brief Configure the pin as alternate function pin
 *        with the given configuratin
 * 
 * \param [in] config : pin configuration setting
 * \param [in] pin    : pin number
 */
void gpio::cfgAlt(u16 config, u16 pin)
{
    this->clearCfg(pin);    // Clear the pin configuration
    // Set the pin as alternate function pin
    common::modify_reg(locGPIO->MODER, 2, 0x2UL, pin*2);
    // Set the GPIO speed to the highest speed
    common::modify_reg(locGPIO->OSPEEDR, 2, HIGH_SPEED, pin*2);
    // Set the alternate function number
    common::modify_reg(locGPIO->AFR[pin>>3], 4, config, (pin%8)*4);
}

/**
 * \brief Get the GPIO port based on the GPIO ID.
 * 
 * \param [in] gpioID : port identifier
 * \return pointer to the port register
 */
GPIO_HW* gpio::getPort(u32 gpioID)
{
    static const array<GPIO_HW*, 6> gpioAddr = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF
    };
    
    return gpioAddr.at(gpioID);
}


/*==================================================================================
|                                PUBLIC FUNCTIONS                                
===================================================================================*/

/**
 * \brief Constructor for the GPIO class
 * 
 * \param [in] gpioID : gpio port identifier
 */
gpio::gpio(u32 gpioID) : portNum((gpioID >= 32U) ? gpioID-32U : gpioID-17U),
                         locGPIO(getPort(portNum))
{
    #if defined (STM32F303)
    assert((gpioID >= 17) && (gpioID <= 22), "Invalid port ID.");
    #elif defined (STM32G474)
    assert((gpioID >= 32) && (gpioID <= 37), "Invalid port ID.");
    #endif

    rcc::enableClock(gpioID);   // Enable the GPIO clock
    // Reset all the content of the arrays
    for (bool& flag : inFlag)  { flag = false; }
    for (bool& flag : outFlag) { flag = false; }
    for (cbk_f& cbk : cbkArray) { cbk = nullptr; }
}


/**
 * \brief Initialize a GPIO pin based on the given configuration
 * 
 * \param [in] config : pin configuration setting
 * \param [in] pin    : pin number to be initialized
 */
void gpio::init(GPIOCfg config, u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    // Configure the pin based on the config type
    switch (auto cfgType = static_cast<u16>(config))
    {
        case 0: case 1:
        case 2: case 3:
            cfgInput(cfgType, pin);
            break;
        case 4: case 5:
            cfgOutput(cfgType - 4, pin);
            break;
        default:
            cfgAlt(cfgType - 6, pin);
            break;
    }
}


/**
 * \brief Initialize GPIO pins based on the provide configuration
 * 
 * \param [in] config  : pin configuration setting used to configure
 *                       the pins as input, output or alternate
 * \param [in] pinMask : pin mask indicating which pins to use
 */
void gpio::init(GPIOCfg config, u32 pinMask)
{
    // Ensure the pin number is within valid range
    assert(pinMask <= 0xFFFF, "Invalid pin mask.");

    // Check if pin is selected in the mask
    for (u16 i = 0; i < MAX_PINS; i++)
    {
        if ((pinMask >> i) & static_cast<u16>(0x1)) {
            // Configure the pin
            this->init(config, i);
        }
    }
}


/**
 * \brief Set the pin to the logic high
 * 
 * \param [in] pin : pin number
 */
void gpio::set(u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (outFlag.at(pin) == true) {
        common::write_reg(locGPIO->BSRR, SET << pin);
    }
}


/**
 * \brief Set the pin to the logic low
 * 
 * \param [in] pin : pin number
 */
void gpio::reset(u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (outFlag.at(pin) == true) {
        common::write_reg(locGPIO->BRR, SET << pin);
    }
}


/**
 * \brief Toggle the corresponding pin
 * 
 * \param [in] pin : pin number
 */
void gpio::toggle(u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (outFlag.at(pin) == true)
    {
        if (common::read_reg_bits(locGPIO->ODR, SET << pin))
        {
            common::write_reg(locGPIO->BRR, SET << pin);
        }
        else {
            common::write_reg(locGPIO->BSRR, SET << pin);
        }
    }
}


/**
 * \brief Read the and return the state of a pin
 * 
 * \param [in] pin : pin number
 * \return true if the pin is set, otherwise false.
 */
bool gpio::read(u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (inFlag.at(pin)  == true || 
        outFlag.at(pin) == true)
    {
        return common::read_reg_bits(locGPIO->IDR, SET, pin);
    }
    return false;
}


/**
 * \brief Lock the configuration of a pin
 * 
 * \param [in] pin : pin number
 * \return true if the pin has correctly been locked,
 *         otherwise false.
 */
bool gpio::lock(u16 pin)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (inFlag.at(pin)  == true || 
        outFlag.at(pin) == true)
    {
        u32 temp = drivers::LOCK_MASK;

        /* Applying the lock sequence */
        common::set_reg_bit(temp, SET, pin);
        common::write_reg(locGPIO->LCKR, temp);
        common::write_reg(locGPIO->LCKR, SET << pin);
        common::write_reg(locGPIO->LCKR, temp);
        temp = locGPIO->LCKR;
        if (temp & LOCK_MASK) {
            return true;
        }
    }
    return false;
}


/**
 * \brief Configure and enable interrupt for a pin
 * 
 * \param [in] pin      : pin number
 * \param [in] edge     : edge polarity
 * \param [in] callback : callback function
 */
void gpio::enableIrq(u16 pin, edge edge, cbk_f callback)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (inFlag.at(pin) == true) {
        // Enable the system configuration clock
        rcc::enableClock(drivers::SYSCFG_ID);
        // Configure the interrupt source
        common::modify_reg(SYSCFG->EXTICR[pin>>2], 4, portNum, (pin&0x3)*4);
        // Unmask the corresponding interrupt request
        common::set_reg_bit(EXTI->IMR1, SET, pin);
        // Set the triggering edge(s) for the line
        if (edge == edge::RISING) {
            common::set_reg_bit(EXTI->RTSR1, SET, pin);
        }
        else if (edge == edge::FALLING) {
            common::set_reg_bit(EXTI->FTSR1, SET, pin);
        }
        else {
            common::set_reg_bit(EXTI->RTSR1, SET, pin);
            common::set_reg_bit(EXTI->FTSR1, SET, pin);
        }

        if (callback != nullptr) {
            // Register the callback function
            cbkArray.add(pin, callback);
        }

        auto extIrq = static_cast<IRQn_Type>((pin >= 10 && pin <= 15) ?
                      EXTI15_10_IRQn : (pin >= 5 && pin <= 9) ?
                      EXTI9_5_IRQn   : EXTI0_IRQn + pin);
        
        // Enable the external interrupt request on the NVIC
        hw_core::irq_setPriority(extIrq, 0);
        hw_core::irq_enable(extIrq);
    }
}


/**
 * \brief Disable an interrupt for a pin
 * 
 * \param [in] pin : pin number
 */
void gpio::disableIrq(u16 pin) const
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (common::read_reg_bits(EXTI->IMR1, SET, pin)) {
        // Clear the pending interrupt request if any
        common::set_reg_bit(EXTI->PR1, SET, pin);
        // Reset the corresponding interrupt input source
        common::reset_reg_bits(SYSCFG->EXTICR[pin>>2], 15, (pin&0x3)*4);
        // Mask the interrupt request for this line
        common::reset_reg_bits(EXTI->IMR1, SET, pin);
        // Reset the trigger configuration for the line
        if (common::read_reg_bits(EXTI->RTSR1, SET, pin))
        {
            common::reset_reg_bits(EXTI->RTSR1, SET, pin);
        }
        if (common::read_reg_bits(EXTI->FTSR1, SET, pin))
        {
            common::reset_reg_bits(EXTI->FTSR1, SET, pin);
        }

        if (cbkArray.at(pin) != nullptr) {
            // Unregister the callback function
            cbkArray.at(pin) = nullptr;
        }
        
        auto extIrq = static_cast<IRQn_Type>((pin >= 10 && pin <= 15) ?
                      EXTI15_10_IRQn : (pin >= 5 && pin <= 9) ?
                      EXTI9_5_IRQn   : EXTI0_IRQn + pin);

        // Disable the external interrupt request on the NVIC
        hw_core::irq_disable(extIrq);
    }
}


/**
 * \brief Configure and enable an event for a pin
 * 
 * \param [in] pin  : pin number
 * \param [in] edge : edge polarity
 */
void gpio::enableEvent(u16 pin, edge edge)
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (inFlag.at(pin) == true)
    {
        // Enable the system configuration clock
        rcc::enableClock(drivers::SYSCFG_ID);
        // Configure the event source
        common::modify_reg(SYSCFG->EXTICR[pin>>2], 4, portNum, (pin&0x3)*4);
        // Unmask the corresponding event source
        common::set_reg_bit(EXTI->EMR1, SET, pin);
        // Set the triggering edge(s) for the line
        if (edge == edge::RISING) {
            common::set_reg_bit(EXTI->RTSR1, SET, pin);
        }
        else if (edge == edge::FALLING) {
            common::set_reg_bit(EXTI->FTSR1, SET, pin);
        }
        else {
            // Both rising and falling edges
            common::set_reg_bit(EXTI->RTSR1, SET, pin);
            common::set_reg_bit(EXTI->FTSR1, SET, pin);
        }
    }
}


/**
 * \brief Disable an event for a pin
 * 
 * \param [in] pin : pin number
 */
void gpio::disableEvent(u16 pin) const
{
    // Ensure the pin number is within valid range
    assert((pin < this->MAX_PINS), "Invalid pin number.");

    if (common::read_reg_bits(EXTI->EMR1, SET, pin))
    {
        // Reset the corresponding event source
        common::reset_reg_bits(SYSCFG->EXTICR[pin>>2], 15, (pin&0x3)*4);
        // Mask the event request for this line
        common::reset_reg_bits(EXTI->EMR1, SET, pin);
        // Reset the trigger configuration for the line
        if (common::read_reg_bits(EXTI->RTSR1, SET, pin))
        {
            // Reset the rising edge trigger for the line
            common::reset_reg_bits(EXTI->RTSR1, SET, pin);
        }
        if (common::read_reg_bits(EXTI->FTSR1, SET, pin))
        {
            // Reset the falling edge trigger for the line
            common::reset_reg_bits(EXTI->FTSR1, SET, pin);
        }
    }
}

section_ccmram
extern "C" void EXTI15_10_IRQHandler(void)
{
    for (auto i = 10; i <= 15; i++)
    {
        if (common::read_reg_bits(EXTI->PR1, SET, i)) {
            // Clear the pending bit
            common::set_reg_bit(EXTI->PR1, SET << i);

            if (gpio::cbkArray.at(i) != nullptr) {
                // Call the callback function
                gpio::cbkArray[i]();
            }
            break;
        }
    }
}

/**@}*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
