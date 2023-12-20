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
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
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
#include "const.hpp"
#include "rcc.hpp"
#include "hw_core.hpp"


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
using namespace driver;

/**
 * \brief Clear the configuration of the GPIO pin
 * 
 * \param [in] pin : pin number
 */
void gpio::clearCfg(u16 pin)
{
    assert((pin < MAX_PINS), "Invalid pin number.");

    // Reset the pin configuration
    locGPIO->MODER      &= ~(0x1UL << (pin * 2));
    locGPIO->OTYPER     &= ~(0x1UL << pin);
    locGPIO->OSPEEDR    &= ~(0x1UL << (pin * 2));
    locGPIO->PUPDR      &= ~(0x1UL << (pin * 2));
    locGPIO->AFR[pin/8] &= ~(0x1UL << (pin * 4));
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
            common::set_reg_bits(locGPIO->PUPDR, config, pin*2);
            break;
        default:
            // Set the pin as analog or input floating pin
            common::set_reg_bits(locGPIO->MODER, config, pin*2);
            break;
    }
    // Set the GPIO speed to the highest speed
    common::set_reg_bits(locGPIO->OSPEEDR, HIGH_SPEED, pin*2);
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
    common::set_reg_bits(locGPIO->MODER, (0x1UL << (pin*2)));
    // Set the output mode (push-pull or opain drain)
    common::set_reg_bits(locGPIO->OTYPER, (config << (pin*2)));
    // Set the GPIO speed to the highest speed
    common::set_reg_bits(locGPIO->OSPEEDR, HIGH_SPEED, pin*2);
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
    common::set_reg_bits(locGPIO->MODER, (0x2UL << (pin*2)));
    // Set the GPIO speed to the highest speed
    common::set_reg_bits(locGPIO->OSPEEDR, HIGH_SPEED, pin*2);
    // Set the alternate function number
    common::set_reg_bits(locGPIO->AFR[pin/8], config, (pin%8)*4);
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
gpio::gpio(u32 gpioID) : portNum(gpioID - 32U),
                         locGPIO(getPort(gpioID - 32U))
{
    assert((gpioID >= 32) && (gpioID <= 38), "Invalid port ID.");

    rcc::enableClock(gpioID);   // Enable the GPIO clock
    // Reset all the content of the arrays
    inFlag.erase(false);
    outFlag.erase(false);
    cbkArray.erase(nullptr);
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
        common::write_reg(locGPIO->BSRR, common::SET << pin);
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
        common::write_reg(locGPIO->BRR, common::SET << pin);
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
        if (common::read_reg_bits(locGPIO->ODR, common::SET << pin))
        {
            common::write_reg(locGPIO->BRR, common::SET << pin);
        }
        else {
            common::write_reg(locGPIO->BSRR, common::SET << pin);
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
        return common::read_reg_bits(locGPIO->IDR, common::SET, pin);
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
        u32 temp = common::LOCK_MASK;

        /* Applying the lock sequence */
        common::set_reg_bits(temp, common::SET, pin);
        common::write_reg(locGPIO->LCKR, temp);
        common::write_reg(locGPIO->LCKR, common::SET << pin);
        common::write_reg(locGPIO->LCKR, temp);
        temp = locGPIO->LCKR;
        if (temp & common::LOCK_MASK) {
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
        rcc::enableClock(common::SYSCFG_ID);
        // Configure the interrupt source
        common::set_reg_bits(SYSCFG->EXTICR[pin>>2], portNum, pin&0x3);
        // Unmask the corresponding interrupt request
        common::set_reg_bits(EXTI->IMR1, common::SET, pin);
        // Set the triggering edge(s) for the line
        if (edge == edge::RISING) {
            common::set_reg_bits(EXTI->RTSR1, common::SET, pin);
        }
        else if (edge == edge::FALLING) {
            common::set_reg_bits(EXTI->FTSR1, common::SET, pin);
        }
        else {
            common::set_reg_bits(EXTI->RTSR1, common::SET, pin);
            common::set_reg_bits(EXTI->FTSR1, common::SET, pin);
        }

        if (callback != nullptr) {
            // Register the callback function
            cbkArray.add(pin, callback);
        }

        auto extIrq = static_cast<IRQn_t>((pin >= 10 && pin <= 15) ?
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

    if (common::read_reg_bits(EXTI->IMR1, common::SET, pin)) {
        // Clear the pending interrupt request if any
        common::set_reg_bits(EXTI->PR1, common::SET, pin);
        // Reset the corresponding interrupt input source
        common::reset_reg_bits(SYSCFG->EXTICR[pin>>2], 0x4, pin&0x3);
        // Mask the interrupt request for this line
        common::reset_reg_bits(EXTI->IMR1, common::SET, pin);
        // Reset the trigger configuration for the line
        if (common::read_reg_bits(EXTI->RTSR1, common::SET, pin))
        {
            common::reset_reg_bits(EXTI->RTSR1, common::SET, pin);
        }
        if (common::read_reg_bits(EXTI->FTSR1, common::SET, pin))
        {
            common::reset_reg_bits(EXTI->FTSR1, common::SET, pin);
        }

        if (cbkArray.at(pin) != nullptr) {
            // Unregister the callback function
            cbkArray.at(pin) = nullptr;
        }
        
        auto extIrq = static_cast<IRQn_t>((pin >= 10 && pin <= 15) ?
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
        rcc::enableClock(common::SYSCFG_ID);
        // Configure the event source
        common::set_reg_bits(SYSCFG->EXTICR[pin>>2], portNum, pin&0x3);
        // Unmask the corresponding event request
        common::set_reg_bits(EXTI->EMR1, common::SET, pin);
        // Set the triggering edge(s) for the line
        if (edge == edge::RISING) {
            common::set_reg_bits(EXTI->RTSR1, common::SET, pin);
        }
        else if (edge == edge::FALLING) {
            common::set_reg_bits(EXTI->FTSR1, common::SET, pin);
        }
        else {
            common::set_reg_bits(EXTI->RTSR1, common::SET, pin);
            common::set_reg_bits(EXTI->FTSR1, common::SET, pin);
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

    if (common::read_reg_bits(EXTI->EMR1, common::SET, pin))
    {
        // Reset the corresponding event source
        common::reset_reg_bits(SYSCFG->EXTICR[pin>>2], 0x4, pin&0x3);
        // Mask the event request for this line
        common::reset_reg_bits(EXTI->EMR1, common::SET, pin);
        // Reset the trigger configuration for the line
        if (common::read_reg_bits(EXTI->RTSR1, common::SET, pin))
        {
            common::reset_reg_bits(EXTI->RTSR1, common::SET, pin);
        }
        if (common::read_reg_bits(EXTI->FTSR1, common::SET, pin))
        {
            common::reset_reg_bits(EXTI->FTSR1, common::SET, pin);
        }
    }
}


extern "C" void EXTI15_10_IRQHandler(void)
{
    for (auto i = 10; i <= 15; i++)
    {
        if (common::read_reg_bits(EXTI->PR1, common::SET, i))
        {
            common::set_reg_bits(EXTI->PR1, common::SET, i);

            if (gpio::cbkArray.at(i) != nullptr)
            {
                gpio::cbkArray[i]();
                break;
            }
        }
    }
}

/**@}*/


/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
