/************************************************************************************* 
 * @file   flash.cpp
 * @date   Dec, 21 2023
 * @author Awatsa Hermann
 * @brief  This file contains the interface used to handle the flash memory
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G474 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.21.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "flash.hpp"
#include "assert.h"



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/


/**
 * \defgroup flash Flash Memory Interface
 * \ingroup drivers
 * \brief This file contains the interface used to handle the flash memory.
 * It is used to handle the common flash operations such as erasing pages, erasing
 * the entire flash memory, read and program the flash memory though the flash memory
 * interface controller.
 * 
 * @{
 */

/*==================================================================================
|                         PRIVATE FUNCTIONS DEFINITIONS                                
===================================================================================*/
using namespace driver;

void flash::IRQ_Handler()
{
    
}


/*==================================================================================
|                         PUBLIC FUNCTIONS DEFINITIONS                                
===================================================================================*/

/**
 * \brief Erases a flash page at the specified address.
 *
 * @param [in] address : the address of the page to erase
 * @return true if the operation was successful, false otherwise
 */
bool flash::erase(const u32 address)
{
    assert(address % PAGE_SIZE == 0, "Address is not a page address!");
}


/**
 * \brief Erases a specified number of flash pages. It erases
 * the pages starting from a specified address @ref address
 *
 * @param [in] address : the starting address of the pages
 * @param [in] nbPages : the number of pages to erase
 * @return true if the operation was successful, false otherwise
 */
bool flash::erase(const u32 address, const size_t nbPages)
{
    assert(address % PAGE_SIZE == 0, "Address is not a page address!");
}


/**
 * \brief Erases the entire flash memory
 *
 * @return true if the operation was successful, false otherwise
 */
bool flash::massErase()
{
    
}


/**
 * \brief Reads the content of the flash memory starting from
 * the specified address @ref address.
 *
 * @param [in] address : starting address of the flash memory to read
 * @param [out] buffer : the buffer to store the read data
 * @param [in] size : the size of the data to read
 * @return true if the operation was successful, false otherwise
 */
bool flash::read(const u32 address, u32* buffer, const size_t size)
{
    assert(address % PAGE_SIZE == 0, "Address is not a page address!");
    assert(buffer != nullptr, "Buffer is null!");
}


/**
 * \brief Reads the value of a flash memory at the specified address
 *
 * @param [in] address : the address of the flash memory to read
 * @return the value of the flash memory at the specified address
 */
u32 flash::read(const u32 address)
{
    
}


/**
 * \brief Programs the flash memory starting from the specified address.
 *
 * @param [in] address : starting address of the flash memory to program
 * @param [in] buffer : the buffer containing the data to program
 * @param [in] size : the size of the data to program
 * @return true if the operation was successful, false otherwise
 */
template<typename T>
bool flash::program(const u32 address, const T* buffer, const size_t size)
{
    
}


/**
 * \brief Programs a value to the flash memory.
 *
 * @param [in] address : the address of the flash memory to program
 * @param [in] value : the value to write
 * @return true if the operation was successful, false otherwise
 */
template<typename T>
bool flash::program(const u32 address, const T value)
{
    
}


/**
 * \brief Protects a flash memory region
 *
 * @param [in] startAddress : the starting address of the region
 * @param [in] size : the size of the flash memory region
 * @return true if the operation was successful, false otherwise
 */
bool flash::protect(const u32 startAddress, const size_t size)
{
    
}


extern "C" void FLASH_IRQHandler(void)
{
    
}


/**@}*/

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
