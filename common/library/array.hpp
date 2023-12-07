/************************************************************************************* 
 * @file   array.hpp
 * @date   Dec, 01 2023
 * @author Awatsa Hermann
 * @brief  interface used to handle the static objects container.
 * 
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.01.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

#ifndef _ARRAY_H_
#define _ARRAY_H_


/**
 * \defgroup library library
 * \brief Modules and classes providing shared functionality.
 * 
 * \defgroup array array
 * \ingroup library
 * lighter version of the array class. It will be used throughout the project to manage
 * static arrays. The class has been designed as a template to support all object types 
 * that will be instantiated throughout the project.
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common.hpp"



/*==================================================================================
|                             CLASSES DECLARATIONS                                
===================================================================================*/

/**
 * \brief This class provide functionalities for arrays manipulation with some features
 * such as bound check, operators overloading for user friendly manipulation and so on.
 */
template <typename T = char, size_t _size = 256>
class array
{
    public:

        using value_type        = T;
        using size_type         = size_t;
        using reference         = T&;
        using const_reference   = const T&;
        using pointer           = T*;
        using const_pointer     = const T*;

        /**
         * \brief Access an element at the specified index with bounds checking.
         *
         * \param [in] i : index of the element to access.
         * \return reference to the element at the specified index.
         */
        reference at(size_t i)
        {
            assert(i < _size, "the index is out of bound");

            return buffer_[i];
        }

        /**
         * \brief Access an element at the specified index with bounds checking (const version).
         *
         * \param [in] i : index of the element to access.
         * \return constant reference to the element at the specified index.
         */
        const_reference at(size_t i) const
        {
            assert(i < _size, "the index is out of bound");
            
            return buffer_[i];
        }

        /**
         * \brief Access to the element at a specific index using the familiar array-like syntax.
         *
         * \param [in] i : index of the element to access.
         * \return reference to the element at the specified index.
         */
        reference operator[](size_t i)
        {
            return buffer_[i];
        }

        /**
         * \brief Access to the element at a specific index using the familiar array-like syntax (const version).
         *
         * \param [in] i : index of the element to access.
         * \return constant reference to the element at the specified index.
         */
        constexpr const_reference operator[](size_t i) const
        {
            return buffer_[i];
        }

        /**
         * \brief Get the size of the array.
         *
         * \return size of the array.
         */
        constexpr size_t size() const
        {
            return _size;
        }

        /**
         * \brief Get a pointer to the beginning of the array.
         *
         * \return pointer to the beginning of the array.
         */
        pointer ptr()
        {
            return &buffer_[0];
        }

        /**
         * \brief Get a constant pointer to the beginning of the array.
         *
         * \return constant pointer to the beginning of the array.
         */
        constexpr const_pointer ptr() const
        {
            return &buffer_[0];
        }

        /**
         * \brief Erase the contents of the array by setting all elements to zero.
         */
        inline void erase()
        {
            common::fill(buffer_, buffer_ + _size, 0);
        }

        /**
         * \brief Erase elements starting from the specified position to the end of the array.
         *
         * \param [in] position : index at which to start erasing.
         */
        inline void erase_at(size_t position)
        {
            assert(position < _size, "the position is out of bound");
            common::fill(&buffer_[position], buffer_ + _size, 0);
        }

        /**
         * \brief Add a value at the specified position in the array.
         *
         * \param [in] position : index at which to add the value.
         * \param [in] value    : value to add to the array.
         */
        inline void add(size_t position, size_t value)
        {
            assert(position < _size, "the position is out of bound");
            buffer_[position] = value;
        }

    private:

        value_type buffer_[_size];
};

/** @} */


#endif      /* _ARRAY_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
