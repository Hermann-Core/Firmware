/************************************************************************************* 
 * @file   hardware_core.hpp
 * @date   March, 12 2023
 * @author Awatsa Hermann
 * @brief  hardware core functions class
 * 
 *         Contains the declarations of
           the core hardware functions
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.12   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _HARDWARE_CORE_H_
#define _HARDWARE_CORE_H_


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common.hpp"



/*==================================================================================
|                             FUNCTIONS DECLARATIONS                                
===================================================================================*/

/**
 * \class core hardware functions interface
 */
class hw_core
{
    public:
        static void disable_all_irq(void);
        static void disable_fault_irq(void);
        static void irq_enable(IRQn_t IRQn);
        static void irq_disable(IRQn_t IRQn);
        static void irq_clearPending(IRQn_t IRQn);
        static void irq_setPriority(IRQn_t IRQn, u8 priority);
        static void systemReset(void);
};


#endif      /* _HARDWARE_CORE_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
