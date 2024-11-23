/************************************************************************************* 
 * @file   hw_core.hpp
 * @date   Dec, 14 2023
 * @author Awatsa Hermann
 * @brief  hardware core functionalities
 * ***********************************************************************************
 * @attention
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.12.14   |    1      |  0         |
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
#include "periph_def.h"



/*==================================================================================
|                             FUNCTIONS DECLARATIONS                                
===================================================================================*/

/**
 * \brief Core hardware utilities functions
 */
namespace hw_core
{
    void disable_all_irq(void);
    void disable_fault_irq(void);
    void irq_enable(IRQn_Type IRQn);
    void irq_disable(IRQn_Type IRQn);
    void irq_clearPending(IRQn_Type IRQn);
    void irq_setPriority(IRQn_Type IRQn, u8 priority);
    void systemReset(void);
    void initTicks(u32 ticks);
};


#endif      /* _HARDWARE_CORE_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
