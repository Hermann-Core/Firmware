/**//************************************************************************************* 
 * \file 	 setup_app.c
 * \date   April, 01 2023
 * \author Awatsa Hermann
 * \brief	 Application setup.
 * ***********************************************************************************
 * \attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.04.01   |    1      |  0         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/**
 * \defgroup setupApp Application Setup
 * 
 * \ingroup core
 * Perform C/C++ runtime initializations before the execution of the main application.
 * These functions set up the C/C++ environment, initialize the stack pointer, copy
 * initialization data, zero-initialize the BSS segment, and call C++ static constructors.
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "peripherals_defs.h"


/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define ATTRIBUTE(a, b)            __attribute__((a, b))
#define NO_RETURN                  __attribute__((noreturn))
#define STATIC_INLINE               __attribute__((always_inline)) static
#define SET_WEAK_ALIAS             __attribute__((weak, alias("Default_Handler")))

#define _AEABI_PORTABILITY_LEVEL   1  /* Enable the Arm standard portability level */

#define CONSTRUCTOR_BASE           __init_array_base
#define CONSTRUCTOR_LIMIT          __init_array_limit



/*======================= Type definition ==========================================*/

typedef void (*VECTOR_TABLE_t)(void);
typedef void (*CONSTRUCTOR_t)(void);

typedef struct
{
  u32 loadAddress;
  u32 runAddress;
  u32 size;
}
CopyRecord_t;

typedef struct
{
  u16 recSize;
  u16 numRecs;
  CopyRecord_t recs[1];
}
CopyTable_t;



/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*================= External references ============================================*/

extern u32 __INITIAL_SP;
extern u32 __STACK_END;
extern u32 __sbss;
extern u32 __ebss;

extern CopyTable_t   const __binit__[];
extern CONSTRUCTOR_t const CONSTRUCTOR_BASE[];
extern CONSTRUCTOR_t const CONSTRUCTOR_LIMIT[];

extern int main(void);


/*================= Internal references ============================================*/

__attribute__((retain, section(".stack")))
int __stack;	/* Initialize the user mode stack */



/**
 * \brief Default exception handler
 */
NO_RETURN void Default_Handler(void)
{
  while(1);     /* infinite loop */
}


STATIC_INLINE void memcopy(u8 *dest, const u8 *source, u32 size)
{
  while(size--)
    *dest++ = *source++; 
}


/**
 * \brief Call all the C++ static constructors 
 */
STATIC_INLINE void __call_constructors(void)
{
    if (CONSTRUCTOR_BASE != CONSTRUCTOR_LIMIT)
    {
        u16 i = 0;

        while (&(CONSTRUCTOR_BASE[i]) != CONSTRUCTOR_LIMIT)
          /* Call the C++ static constructors */
          CONSTRUCTOR_BASE[i++]();
    }
}


/**
 * \brief Perfom the copies of all enties inside
 * 		  the linker generated copy table
 * 
 * \param [in] copy the address of the generated copy table
 */
STATIC_INLINE void __copy_table(CopyTable_t const *copy)
{
    for (u16 i = 0; i < copy->numRecs; i++)
    {
        CopyRecord_t cpyRec   = copy->recs[i];
        const u8*    loadAddr = (u8*)cpyRec.loadAddress;
        u8*          runAdrr  = (u8*)cpyRec.runAddress;

        if (cpyRec.size)
          memcopy(runAdrr, loadAddr, cpyRec.size);
    }
}


/**
 * \brief Zero initialized the bss segment 
 */
STATIC_INLINE void __zero_init(void)
{
    if (__sbss != __ebss)
    {
        u8* idx    = (u8*)&__sbss;
        u32 count  = (u32)(&__ebss - &__sbss);

        while (count--) *idx++ = 0;
    }
}


/**
 * \brief Setup and start the application
 */
NO_RETURN void __program_start(void)
{
    /* Setting up the C/C++ environment */
    if (__binit__ != (CopyTable_t*)-1)
      __copy_table((CopyTable_t const*)__binit__);

    __zero_init();
    __call_constructors();

    /* Set the stack pointer */
    __set_MSP((u32)&__STACK_END);

    __enable_irq();       /* Enable the global interrupts */
    __enable_fault_irq(); /* Enable fault exceptions */

    main();     /* Call the main function */

    while(1);   /* Will normally never be reached */
}


/*======================== Application vector table ================================*/

ATTRIBUTE(retain, section(".app_vector_table"))
const VECTOR_TABLE_t __APP_VECTOR_TABLE[] =
{
    (VECTOR_TABLE_t)&__STACK_END,
    &__program_start
};


#ifdef __cplusplus
}
#endif

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
