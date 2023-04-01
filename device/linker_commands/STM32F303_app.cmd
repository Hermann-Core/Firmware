/*************************************************************************************
 * @file 	 STM32F303_app.cmd
 * @date   March, 26 2023
 * @author AWATSA HERMANN
 * @brief	 STM32F303 linker command file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * This file is a linker description command file used by the TI linker
 * to produce an output file for the STM32F303 MCU
 *
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.26   |     1     |      0     |

*************************************************************************************/

/* Macros definitions. Only support by the TI linker */
#define CCMRAM_SIZE           0x2000
#define CCMRAM_ORIGIN         0x10000000
#define APP1_SIZE             0x19000
#define APP2_SIZE             0x19000
#define APP1_ORIGIN           0x0800B000
#define APP2_ORIGIN           0x08025000
#define SRAM_SIZE             0x9E78
#define SRAM_ORIGIN           0x20000188
#define RESERVED_ORIGIN       0x0803E800
#define RESERVED_SIZE         0x1800


/* Linker options */
--entry_point __program_start     /* Entry point of the program */
--retain="*(.ccmram)"             /* Do not discard the .ccmram section */
--retain="*(.init_array)"         /* Do not discard the .init_array section */
--stack_size=0x1000               /* Define a 4K stack */
--heap_size=0x200                 /* Define a 512 Octets heap */


MEMORY
{
  CCMRAM   (XRW)   : ORIGIN = CCMRAM_ORIGIN,    LENGTH = CCMRAM_SIZE
  RAM      (XRW)   : ORIGIN = SRAM_ORIGIN,      LENGTH = SRAM_SIZE
  APP1     (RX)    : ORIGIN = APP1_ORIGIN,      LENGTH = APP1_SIZE
  APP2     (RX)    : ORIGIN = APP2_ORIGIN,      LENGTH = APP2_SIZE
  RESERVED (RX)    : ORIGIN = RESERVED_ORIGIN,  LENGTH = RESERVED_SIZE
}

/* Initial stack value. Define at the end of the RAM memory region */
__INITIAL_SP = 0x20000000 + 0xA000;


SECTIONS
{
  .app_vector_table : > APP1_ORIGIN   /* Application vector table */

  GROUP : > APP1
  {
    .binit : {}         /* Linker generated copy tables section */
    .cinit : {}         /* compiler generated C/C++ initialization table section */

    .text :
    {
      . = align(4);     /* 4 bytes aligned the start of the .text section */
      *(.text)
      . = align(4);     /* 4 bytes aligned the end of the .text section */
    }
  }

  /* The read only datas section */
  .rodata : palign(4) {}
            load = APP1,
            run  = RAM,
            table(BINIT)

  /* Section containing some switch statements */
  .switch : ALIGN(4) {}
          load = APP1,
          run  = RAM,
          table(BINIT)

  /* Initialized global and static variables section */
  .data : ALIGN(4) {}
          load = APP1,
          run  = RAM,
          table(BINIT)

  /* C++ global constructors section */
  .init_array : ALIGN(4) {}
              load = APP1,
              run = RAM,
              RUN_START(__init_array_base),
              RUN_END(__init_array_limit),
              table(BINIT)

  .ccmram : ALIGN(4) {}
            load = APP1,
            run  = CCMRAM,
            table(BINIT)

  /* Uninitialized global and static variables section */
  .bss : ALIGN(4)
  {
    . = align(4);
    *(.bss)           /* Uninitialized datas section */
    . = align(4);

  } > RAM, type = NOLOAD,
      RUN_START(__sbss),
      RUN_END(__ebss)

  .stack :  > RAM     /* C/C++ runtime stack section */
  .sysmem : > RAM     /* Heap section used for the dynamic memory allocation */
}
