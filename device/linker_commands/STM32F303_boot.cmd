/*************************************************************************************
 * @file 	 STM32F303_boot.cmd
 * @date   March, 26 2023
 * @author AWATSA HERMANN
 * @brief	 STM32F303 bootloader linker command file
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
#define BOOT_SIZE             0x8000
#define BOOT_ORIGIN           0x08000000
#define SRAM_SIZE             0xA000
#define SRAM_ORIGIN           0x20000000


/* Linker options */
--entry_point __Setup_boot        /* Entry point of the program */
--retain="*(.vector_table)"       /* Do not discard the .vector_table section */
--retain="*(.ccmram)"             /* Do not discard the .ccmram section */
--retain="*(.init_array)"         /* Do not discard the .init_array section */
--stack_size=0x1000               /* Define a 4K stack */
--heap_size=0x200                 /* Define a 512 Octets heap */


MEMORY
{
  CCMRAM   (RWX)   : ORIGIN = CCMRAM_ORIGIN,  LENGTH = CCMRAM_SIZE
  RAM      (RWX)   : ORIGIN = SRAM_ORIGIN,    LENGTH = SRAM_SIZE
  BOOT     (RX)    : ORIGIN = BOOT_ORIGIN,    LENGTH = BOOT_SIZE
}

/* Initial stack value. Define at the end of the RAM memory */
__INITIAL_SP = SRAM_ORIGIN + SRAM_SIZE;


SECTIONS
{
  .vector_table : load = BOOT_ORIGIN,
                  run  = SRAM_ORIGIN,
                  RUN_START(__VECTOR_TABLE_BASE),
                  table(BINIT)

  GROUP : > BOOT
  {
    .binit : {}         /* Linker generated copy tables section */
    .cinit : {}         /* compiler generated C/C++ initialization table section */

    .text :
    {
      . = align(4);     /* 4 bytes aligned the start of the .text section */
      *(.text)
      . = align(4);     /* 4 bytes aligned the end of the .text section */
    }

    .rodata : palign(4)
    {
      . = align(4);
      *(.rodata)        /* The read only datas section */
      . = align(4);
    }

    .switch :
    {
      . = align(4);
      *(.switch)        /* Section containing some switch statements */
      . = align(4);
    }

    /* C++ global constructors section. define SHT$$INIT_ARRAY$$Base and SHT$$INIT_ARRAY$$Limit symbols */
    .init_array : {}
                START(__init_array_base),
                END(__init_array_limit)
  }

  /* Initialized global and static variables section */
  .data : ALIGN(4) {}
          load = BOOT,
          run  = RAM,
          table(BINIT)

  .ccmram : ALIGN(4) {}
          load = BOOT,
          run  = CCMRAM,
          table(BINIT)

  /* Uninitialized global and static variables section */
  .bss : ALIGN(4)
  {
    . = align(4);
    *(.bss)             /* Uninitialized datas section */
    . = align(4);

  } > RAM, type = NOLOAD,
      RUN_START(__sbss),
      RUN_END(__ebss)

  .stack :  > RAM       /* C/C++ runtime stack section */
  .sysmem : > RAM       /* Heap section used for the dynamic memory allocation */
}
