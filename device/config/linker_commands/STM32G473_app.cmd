/*************************************************************************************
 * @file   STM32G473_app.cmd
 * @date   Nov, 29 2023
 * @author Awatsa Hermann
 * @brief  STM32G473 application linker command file
 * 
 * ***********************************************************************************
 * @attention
 * 
 * This file is a linker description command file used by the TI linker
 * to produce an output file for the STM32G473 MCU
 *
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.29.11   |     1     |      1     |

*************************************************************************************/

/* Macros definitions. Only support by the TI linker */
#define CCMRAM_SIZE           0x8000      /* 32KB */
#define CCMRAM_ORIGIN         0x10000000
#define APP_SIZE              0x32000     /* 200KB */
#define SLOT_SIZE             0x32000     /* 200KB */
#define APP_ORIGIN            0x0800D000
#define SLOT_ORIGIN           0x08044000
#define SRAM_SIZE             0x17E28     /* 95.54KB */
#define SRAM_ORIGIN           0x200001D8
#define CRC_ORIGIN            0x0803F000
#define CRC_SIZE              0x40        /* 64B */


/* Linker options */
--entry_point __program_start     /* Entry point of the program */
--retain="*(.ccmram)"             /* Do not discard the .ccmram section */
--retain="*(.init_array)"         /* Do not discard the .init_array section */
--stack_size=0x1000               /* Define a 4K stack */
--heap_size=0x200                 /* Define a 512 Octets heap */


MEMORY
{
  GROUP
  {
    APP (RX) : ORIGIN = APP_ORIGIN,  LENGTH = APP_SIZE

  } crc(__crc_table__, algorithm=CRC32_PRIME)

  CCMRAM   (XRW)   : ORIGIN = CCMRAM_ORIGIN,    LENGTH = CCMRAM_SIZE
  RAM      (XRW)   : ORIGIN = SRAM_ORIGIN,      LENGTH = SRAM_SIZE
  SLOT     (RX)    : ORIGIN = SLOT_ORIGIN,      LENGTH = SLOT_SIZE
  CRC_REG  (RX)    : ORIGIN = CRC_ORIGIN,       LENGTH = CRC_SIZE
}

/* Initial stack value. Define at the end of the RAM memory region */
__INITIAL_SP = 0x20000000 + 0x18000;

/* Address of the reset region */
RESET_BASE = 0x0807B000;


SECTIONS
{
  .app_vector_table : > APP_ORIGIN   /* Application vector table */

  .TI.memcrc > CRC_REG               /* generated CRC table section */

  GROUP : > APP
  {
    .binit : {}         /* Linker generated copy tables section */

    .text :
    {
      . = align(4);     /* 4 bytes aligned the start of the .text section */
      *(.text)
      . = align(4);     /* 4 bytes aligned the end of the .text section */
    }
  }

  /* The read only datas section */
  .rodata : palign(4) {}
            load = APP,
            run  = RAM,
            table(BINIT)

  /* Section containing some switch statements */
  .switch : ALIGN(4) {}
          load = APP,
          run  = RAM,
          table(BINIT)

  /* Initialized global and static variables section */
  .data : ALIGN(4) {}
          load = APP,
          run  = RAM,
          table(BINIT)

  /* C++ global constructors section */
  .init_array : ALIGN(4) {}
              load = APP,
              run = RAM,
              RUN_START(__init_array_base),
              RUN_END(__init_array_limit),
              table(BINIT)

  .ccmram : ALIGN(4) {}
            load = APP,
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
