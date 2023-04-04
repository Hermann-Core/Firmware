/************************************************************************************* 
 * @file 	 setup_boot.c
 * @date     March, 31 2023
 * @author   AWATSA HERMANN
 * @brief	 Device setup source file
 * 
 *           This file contains the functions used to perform
 *           the early setup of the device just after a reset
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation on other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.31   |    1      |  0         |

*************************************************************************************/

/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "peripherals_defs.h"


/************************************************************************************#
|                                       DEFINES                                      |
#************************************************************************************/
#define ATTRIBUTE(a, b)            __attribute__((a, b))
#define NO_RETURN                  __attribute__((noreturn))
#define STATIC_INLINE              __attribute__((always_inline)) static
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


/************************************************************************************#
|                              FUNCTIONS DEFINITIONS                                 |
#************************************************************************************/
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

extern int  main(void);
extern void FPU_Init(void);
extern void SystemClock_Init(void);

extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);


/*================= Internal references ============================================*/

__attribute__((retain, section(".stack")))
int __stack;	/* Initialize the user mode stack */

void WWDG_IRQHandler(void) SET_WEAK_ALIAS;
void PVD_IRQHandler(void) SET_WEAK_ALIAS;
void TAMP_STAMP_IRQHandler(void) SET_WEAK_ALIAS;
void RTC_WKUP_IRQHandler(void) SET_WEAK_ALIAS;
void FLASH_IRQHandler(void) SET_WEAK_ALIAS;
void RCC_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI0_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI1_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI2_TSC_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI3_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI4_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel1_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel2_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel3_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel4_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel5_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel6_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel7_IRQHandler(void) SET_WEAK_ALIAS;
void ADC1_2_IRQHandler (void) SET_WEAK_ALIAS;
#if defined (STM32F303)
void USB_HP_CAN_TX_IRQHandler(void) SET_WEAK_ALIAS;
void USB_LP_CAN_RX0_IRQHandler(void) SET_WEAK_ALIAS;
void CAN_RX1_IRQHandler(void) SET_WEAK_ALIAS;
void CAN_SCE_IRQHandler(void) SET_WEAK_ALIAS;
void USB_HP_IRQHandler(void) SET_WEAK_ALIAS;
void USB_LP_IRQHandler(void) SET_WEAK_ALIAS;
void USBWakeUp_RMP_IRQHandler(void) SET_WEAK_ALIAS;
void FPU_IRQHandler(void) SET_WEAK_ALIAS;
#elif defined (STM32G473)
void USB_HP_IRQHandler(void) SET_WEAK_ALIAS;
void USB_LP_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN1_IT0_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN1_IT1_IRQHandler(void) SET_WEAK_ALIAS;
void FMC_IRQHandler(void) SET_WEAK_ALIAS;
void LPTIM1_IRQHandler(void) SET_WEAK_ALIAS;
void TIM5_IRQHandler(void) SET_WEAK_ALIAS;
void ADC5_IRQHandler(void) SET_WEAK_ALIAS;
void UCPD1_IRQHandler(void) SET_WEAK_ALIAS;
void CRS_IRQHandler(void) SET_WEAK_ALIAS;
void SAI1_IRQHandler(void) SET_WEAK_ALIAS;
void TIM20_BRK_IRQHandler(void) SET_WEAK_ALIAS;
void TIM20_UP_IRQHandler(void) SET_WEAK_ALIAS;
void TIM20_TRG_COM_IRQHandler(void) SET_WEAK_ALIAS;
void TIM20_CC_IRQHandler(void) SET_WEAK_ALIAS;
void FPU_IRQHandler(void) SET_WEAK_ALIAS;
void I2C4_EV_IRQHandler(void) SET_WEAK_ALIAS;
void I2C4_ER_IRQHandler(void) SET_WEAK_ALIAS;
void SPI4_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN2_IT0_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN2_IT1_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN3_IT0_IRQHandler(void) SET_WEAK_ALIAS;
void FDCAN3_IT1_IRQHandler(void) SET_WEAK_ALIAS;
void RNG_IRQHandler(void) SET_WEAK_ALIAS;
void LPUART1_IRQHandler(void) SET_WEAK_ALIAS;
void I2C3_EV_IRQHandler(void) SET_WEAK_ALIAS;
void I2C3_ER_IRQHandler(void) SET_WEAK_ALIAS;
void DMAMUX_OVR_IRQHandler(void) SET_WEAK_ALIAS;
void QUADSPI_IRQHandler(void) SET_WEAK_ALIAS;
void DMA1_Channel8_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel6_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel7_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel8_IRQHandler(void) SET_WEAK_ALIAS;
void CORDIC_IRQHandler(void) SET_WEAK_ALIAS;
void FMAC_IRQHandler(void) SET_WEAK_ALIAS;
#endif
void EXTI9_5_IRQHandler(void) SET_WEAK_ALIAS;
void TIM1_BRK_TIM15_IRQHandler(void) SET_WEAK_ALIAS;
void TIM1_UP_TIM16_IRQHandler(void) SET_WEAK_ALIAS;
void TIM1_TRG_COM_TIM17_IRQHandler(void) SET_WEAK_ALIAS;
void TIM1_CC_IRQHandler(void) SET_WEAK_ALIAS;
void TIM2_IRQHandler(void) SET_WEAK_ALIAS;
void TIM3_IRQHandler(void) SET_WEAK_ALIAS;
void TIM4_IRQHandler(void) SET_WEAK_ALIAS;
void I2C1_EV_IRQHandler(void) SET_WEAK_ALIAS;
void I2C1_ER_IRQHandler(void) SET_WEAK_ALIAS;
void I2C2_EV_IRQHandler(void) SET_WEAK_ALIAS;
void I2C2_ER_IRQHandler(void) SET_WEAK_ALIAS;
void SPI1_IRQHandler(void) SET_WEAK_ALIAS;
void SPI2_IRQHandler(void) SET_WEAK_ALIAS;
void USART1_IRQHandler(void) SET_WEAK_ALIAS;
void USART2_IRQHandler(void) SET_WEAK_ALIAS;
void USART3_IRQHandler(void) SET_WEAK_ALIAS;
void EXTI15_10_IRQHandler(void) SET_WEAK_ALIAS;
void RTC_Alarm_IRQHandler(void) SET_WEAK_ALIAS;
void USBWakeUp_IRQHandler(void) SET_WEAK_ALIAS;
void TIM8_BRK_IRQHandler(void) SET_WEAK_ALIAS;
void TIM8_UP_IRQHandler(void) SET_WEAK_ALIAS;
void TIM8_TRG_COM_IRQHandler(void) SET_WEAK_ALIAS;
void TIM8_CC_IRQHandler(void) SET_WEAK_ALIAS;
void ADC3_IRQHandler(void) SET_WEAK_ALIAS;
void SPI3_IRQHandler(void) SET_WEAK_ALIAS;
void UART4_IRQHandler(void) SET_WEAK_ALIAS;
void UART5_IRQHandler(void) SET_WEAK_ALIAS;
void TIM6_DAC_IRQHandler(void) SET_WEAK_ALIAS;
void TIM7_DAC_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel1_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel2_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel3_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel4_IRQHandler(void) SET_WEAK_ALIAS;
void DMA2_Channel5_IRQHandler(void) SET_WEAK_ALIAS;
void ADC4_IRQHandler(void) SET_WEAK_ALIAS;
void COMP1_2_3_IRQHandler(void) SET_WEAK_ALIAS;
void COMP4_5_6_IRQHandler(void) SET_WEAK_ALIAS;
void COMP7_IRQHandler(void) SET_WEAK_ALIAS;


/**
 * @brief Default exception handler
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
 * @brief Perform the low level initialization of the hardware
 */
STATIC_INLINE void __hardware_init(void)
{
  __disable_irq();   	/* Prevents unexpected interrupts during startup */

  FPU_Init();			/* Initialize the Floating point coprocessor */
  SystemClock_Init();	/* Initialize the system clock */
}


/**
 * @brief Call all the C++ static constructors 
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
 * @brief Perfom the copies of all enties inside
 * 		  the linker generated copy table
 * 
 * @param copy the address of the generated copy table
 */
STATIC_INLINE void __copy_table(CopyTable_t const *copy)
{
  for (u16 i = 0; i < copy->numRecs; i++)
  {
    CopyRecord_t cpyRec = copy->recs[i];
    u8 *loadAddr = (u8*)cpyRec.loadAddress;
    u8 *runAdrr  = (u8*)cpyRec.runAddress;

    if (cpyRec.size)
      memcopy(runAdrr, loadAddr, cpyRec.size);
  }
}


/**
 * @brief Zero initialized the bss segment 
 */
STATIC_INLINE void __zero_init(void)
{
  if (__sbss != __ebss)
  {
    u8 *idx   = (u8*)&__sbss;
    u32 count = (u32)&__ebss - (u32)&__sbss;

    while (count--) *idx++ = 0;
  } 
}


/**
 * @brief Start the C/C++ environment
 */
NO_RETURN void __program_start(void)
{
  /* Setting up the C/C++ environment */
  if (__binit__ != (CopyTable_t*)-1)
    __copy_table((CopyTable_t const*)__binit__);
  
  __zero_init();
  __call_constructors();

  __enable_irq();   /* Reactivated global interrupts */

  main();           /* Call the main function */

  while(1);         /* Will normally never be reached */
}


/**
 * @brief Setting up the device
 */
NO_RETURN void __setup_boot(void)
{
  __set_MSP((u32)&__STACK_END);	/* Set the stack pointer */
  
  __hardware_init();  			/* Initialize the hardware */
  __program_start();  			/* Run the C/C++ environment */
}


/*=============================== Vector table =====================================*/

ATTRIBUTE(retain, section(".vector_table"))
const VECTOR_TABLE_t __VECTOR_TABLE[] =
{
	(VECTOR_TABLE_t)&__INITIAL_SP,
	&__setup_boot,

  	/* Cortex exeptions handlers addresses */
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

  	/* STM32 specific interrupts handlers addresses */
  	WWDG_IRQHandler,
	PVD_IRQHandler,
	TAMP_STAMP_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_TSC_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_2_IRQHandler,
#if defined (STM32F303)
  	USB_HP_CAN_TX_IRQHandler,
	USB_LP_CAN_RX0_IRQHandler,
	CAN_RX1_IRQHandler,
	CAN_SCE_IRQHandler,
#elif defined (STM32G473)
  	USB_HP_IRQHandler,
	USB_LP_IRQHandler,
	FDCAN1_IT0_IRQHandler,
	FDCAN1_IT1_IRQHandler,
#endif
  	EXTI9_5_IRQHandler,
	TIM1_BRK_TIM15_IRQHandler,
	TIM1_UP_TIM16_IRQHandler,
	TIM1_TRG_COM_TIM17_IRQHandler,
	TIM1_CC_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	USBWakeUp_IRQHandler,
	TIM8_BRK_IRQHandler,
	TIM8_UP_IRQHandler,
	TIM8_TRG_COM_IRQHandler,
	TIM8_CC_IRQHandler,
	ADC3_IRQHandler,
#if defined (STM32F303)
	0,
	0,
	0,
#elif defined (STM32G473)
  	FMC_IRQHandler,
	LPTIM1_IRQHandler,
	TIM5_IRQHandler,
#endif
  	SPI3_IRQHandler,
	UART4_IRQHandler,
	UART5_IRQHandler,
	TIM6_DAC_IRQHandler,
	TIM7_DAC_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_IRQHandler,
	DMA2_Channel5_IRQHandler,
	ADC4_IRQHandler,
#if defined (STM32F303)
	0,
	0,
#elif defined (STM32G473)
  	ADC5_IRQHandler,
	UCPD1_IRQHandler,
#endif
  	COMP1_2_3_IRQHandler,
	COMP4_5_6_IRQHandler,
	COMP7_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
#if defined (STM32F303)
  	USB_HP_IRQHandler,
	USB_LP_IRQHandler,
	USBWakeUp_RMP_IRQHandler,
	0,
	0,
	0,
	0,
	FPU_IRQHandler,
#elif defined (STM32G473)
	0,
	CRS_IRQHandler,
	SAI1_IRQHandler,
	TIM20_BRK_IRQHandler,
	TIM20_UP_IRQHandler,
	TIM20_TRG_COM_IRQHandler,
	TIM20_CC_IRQHandler,
	FPU_IRQHandler,
	I2C4_EV_IRQHandler,
	I2C4_ER_IRQHandler,
	SPI4_IRQHandler,
	0,
	FDCAN2_IT0_IRQHandler,
	FDCAN2_IT1_IRQHandler,
	FDCAN3_IT0_IRQHandler,
	FDCAN3_IT1_IRQHandler,
	RNG_IRQHandler,
	LPUART1_IRQHandler,
	I2C3_EV_IRQHandler,
	I2C3_ER_IRQHandler,
	DMAMUX_OVR_IRQHandler,
	QUADSPI_IRQHandler,
	DMA1_Channel8_IRQHandler,
	DMA2_Channel6_IRQHandler,
	DMA2_Channel7_IRQHandler,
	DMA2_Channel8_IRQHandler,
	CORDIC_IRQHandler,
	FMAC_IRQHandler,
#endif
};

#ifdef __cplusplus
}
#endif

/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
