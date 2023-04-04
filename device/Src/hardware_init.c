/************************************************************************************* 
 * @file 	 hardware_init.c
 * @date     March, 31 2023
 * @author   AWATSA HERMANN
 * @brief	 Hardware initialization source file
 * 
 *           This file Contains the definitions of all the functions
 *           used to performs the low level initialization of the MCU
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.03.31   |    1      |  0         |

*************************************************************************************/

/************************************************************************************#
|                                      INCLUDES                                      |
#************************************************************************************/
#include "hardware_core.h"
#include "peripherals_defs.h"


/************************************************************************************#
|                                       DEFINES                                      |
#************************************************************************************/
#define FORCE_INLINE    __attribute__((always_inline))

#if defined (STM32F303)
#define FLASH_LATENCY   2
#elif defined (STM32G473)
#define FLASH_LATENCY   4
#endif

#define ON              1
#define OFF             0
#define PLL_SRC         2
#define PLL_MUL_7       7
#define HSE_SRC         3
#define PLLM_6          5
#define PLLN_85         85



/************************************************************************************#
|                              FUNCTIONS DEFINITIONS                                 |
#************************************************************************************/

/**
 * @brief Initialize the floating point coprocessor 
 */
FORCE_INLINE void FPU_Init(void)
{
    /* set Full Access to the FPU coprocessor */
    SCB->CPACR |= ((0x3U << 20U) | (0x3U << 22U));
}


/**
 * @brief Initialize the system clock
 */
void SystemClock_Init(void)
{
    /* Clear all the reset interrupts flags */
    RCC->CSR_b.RMVF             = ON;
    
    /* Configure the flash memory latency */
    FLASH->ACR_b.LATENCY        = FLASH_LATENCY;
    while(FLASH->ACR_b.LATENCY != FLASH_LATENCY);

    /* Enable the HSE oscillator and wait for it to become ready */
    RCC->CR_b.HSEON             = ON;
    while(RCC->CR_b.HSERDY     != ON);

    /* Configure the RTC backup domain registers */
    RCC->APB1ENR_b.PWREN        = ON;
    PWR->CR_b.DBP               = ON;
    if (RCC->BDCR_b.RTCSEL     != ON)
    {
        RCC->BDCR_b.BDRST       = ON;
        RCC->BDCR_b.BDRST       = OFF;
    }
    RCC->BDCR_b.LSEDRV          = OFF;

#if defined (STM32G473)

    /* Enable the LSE oscillator and wait for it to become ready */
    RCC->BDCR_b.LSEON           = ON;
    while(RCC->BDCR_b.LSERDY   != ON);

    /* Set the LSE as the RTC and backup registers clock */
    if (RCC->BDCR_b.RTCSEL     != ON)
        RCC->BDCR_b.RTCSEL      = ON;

    /* Configure the flash memory accelerators */
    FLASH->ACR_b.DCEN           = ON;
    FLASH->ACR_b.ICEN           = ON;
    FLASH->ACR_b.PRFTEN         = ON;
    PWR->CR5                   &= ~(0x1 << 8U);

    /* Enable the high speed 48 Mhz internal oscillator */
    RCC->CRRCR_b.HSI48ON         = ON;
    while(RCC->CRRCR_b.HSI48RDY != ON);

    /* Configure the phase lock loop (PLL) for 170 Mhz clock operation */
    RCC->PLLCFGR_b.PLLSRC      = HSE_SRC;
    RCC->PLLCFGR_b.PLLM        = PLLM_6;
    RCC->PLLCFGR_b.PLLN        = PLLN_85;
    RCC->PLLCFGR_b.PLLR        = OFF;
    RCC->PLLCFGR_b.PLLREN      = ON;
    RCC->CFGR_b.MCOSEL         = ON;
#elif defined (STM32F303)

    /* Enable the flash memory prefetch buffer */
    FLASH->ACR_b.PRFTBE         = ON;

    /* Configure the phase lock loop (PLL) for 72 Mhz clock operation */
    RCC->CFGR_b.PLLSRC          = PLL_SRC;
    RCC->CFGR_b.PLLXTPRE        = OFF;
    RCC->CFGR_b.PLLMUL          = PLL_MUL_7;
    RCC->CFGR_b.MCO             = 4;
#endif

    /* Enable the PLL and wait for it to become ready */
    RCC->CR_b.PLLON             = ON;
    while(RCC->CR_b.PLLRDY     != ON);

    /* Set the AHB and APB2 bus clocks prescalers */
    RCC->CFGR_b.HPRE            = OFF;
    RCC->CFGR_b.PPRE2           = OFF;

    /* Set the PLL as the system clock and wait for it to become ready */
    RCC->CFGR_b.SW              = PLL_SRC;
    while(RCC->CFGR_b.SWS      != PLL_SRC);
    ;

#if defined (STM32F303)

    RCC->CFGR_b.PPRE1           = 4;     /* Set the APB1 prescaler to 2 */
    RCC->CFGR3_b.TIM1SW         = ON;    /* Set PLL output as TIMER1 clock source */
    RCC->CFGR3_b.TIM8SW         = ON;    /* Set PLL output as TIMER8 clock source */
    RCC->CFGR3_b.I2C1SW         = ON;    /* Set System clock as I2C1 clock source */
    RCC->CFGR3_b.I2C2SW         = ON;    /* Set System clock as I2C2 clock source */
    RCC->CFGR3_b.USART1SW       = ON;    /* Set System clock as USART1 clock source */
    RCC->CFGR3_b.USART2SW       = ON;    /* Set System clock as USART2 clock source */
    RCC->CFGR3_b.USART3SW       = ON;    /* Set System clock as USART3 clock source */
    RCC->CFGR3_b.UART4SW        = ON;    /* Set System clock as UART4 clock source */
    RCC->CFGR3_b.UART5SW        = ON;    /* Set System clock as UART5 clock source */
#elif defined (STM32G473)

    RCC->CFGR_b.PPRE1           = OFF;   /* Set the APB1 prescaler to 1 */
    RCC->CCIPR_b.FDCANSEL       = 2;     /* Set the APB clock as FDCAN clock source */
    RCC->CCIPR_b.ADC12SEL       = 2;     /* Set the system clock as ADC1,2 clock source */
    RCC->CCIPR_b.ADC345SEL      = 2;     /* Set the system clock as ADC1,2,3 clock source */
#endif
    IRQ_Enable(NonMaskableInt_IRQn);
}

/************************************************************************************#
|                                    END OF FILE                                     |
#************************************************************************************/
