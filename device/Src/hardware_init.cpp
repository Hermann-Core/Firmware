/************************************************************************************* 
 * @file   hardware_init.c
 * @date   March, 31 2023
 * @author AWATSA HERMANN
 * @brief  Hardware initialization source file
 * 
 *         This file Contains the definitions of all the functions
 *         used to performs the low level initialization of the MCU
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

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "hardware_core.hpp"


/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define FORCE_INLINE    __attribute__((always_inline))

#if defined (STM32F303)
#define FLASH_LATENCY   2
#elif defined (STM32G473)
#define FLASH_LATENCY   4
#endif

#define SET             1
#define CLEAR           0
#define PLL_SRC         2
#define PLL_MUL_7       7
#define HSE_SRC         3
#define PLLM_6          5
#define PLLN_85         85



/*==================================================================================
|                             FUNCTIONS DEFINITIONS                                
===================================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

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
    RCC->CSR_b.RMVF             = SET;
    
    /* Configure the flash memory latency */
    FLASH->ACR_b.LATENCY        = FLASH_LATENCY;
    while(FLASH->ACR_b.LATENCY != FLASH_LATENCY);

    /* Enable the HSE oscillator and wait for it to become ready */
    RCC->CR_b.HSEON             = SET;
    while(RCC->CR_b.HSERDY     != SET);

    /* Configure the RTC backup domain registers */
    RCC->APB1ENR_b.PWREN        = SET;
    PWR->CR_b.DBP               = SET;
    if (RCC->BDCR_b.RTCSEL     != SET)
    {
        RCC->BDCR_b.BDRST       = SET;
        RCC->BDCR_b.BDRST       = CLEAR;
    }
    RCC->BDCR_b.LSEDRV          = CLEAR;

#if defined (STM32G473)

    /* Enable the LSE oscillator and wait for it to become ready */
    RCC->BDCR_b.LSEON           = SET;
    while(RCC->BDCR_b.LSERDY   != SET);

    /* Set the LSE as the RTC and backup registers clock */
    if (RCC->BDCR_b.RTCSEL     != SET)
        RCC->BDCR_b.RTCSEL      = SET;

    /* Configure the flash memory accelerators */
    FLASH->ACR_b.DCEN           = SET;
    FLASH->ACR_b.ICEN           = SET;
    FLASH->ACR_b.PRFTEN         = SET;
    PWR->CR5                   &= ~(0x1 << 8U);

    /* Enable the high speed 48 Mhz internal oscillator */
    RCC->CRRCR_b.HSI48ON         = SET;
    while(RCC->CRRCR_b.HSI48RDY != SET);

    /* Configure the phase lock loop (PLL) for 170 Mhz clock operation */
    RCC->PLLCFGR_b.PLLSRC      = HSE_SRC;
    RCC->PLLCFGR_b.PLLM        = PLLM_6;
    RCC->PLLCFGR_b.PLLN        = PLLN_85;
    RCC->PLLCFGR_b.PLLR        = CLEAR;
    RCC->PLLCFGR_b.PLLREN      = SET;
    RCC->CFGR_b.MCOSEL         = SET;
#elif defined (STM32F303)

    /* Enable the flash memory prefetch buffer */
    FLASH->ACR_b.PRFTBE         = SET;

    /* Configure the phase lock loop (PLL) for 72 Mhz clock operation */
    RCC->CFGR_b.PLLSRC          = PLL_SRC;
    RCC->CFGR_b.PLLXTPRE        = CLEAR;
    RCC->CFGR_b.PLLMUL          = PLL_MUL_7;
    RCC->CFGR_b.MCO             = 4;
#endif

    /* Enable the PLL and wait for it to become ready */
    RCC->CR_b.PLLON             = SET;
    while(RCC->CR_b.PLLRDY     != SET);

    /* Set the AHB and APB2 bus clocks prescalers */
    RCC->CFGR_b.HPRE            = CLEAR;
    RCC->CFGR_b.PPRE2           = CLEAR;

    /* Set the PLL as the system clock and wait for it to become ready */
    RCC->CFGR_b.SW              = PLL_SRC;
    while(RCC->CFGR_b.SWS      != PLL_SRC);

#if defined (STM32F303)

    RCC->CFGR_b.PPRE1           = 4;      /* Set the APB1 prescaler to 2 */
    RCC->CFGR3_b.TIM1SW         = SET;    /* Set PLL output as TIMER1 clock source */
    RCC->CFGR3_b.TIM8SW         = SET;    /* Set PLL output as TIMER8 clock source */
    RCC->CFGR3_b.I2C1SW         = SET;    /* Set System clock as I2C1 clock source */
    RCC->CFGR3_b.I2C2SW         = SET;    /* Set System clock as I2C2 clock source */
    RCC->CFGR3_b.USART1SW       = SET;    /* Set System clock as USART1 clock source */
    RCC->CFGR3_b.USART2SW       = SET;    /* Set System clock as USART2 clock source */
    RCC->CFGR3_b.USART3SW       = SET;    /* Set System clock as USART3 clock source */
    RCC->CFGR3_b.UART4SW        = SET;    /* Set System clock as UART4 clock source */
    RCC->CFGR3_b.UART5SW        = SET;    /* Set System clock as UART5 clock source */
#elif defined (STM32G473)

    RCC->CFGR_b.PPRE1           = CLEAR;  /* Set the APB1 prescaler to 1 */
    RCC->CCIPR_b.FDCANSEL       = 2;      /* Set the APB clock as FDCAN clock source */
    RCC->CCIPR_b.ADC12SEL       = 2;      /* Set the system clock as ADC1,2 clock source */
    RCC->CCIPR_b.ADC345SEL      = 2;      /* Set the system clock as ADC3,4,5 clock source */
#endif
    hw_core::irq_enable(NonMaskableInt_IRQn);
}


#ifdef __cplusplus
}
#endif

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
