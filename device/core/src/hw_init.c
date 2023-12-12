/************************************************************************************* 
 * @file   hw_init.c
 * @date   Nov, 29 2023
 * @author Awatsa Hermann
 * @brief  Low-level hardware initialization functions.
 * ***********************************************************************************
 * @attention
 * 
 * The functions used in this file have been written mainly for the STM32F303
 * and STM32G473 MCUs. There is no guarantee of operation for other microcontrollers.
 * 
 #   DATE       |  Version  | revision   |
 -----------------------------------------
 # 2023.29.11   |    1      |  1         |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/


/**
 * \defgroup hardwareInit Hardware Initialization
 * \ingroup core
 * Provides low-level functions for initializing the MCU's hardware, including system
 * clocks, peripherals, and other essential settings. These functions will be executed
 * at the beginning of the application to set up the hardware environment.
 * 
 * @{
 */

/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "periph_def.h"



/*==================================================================================
|                                  DEFINES                                
===================================================================================*/
#define FORCE_INLINE    __attribute__((always_inline))

#if defined (STM32F303)

#define FLASH_LATENCY        2
#define PLL_SRC              2
#define PLL_MUL_9            7
#define SYSCLK_OUT           4
#define APB1_PRESC_2         4
#define CLK_FREQUENCY        72000000UL

#elif defined (STM32G473)

#define PLL_SRC              3 
#define LSE_SRC              1 
#define FLASH_LATENCY        4 
#define HSE_SRC              3 
#define PLLM_6               5 
#define PLLN_85              85
#define CLK_FREQUENCY        170000000UL

#endif

#define SET                  1
#define CLEAR                0



/*==================================================================================
|                                PRIVATES FUNCTIONS                                
===================================================================================*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \brief Configure the Flash memory latency
 */
inline static void setFlashLatency(void)
{
    /* Set the flash memory latency to 4 cycles */
    FLASH->ACR_b.LATENCY        = FLASH_LATENCY;

    /* Wait for the parameter to be take in account */
    while(FLASH->ACR_b.LATENCY != FLASH_LATENCY);
}

/**
 * \brief Enable the HSE oscillator
 */
inline static void enableHSEClk(void)
{
    /* Enable the HSE oscillator */
    RCC->CR_b.HSEON = SET;

    /* Wait for the oscillator to become stable */
    while(RCC->CR_b.HSERDY != SET);
}

/**
 * \brief Configure the RTC backup domain
 */
static void rtcConfig(void)
{
    /* Configure the RTC backup domain registers */
    RCC->APB1ENR_b.PWREN        = SET;
    PWR->CR_b.DBP               = SET;
    if (RCC->BDCR_b.RTCSEL     != SET)
    {
        RCC->BDCR_b.BDRST       = SET;
        RCC->BDCR_b.BDRST       = CLEAR;
    }
    RCC->BDCR_b.LSEDRV          = CLEAR;
}

/**
 * \brief Enable the LSE oscillator
 */
#if defined (STM32G473)
inline static void enableLSEClk(void)
{
    /* Enable the LSE oscillator */
    RCC->BDCR_b.LSEON = SET;

    /* wait for the LSE to become stable */
    while(RCC->BDCR_b.LSERDY != SET);
}
#endif  /* STM32G473 */

/**
 * \brief Configure the flash memory interface
 */
#if defined (STM32G473)
static void flashConfig(void)
{
    FLASH->ACR_b.DCEN   = SET;  /* enable the data cache lines */
    FLASH->ACR_b.ICEN   = SET;  /* enable the instruction cache lines */
    FLASH->ACR_b.PRFTEN = SET;  /* enable the prefetch buffer */
    PWR->CR5 &= ~(0x1 << 8U);   /* set the clock full speed */
}
#endif  /* STM32G473 */

/**
 * \brief Configure the phase locked loop
 */
static void pllConfig(void)
{
    #if defined (STM32G473)
    /* Configure the phase lock loop (PLL) for 170 Mhz clock operation */
    RCC->PLLCFGR_b.PLLSRC  = HSE_SRC;
    RCC->PLLCFGR_b.PLLM    = PLLM_6;
    RCC->PLLCFGR_b.PLLN    = PLLN_85;
    RCC->PLLCFGR_b.PLLR    = CLEAR;
    RCC->PLLCFGR_b.PLLREN  = SET;
    RCC->CFGR_b.MCOSEL     = SET;           // Sysclk as MCO output clock

    #elif defined (STM32F303)
    /* Configure the phase lock loop (PLL) for 72 Mhz clock operation */
    RCC->CFGR_b.PLLSRC     = PLL_SRC;
    RCC->CFGR_b.PLLXTPRE   = CLEAR;
    RCC->CFGR_b.PLLMUL     = PLL_MUL_9;
    RCC->CFGR_b.MCO        = SYSCLK_OUT;    // Sysclk as MCO output clock
    #endif

    /* Enable the PLL clock */
    RCC->CR_b.PLLON = SET;

    /* Wait for the pll to be locked */
    while(RCC->CR_b.PLLRDY != SET);
}

/**
 * \brief Set the buses, peripherals clock sources and prescalers
 */
static void setPrescalers(void)
{
    /* Set the AHB and APB2 buses clock prescalers */
    RCC->CFGR_b.HPRE            = CLEAR;
    RCC->CFGR_b.PPRE2           = CLEAR;

    #if defined (STM32F303)
    RCC->CFGR_b.PPRE1           = APB1_PRESC_2;  /* Set the APB1 prescaler to 2 */
    RCC->CFGR3_b.I2C1SW         = SET;           /* Set System clock as I2C1 clock source */
    RCC->CFGR3_b.I2C2SW         = SET;           /* Set System clock as I2C2 clock source */
    RCC->CFGR3_b.USART1SW       = SET;           /* Set System clock as USART1 clock source */
    RCC->CFGR3_b.USART2SW       = SET;           /* Set System clock as USART2 clock source */
    RCC->CFGR3_b.USART3SW       = SET;           /* Set System clock as USART3 clock source */
    RCC->CFGR3_b.UART4SW        = SET;           /* Set System clock as UART4 clock source */
    RCC->CFGR3_b.UART5SW        = SET;           /* Set System clock as UART5 clock source */
    #elif defined (STM32G473)
    RCC->CFGR_b.PPRE1           = CLEAR;         /* Set the APB1 prescaler to 1 */
    RCC->CCIPR_b.FDCANSEL       = 2;             /* Set the APB clock as FDCAN clock source */
    RCC->CCIPR_b.ADC12SEL       = 2;             /* Set the system clock as ADC1,2 clock source */
    RCC->CCIPR_b.ADC345SEL      = 2;             /* Set the system clock as ADC3,4,5 clock source */
    #endif
}

/**
 * \brief Enable the system clock
 */
inline static void enableSysClk(void)
{
    /* Set the pll clock as system clock source */
    RCC->CFGR_b.SW = PLL_SRC;

    /* Wait for the clock to become ready */
    while(RCC->CFGR_b.SWS != PLL_SRC);
}


/*==================================================================================
|                                PUBLICS FUNCTIONS                                
===================================================================================*/

/**
 * \brief Enable the faults exception handlers 
 */
FORCE_INLINE void enableFaultsHandlers(void)
{
    /* Enable the memory management faults,
       bus faults and usage faults handlers */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_USGFAULTENA_Msk ;
}

/**
 * \brief Initialize the floating point coprocessor 
 */
FORCE_INLINE void FPUInit(void)
{
    /* set Full Access to the FPU coprocessor */
    SCB->CPACR |= ((0x3U << 20U) | (0x3U << 22U));
}


/**
 * \brief Initialize the system clock
 */
void systemClockInit(void)
{
    /* Clear all the reset interrupts flags */
    RCC->CSR_b.RMVF = SET;

    setFlashLatency();  /* set the flash latency */

    enableHSEClk();     /* enable the HSE oscillator */

    rtcConfig();        /* configure the rtc backup domain */

    #if defined (STM32G473)

    enableLSEClk();     /* enable the LSE oscillator */

    /* Set the LSE as the RTC and backup registers clock */
    if (RCC->BDCR_b.RTCSEL!= LSE_SRC) {
        RCC->BDCR_b.RTCSEL = LSE_SRC;
    }

    flashConfig();

    #elif defined (STM32F303)

    /* Enable the flash memory prefetch buffer */
    FLASH->ACR_b.PRFTBE = SET;

    #endif

    pllConfig();        /* configure the phase lock loop */

    setPrescalers();    /* set the buses and peripherals prescalers */

    enableSysClk();     /* enable the system clock */

    /* Configure the systick for 1ms tick */
    SysTick_Config(CLK_FREQUENCY / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 5);
}


#ifdef __cplusplus
}
#endif

/** @} */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
