/*************************************************************************************
 * @file 	 peripherals_defs.h
 * @date   June, 30 2023
 * @author Awatsa Hermann
 * @brief	 This file comprises the STM32F303 & STM32G473
 *         peripherals access layer and registers bitfields
 * 
 * ***********************************************************************************
 * 
 * #   DATE       |  Version  | revision   |
 * -----------------------------------------
 * # 2023.06.30   |     1     |      2     |
 *
 * Smart Ebike Controller
 * https://github.com/Hermann-Core/smart-ebike-controller
 * 
 * @copyright Copyright (c) 2023 Hermann Awatsa
*************************************************************************************/

/* Prevent recursive inclusion */
#ifndef _PERIPHERALS_DEFS_H_
#define _PERIPHERALS_DEFS_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Cortex M4 core peripherals configuration */
#define __Vendor_SysTickConfig    0U
#define __CM4_REV                 0x0001U  /* Core revision r0p1 */
#define __MPU_PRESENT             1U       /* STM32F303 and STM32G473 devices provide an MPU */
#define __NVIC_PRIO_BITS          4U       /* 4 Bits for the Priority Levels */
#define __FPU_PRESENT             1U       /* STM32F303 and STM32G473 devices provide an FPU */


/*==================================================================================
|                            IRQ/EXCEPTIONS NUMBERS                                
===================================================================================*/

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line 19          */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line 20                     */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_TSC_IRQn              = 8,      /*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt         */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 Interrupt                                          */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 Interrupt                                          */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 Interrupt                                          */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 Interrupt                                          */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 Interrupt                                          */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 Interrupt                                          */
  ADC1_2_IRQn                 = 18,     /*!< ADC1 & ADC2 Interrupts                                            */
  USB_HP_CAN_TX_IRQn          = 19,     /*!< USB Device High Priority or CAN TX Interrupts                     */
  USB_LP_CAN_RX0_IRQn         = 20,     /*!< USB Device Low Priority or CAN RX0 Interrupts                     */
  #if defined (STM32F303)               /* STM32F303 specific interrupts numbers */
  CAN_RX1_IRQn                = 21,     /*!< CAN RX1 Interrupt                                                 */
  CAN_SCE_IRQn                = 22,     /*!< CAN SCE Interrupt                                                 */
  #elif defined (STM32G473)             /* STM32G473 specific interrupts numbers */
  FDCAN1_IT0_IRQn             = 21,     /*!< FDCAN1 IT0 Interrupt                                              */
  FDCAN1_IT1_IRQn             = 22,     /*!< FDCAN1 IT1 Interrupt                                              */
  #endif    /* STM32F303 */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)        */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)        */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)   */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)   */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)   */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB Wakeup Interrupt                                              */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                             */
  #if defined (STM32G473)               /* STM32G473 specific interrupts */
  FMC_IRQn                    = 48,     /*!< FMC global interrupt                                              */
  LPTIM1_IRQn                 = 49,     /*!< LPTIM1 global interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global interrupt                                             */
  #endif      /* STM32G473 */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt & EXTI Line34 Interrupt (UART4 wakeup)     */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt & EXTI Line35 Interrupt (UART5 wakeup)     */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&3 underrun error  interrupts                 */
  TIM7_DAC_IRQn               = 55,     /*!< TIM7 global and DAC2&4 underrun error  interrupts                 */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  ADC4_IRQn                   = 61,     /*!< ADC4  global Interrupt                                            */
  #if defined (STM32G473)               /* STM32G473 specific interrupts */
  ADC5_IRQn                   = 62,     /*!< ADC5 global Interrupt                                             */
  UCPD1_IRQn                  = 63,     /*!< UCPD global Interrupt                                             */
  #endif      /* STM32G473 */
  COMP1_2_3_IRQn              = 64,     /*!< COMP1, COMP2 and COMP3 global Interrupt via EXTI Line21, 22 and 29*/
  COMP4_5_6_IRQn              = 65,     /*!< COMP4, COMP5 and COMP6 global Interrupt via EXTI Line30, 31 and 32*/
  COMP7_IRQn                  = 66,     /*!< COMP7 global Interrupt via EXTI Line33                            */
  #if defined (STM32F303)               /* STM32F303 specific interrupts numbers */
  USB_HP_IRQn                 = 74,     /*!< USB High Priority global Interrupt                                */
  USB_LP_IRQn                 = 75,     /*!< USB Low Priority global Interrupt                                 */
  USBWakeUp_RMP_IRQn          = 76,     /*!< USB Wakeup Interrupt remap                                        */
  FPU_IRQn                    = 81,     /*!< Floating point Interrupt                                          */
  #elif defined (STM32G473)             /* STM32G473 specific interrupts numbers */
  CRS_IRQn                    = 75,     /*!< CRS global interrupt                                              */
  SAI1_IRQn                   = 76,     /*!< Serial Audio Interface global interrupt                           */
  TIM20_BRK_IRQn              = 77,     /*!< TIM20 Break, Transition error and Index error Interrupt           */
  TIM20_UP_IRQn               = 78,     /*!< TIM20 Update interrupt                                            */
  TIM20_TRG_COM_IRQn          = 79,     /*!< TIM20 Trigger, Commutation, Direction change and Index Interrupt  */
  TIM20_CC_IRQn               = 80,     /*!< TIM20 Capture Compare interrupt                                   */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  I2C4_EV_IRQn                = 82,     /*!< I2C4 Event interrupt                                              */
  I2C4_ER_IRQn                = 83,     /*!< I2C4 Error interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 Event interrupt                                              */
  FDCAN2_IT0_IRQn             = 86,     /*!< FDCAN2 interrupt line 0 interrupt                                 */
  FDCAN2_IT1_IRQn             = 87,     /*!< FDCAN2 interrupt line 1 interrupt                                 */
  FDCAN3_IT0_IRQn             = 88,     /*!< FDCAN3 interrupt line 0 interrupt                                 */
  FDCAN3_IT1_IRQn             = 89,     /*!< FDCAN3 interrupt line 1 interrupt                                 */
  RNG_IRQn                    = 90,     /*!< RNG global interrupt                                              */
  LPUART1_IRQn                = 91,     /*!< LP UART 1 Interrupt                                               */
  I2C3_EV_IRQn                = 92,     /*!< I2C3 Event Interrupt                                              */
  I2C3_ER_IRQn                = 93,     /*!< I2C3 Error interrupt                                              */
  DMAMUX_OVR_IRQn             = 94,     /*!< DMAMUX overrun global interrupt                                   */
  QUADSPI_IRQn                = 95,     /*!< QUADSPI interrupt                                                 */
  DMA1_Channel8_IRQn          = 96,     /*!< DMA1 Channel 8 interrupt                                          */
  DMA2_Channel6_IRQn          = 97,     /*!< DMA2 Channel 6 interrupt                                          */
  DMA2_Channel7_IRQn          = 98,     /*!< DMA2 Channel 7 interrupt                                          */
  DMA2_Channel8_IRQn          = 99,     /*!< DMA2 Channel 8 interrupt                                          */
  CORDIC_IRQn                 = 100,    /*!< CORDIC global Interrupt                                           */
  FMAC_IRQn                   = 101     /*!< FMAC global Interrupt                                             */
  #endif    /* STM32F303 */
  
} IRQn_t;


/*==================================================================================
|                                 INCLUDES                                
===================================================================================*/
#include "common_types.h"
#include "core_cm4.h"



/*==================================================================================
|                      PERIPHERALS REGISTERS STRUCTURES                                
===================================================================================*/

/*==================== ANALOG TO DIGITAL CONVERTER ================================*/

typedef struct {                     
  
  union {
    __IOM u32 ISR;              
    
    struct {
      __IOM u32 ADRDY      : 1; 
      __IOM u32 EOSMP      : 1; 
      __IOM u32 EOC        : 1; 
      __IOM u32 EOS        : 1; 
      __IOM u32 OVR        : 1; 
      __IOM u32 JEOC       : 1; 
      __IOM u32 JEOS       : 1; 
      __IOM u32 AWD1       : 1; 
      __IOM u32 AWD2       : 1; 
      __IOM u32 AWD3       : 1; 
      __IOM u32 JQOVF      : 1; 
            u32            : 21;
    } ISR_b;
  } ;
  
  union {
    __IOM u32 IER;              
    
    struct {
      __IOM u32 ADRDYIE    : 1; 
      __IOM u32 EOSMPIE    : 1; 
      __IOM u32 EOCIE      : 1; 
      __IOM u32 EOSIE      : 1; 
      __IOM u32 OVRIE      : 1; 
      __IOM u32 JEOCIE     : 1; 
      __IOM u32 JEOSIE     : 1; 
      __IOM u32 AWD1IE     : 1; 
      __IOM u32 AWD2IE     : 1; 
      __IOM u32 AWD3IE     : 1; 
      __IOM u32 JQOVFIE    : 1; 
            u32            : 21;
    } IER_b;
  } ;
  
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 ADEN       : 1; 
      __IOM u32 ADDIS      : 1; 
      __IOM u32 ADSTART    : 1; 
      __IOM u32 JADSTART   : 1; 
      __IOM u32 ADSTP      : 1; 
      __IOM u32 JADSTP     : 1; 
            u32            : 22;
      __IOM u32 ADVREGEN   : 1; 
      __IOM u32 DEEPPWD    : 1; 
      __IOM u32 ADCALDIF   : 1; 
      __IOM u32 ADCAL      : 1; 
    } CR_b;
  } ;
  
  union {
    __IOM u32 CFGR;             
    
    struct {
      __IOM u32 DMAEN      : 1; 
      __IOM u32 DMACFG     : 1; 
            u32            : 1;
      __IOM u32 RES        : 2; 
      __IOM u32 EXTSEL     : 5; 
      __IOM u32 EXTEN      : 2; 
      __IOM u32 OVRMOD     : 1; 
      __IOM u32 CONT       : 1; 
      __IOM u32 AUTDLY     : 1; 
      __IOM u32 ALIGN      : 1; 
      __IOM u32 DISCEN     : 1; 
      __IOM u32 DISCNUM    : 3; 
      __IOM u32 JDISCEN    : 1; 
      __IOM u32 JQM        : 1; 
      __IOM u32 AWD1SGL    : 1; 
      __IOM u32 AWD1EN     : 1; 
      __IOM u32 JAWD1EN    : 1; 
      __IOM u32 JAUTO      : 1; 
      __IOM u32 AWD1CH     : 5; 
      __IOM u32 JQDIS      : 1; 
    } CFGR_b;
  } ;
  
  union {
    __IOM u32 CFGR2;            
    
    struct {
      __IOM u32 ROVSE      : 1; 
      __IOM u32 JOVSE      : 1; 
      __IOM u32 OVSR       : 3; 
      __IOM u32 OVSS       : 4; 
      __IOM u32 TROVS      : 1; 
      __IOM u32 ROVSM      : 1; 
            u32            : 5;
      __IOM u32 GCOMP      : 1; 
            u32            : 8;
      __IOM u32 SWTRIG     : 1; 
      __IOM u32 BULB       : 1; 
      __IOM u32 SMPTRIG    : 1; 
            u32            : 4;
    } CFGR2_b;
  } ;
  
  union {
    __IOM u32 SMPR1;            
    
    struct {
      __IOM u32 SMP0       : 3; 
      __IOM u32 SMP1       : 3; 
      __IOM u32 SMP2       : 3; 
      __IOM u32 SMP3       : 3; 
      __IOM u32 SMP4       : 3; 
      __IOM u32 SMP5       : 3; 
      __IOM u32 SMP6       : 3; 
      __IOM u32 SMP7       : 3; 
      __IOM u32 SMP8       : 3; 
      __IOM u32 SMP9       : 3; 
            u32            : 1;
      __IOM u32 SMPPLUS    : 1; 
    } SMPR1_b;
  } ;
  
  union {
    __IOM u32 SMPR2;            
    
    struct {
      __IOM u32 SMP10      : 3; 
      __IOM u32 SMP11      : 3; 
      __IOM u32 SMP12      : 3; 
      __IOM u32 SMP13      : 3; 
      __IOM u32 SMP14      : 3; 
      __IOM u32 SMP15      : 3; 
      __IOM u32 SMP16      : 3; 
      __IOM u32 SMP17      : 3; 
      __IOM u32 SMP18      : 3; 
            u32            : 5;
    } SMPR2_b;
  } ;
  __IM  u32  RESERVED;
  
  union {
    __IOM u32 TR1;              
    
    struct {
      __IOM u32 LT1        : 12;
      __IOM u32 AWDFILT    : 3; 
            u32            : 1;
      __IOM u32 HT1        : 12;
            u32            : 4;
    } TR1_b;
  } ;
  
  union {
    __IOM u32 TR2;              
    
    struct {
      __IOM u32 LT2        : 8; 
            u32            : 8;
      __IOM u32 HT2        : 8; 
            u32            : 8;
    } TR2_b;
  } ;
  
  union {
    __IOM u32 TR3;              
    
    struct {
      __IOM u32 LT3        : 8; 
            u32            : 8;
      __IOM u32 HT3        : 8; 
            u32            : 8;
    } TR3_b;
  } ;
  __IM  u32  RESERVED1;
  
  union {
    __IOM u32 SQR1;             
    
    struct {
      __IOM u32 L          : 4; 
            u32            : 2;
      __IOM u32 SQ1        : 5; 
            u32            : 1;
      __IOM u32 SQ2        : 5; 
            u32            : 1;
      __IOM u32 SQ3        : 5; 
            u32            : 1;
      __IOM u32 SQ4        : 5; 
            u32            : 3;
    } SQR1_b;
  } ;
  
  union {
    __IOM u32 SQR2;             
    
    struct {
      __IOM u32 SQ5        : 5; 
            u32            : 1;
      __IOM u32 SQ6        : 5; 
            u32            : 1;
      __IOM u32 SQ7        : 5; 
            u32            : 1;
      __IOM u32 SQ8        : 5; 
            u32            : 1;
      __IOM u32 SQ9        : 5; 
            u32            : 3;
    } SQR2_b;
  } ;
  
  union {
    __IOM u32 SQR3;             
    
    struct {
      __IOM u32 SQ10       : 5; 
            u32            : 1;
      __IOM u32 SQ11       : 5; 
            u32            : 1;
      __IOM u32 SQ12       : 5; 
            u32            : 1;
      __IOM u32 SQ13       : 5; 
            u32            : 1;
      __IOM u32 SQ14       : 5; 
            u32            : 3;
    } SQR3_b;
  } ;
  
  union {
    __IOM u32 SQR4;             
    
    struct {
      __IOM u32 SQ15       : 5; 
            u32            : 1;
      __IOM u32 SQ16       : 5; 
            u32            : 21;
    } SQR4_b;
  } ;
  
  union {
    __IM  u32 DR;               
    
    struct {
      __IM  u32 RDATA      : 16;
            u32            : 16;
    } DR_b;
  } ;
  __IM  u32  RESERVED2[2];
  
  union {
    __IOM u32 JSQR;             
    
    struct {
      __IOM u32 JL         : 2; 
      __IOM u32 JEXTSEL    : 5; 
      __IOM u32 JEXTEN     : 2; 
      __IOM u32 JSQ1       : 5; 
            u32            : 1;
      __IOM u32 JSQ2       : 5; 
            u32            : 1;
      __IOM u32 JSQ3       : 5; 
            u32            : 1;
      __IOM u32 JSQ4       : 5; 
    } JSQR_b;
  } ;
  __IM  u32  RESERVED3[4];
  
  union {
    __IOM u32 OFR1;             
    
    struct {
      __IOM u32 OFFSET1    : 12;
            u32            : 12;
      __IOM u32 OFFSETPOS  : 1; 
      __IOM u32 SATEN      : 1; 
      __IOM u32 OFFSET1_CH : 5; 
      __IOM u32 OFFSET1_EN : 1; 
    } OFR1_b;
  } ;
  
  union {
    __IOM u32 OFR2;             
    
    struct {
      __IOM u32 OFFSET1    : 12;
            u32            : 12;
      __IOM u32 OFFSETPOS  : 1; 
      __IOM u32 SATEN      : 1; 
      __IOM u32 OFFSET1_CH : 5; 
      __IOM u32 OFFSET1_EN : 1; 
    } OFR2_b;
  } ;
  
  union {
    __IOM u32 OFR3;             
    
    struct {
      __IOM u32 OFFSET1    : 12;
            u32            : 12;
      __IOM u32 OFFSETPOS  : 1; 
      __IOM u32 SATEN      : 1; 
      __IOM u32 OFFSET1_CH : 5; 
      __IOM u32 OFFSET1_EN : 1; 
    } OFR3_b;
  } ;
  
  union {
    __IOM u32 OFR4;             
    
    struct {
      __IOM u32 OFFSET1    : 12;
            u32            : 12;
      __IOM u32 OFFSETPOS  : 1; 
      __IOM u32 SATEN      : 1; 
      __IOM u32 OFFSET1_CH : 5; 
      __IOM u32 OFFSET1_EN : 1; 
    } OFR4_b;
  } ;
  __IM  u32  RESERVED4[4];
  
  union {
    __IM  u32 JDR1;             
    
    struct {
      __IM  u32 JDATA1     : 16;
            u32            : 16;
    } JDR1_b;
  } ;
  
  union {
    __IM  u32 JDR2;             
    
    struct {
      __IM  u32 JDATA2     : 16;
            u32            : 16;
    } JDR2_b;
  } ;
  
  union {
    __IM  u32 JDR3;             
    
    struct {
      __IM  u32 JDATA3     : 16;
            u32            : 16;
    } JDR3_b;
  } ;
  
  union {
    __IM  u32 JDR4;             
    
    struct {
      __IM  u32 JDATA4     : 16;
            u32            : 16;
    } JDR4_b;
  } ;
  __IM  u32  RESERVED5[4];
  
  union {
    __IOM u32 AWD2CR;           
    
    struct {
      __IOM u32 AWD2CH     : 19;
            u32            : 13;
    } AWD2CR_b;
  } ;
  
  union {
    __IOM u32 AWD3CR;           
    
    struct {
      __IOM u32 AWD3CH     : 19;
            u32            : 13;
    } AWD3CR_b;
  } ;
  __IM  u32  RESERVED6[2];
  
  union {
    __IOM u32 DIFSEL;           
    
    struct {
      __IM  u32 DIFSEL_0   : 1; 
      __IOM u32 DIFSEL_1_18: 18;
            u32            : 13;
    } DIFSEL_b;
  } ;
  
  union {
    __IOM u32 CALFACT;          
    
    struct {
      __IOM u32 CALFACT_S  : 7; 
            u32            : 9;
      __IOM u32 CALFACT_D  : 7; 
            u32            : 9;
    } CALFACT_b;
  } ;
  __IM  u32  RESERVED7[2];
  
  union {
    __IOM u32 GCOMP;            
    
    struct {
      __IOM u32 GCOMPCOEFF : 14;
            u32            : 18;
    } GCOMP_b;
  } ;

} ADC_HW;


typedef struct
{
  __IO u32 CSR;      
  u32      RESERVED1;
  __IO u32 CCR;      
  __IO u32 CDR; 

} ADC_Common_HW;


/*===================== FD CONTROLLER AREA NETWORK =================================*/

typedef struct {                     
  
  union {
    __IM  u32 CREL;             
    
    struct {
      __IM  u32 DAY        : 8; 
      __IM  u32 MON        : 8; 
      __IM  u32 YEAR       : 4; 
      __IM  u32 SUBSTEP    : 4; 
      __IM  u32 STEP       : 4; 
      __IM  u32 REL        : 4; 
    } CREL_b;
  } ;
  
  union {
    __IM  u32 ENDN;             
    
    struct {
      __IM  u32 ETV        : 32;
    } ENDN_b;
  } ;
  __IM  u32  RESERVED;
  
  union {
    __IOM u32 DBTP; 

    struct {
      __IOM u32 DSJW       : 4; 
      __IOM u32 DTSEG2     : 4; 
      __IOM u32 DTSEG1     : 5; 
            u32            : 3;
      __IOM u32 DBRP       : 5; 
            u32            : 2;
      __IOM u32 TDC        : 1; 
            u32            : 8;
    } DBTP_b;
  } ;
  
  union {
    __IOM u32 TEST; 

    struct {
            u32            : 4;
      __IOM u32 LBCK       : 1; 
      __IOM u32 TX         : 2; 
      __IM  u32 RX         : 1; 
            u32            : 24;
    } TEST_b;
  } ;
  
  union {
    __IOM u32 RWD; 

    struct {
      __IOM u32 WDC        : 8; 
      __IM  u32 WDV        : 8; 
            u32            : 16;
    } RWD_b;
  } ;
  
  union {
    __IOM u32 CCCR; 
    
    struct {
      __IOM u32 INIT       : 1; 
      __IOM u32 CCE        : 1; 
      __IOM u32 ASM        : 1; 
      __IM  u32 CSA        : 1; 
      __IOM u32 CSR        : 1; 
      __IOM u32 MON        : 1; 
      __IOM u32 DAR        : 1; 
      __IOM u32 TEST       : 1; 
      __IOM u32 FDOE       : 1; 
      __IOM u32 BRSE       : 1; 
            u32            : 2;
      __IOM u32 PXHD       : 1; 
      __IOM u32 EFBI       : 1; 
      __IOM u32 TXP        : 1; 
      __IOM u32 NISO       : 1; 
            u32            : 16;
    } CCCR_b;
  } ;
  
  union {
    __IOM u32 NBTP;             
    
    struct {
      __IOM u32 NTSEG2     : 7; 
            u32            : 1;
      __IOM u32 NTSEG1     : 8; 
      __IOM u32 NBRP       : 9; 
      __IOM u32 NSJW       : 7; 
    } NBTP_b;
  } ;
  
  union {
    __IOM u32 TSCC;             
    
    struct {
      __IOM u32 TSS        : 2; 
            u32            : 14;
      __IOM u32 TCP        : 4; 
            u32            : 12;
    } TSCC_b;
  } ;
  
  union {
    __IM  u32 TSCV;             
    
    struct {
      __IM  u32 TSC        : 16;
            u32            : 16;
    } TSCV_b;
  } ;
  
  union {
    __IOM u32 TOCC;             
    
    struct {
      __IOM u32 ETOC       : 1; 
      __IOM u32 TOS        : 2; 
            u32            : 13;
      __IOM u32 TOP        : 16;
    } TOCC_b;
  } ;
  
  union {
    __IM  u32 TOCV;             
    
    struct {
      __IM  u32 TOC        : 16;
            u32            : 16;
    } TOCV_b;
  } ;
  __IM  u32  RESERVED1[4];
  
  union {
    __IM  u32 ECR;              
    
    struct {
      __IM  u32 TEC        : 8; 
      __IM  u32 REC        : 7; 
      __IM  u32 RP         : 1; 
      __IM  u32 CEL        : 8; 
            u32            : 8;
    } ECR_b;
  } ;
  
  union {
    __IOM u32 PSR;              
    
    struct {
      __IOM u32 LEC        : 3; 
      __IM  u32 ACT        : 2; 
      __IM  u32 EP         : 1; 
      __IM  u32 EW         : 1; 
      __IM  u32 BO         : 1; 
      __IOM u32 DLEC       : 3; 
      __IOM u32 RESI       : 1; 
      __IOM u32 RBRS       : 1; 
      __IOM u32 REDL       : 1; 
      __IOM u32 PXE        : 1; 
            u32            : 1;
      __IOM u32 TDCV       : 7; 
            u32            : 9;
    } PSR_b;
  } ;
  
  union {
    __IOM u32 TDCR;             
    
    struct {
      __IOM u32 TDCF       : 7; 
            u32            : 1;
      __IOM u32 TDCO       : 7; 
            u32            : 17;
    } TDCR_b;
  } ;
  __IM  u32  RESERVED2;
  
  union {
    __IOM u32 IR; 

    struct {
      __IOM u32 RF0N       : 1; 
      __IOM u32 RF0F       : 1; 
      __IOM u32 RF0L       : 1; 
      __IOM u32 RF1N       : 1; 
      __IOM u32 RF1F       : 1; 
      __IOM u32 RF1L       : 1; 
      __IOM u32 HPM        : 1; 
      __IOM u32 TC         : 1; 
      __IOM u32 TCF        : 1; 
      __IOM u32 TFE        : 1; 
      __IOM u32 TEFN       : 1; 
      __IOM u32 TEFF       : 1; 
      __IOM u32 TEFL       : 1; 
      __IOM u32 TSW        : 1; 
      __IOM u32 MRAF       : 1; 
      __IOM u32 TOO        : 1; 
      __IOM u32 ELO        : 1; 
      __IOM u32 EP         : 1; 
      __IOM u32 EW         : 1; 
      __IOM u32 BO         : 1; 
      __IOM u32 WDI        : 1; 
      __IOM u32 PEA        : 1; 
      __IOM u32 PED        : 1; 
      __IOM u32 ARA        : 1; 
            u32            : 8;
    } IR_b;
  } ;
  
  union {
    __IOM u32 IE;  

    struct {
      __IOM u32 RF0NE      : 1; 
      __IOM u32 RF0FE      : 1; 
      __IOM u32 RF0LE      : 1; 
      __IOM u32 RF1NE      : 1; 
      __IOM u32 RF1FE      : 1; 
      __IOM u32 RF1LE      : 1; 
      __IOM u32 HPME       : 1; 
      __IOM u32 TCE        : 1; 
      __IOM u32 TCFE       : 1; 
      __IOM u32 TFEE       : 1; 
      __IOM u32 TEFNE      : 1; 
      __IOM u32 TEFFE      : 1; 
      __IOM u32 TEFLE      : 1; 
      __IOM u32 TSWE       : 1; 
      __IOM u32 MRAFE      : 1; 
      __IOM u32 TOOE       : 1; 
      __IOM u32 ELOE       : 1; 
      __IOM u32 EPE        : 1; 
      __IOM u32 EWE        : 1; 
      __IOM u32 BOE        : 1; 
      __IOM u32 WDIE       : 1; 
      __IOM u32 PEAE       : 1; 
      __IOM u32 PEDE       : 1; 
      __IOM u32 ARAE       : 1; 
            u32            : 8;
    } IE_b;
  } ;
  
  union {
    __IOM u32 ILS; 

    struct {
      __IOM u32 RxFIFO0    : 1; 
      __IOM u32 RxFIFO1    : 1; 
      __IOM u32 SMSG       : 1; 
      __IOM u32 TFERR      : 1; 
      __IOM u32 MISC       : 1; 
      __IOM u32 BERR       : 1; 
      __IOM u32 PERR       : 1; 
            u32            : 25;
    } ILS_b;
  } ;
  
  union {
    __IOM u32 ILE;

    struct {
      __IOM u32 EINT0      : 1; 
      __IOM u32 EINT1      : 1; 
            u32            : 30;
    } ILE_b;
  } ;
  __IM  u32  RESERVED3[8];
  
  union {
    __IOM u32 RXGFC; 
    
    struct {
      __IOM u32 RRFE       : 1; 
      __IOM u32 RRFS       : 1; 
      __IOM u32 ANFE       : 2; 
      __IOM u32 ANFS       : 2; 
            u32            : 2;
      __IOM u32 F1OM       : 1; 
      __IOM u32 F0OM       : 1; 
            u32            : 6;
      __IOM u32 LSS        : 5; 
            u32            : 3;
      __IOM u32 LSE        : 4; 
            u32            : 4;
    } RXGFC_b;
  } ;
  
  union {
    __IOM u32 XIDAM;            
    
    struct {
      __IOM u32 EIDM       : 29;
            u32            : 3;
    } XIDAM_b;
  } ;
  
  union {
    __IM  u32 HPMS;
    
    struct {
      __IM  u32 BIDX       : 3; 
            u32            : 3;
      __IM  u32 MSI        : 2; 
      __IM  u32 FIDX       : 5; 
            u32            : 2;
      __IM  u32 FLST       : 1; 
            u32            : 16;
    } HPMS_b;
  } ;
  __IM  u32  RESERVED4;
  
  union {
    __IM  u32 RXF0S;            
    
    struct {
      __IM  u32 F0FL       : 4; 
            u32            : 4;
      __IM  u32 F0GI       : 2; 
            u32            : 6;
      __IM  u32 F0PI       : 2; 
            u32            : 6;
      __IM  u32 F0F        : 1; 
      __IM  u32 RF0L       : 1; 
            u32            : 6;
    } RXF0S_b;
  } ;
  
  union {
    __IOM u32 RXF0A;            
    
    struct {
      __IOM u32 F0AI       : 3; 
            u32            : 29;
    } RXF0A_b;
  } ;
  
  union {
    __IM  u32 RXF1S;            
    
    struct {
      __IM  u32 F1FL       : 4; 
            u32            : 4;
      __IM  u32 F1GI       : 2; 
            u32            : 6;
      __IM  u32 F1PI       : 2; 
            u32            : 6;
      __IM  u32 F1F        : 1; 
      __IM  u32 RF1L       : 1; 
            u32            : 6;
    } RXF1S_b;
  } ;
  
  union {
    __IOM u32 RXF1A;            
    
    struct {
      __IOM u32 F1AI       : 3; 
            u32            : 29;
    } RXF1A_b;
  } ;
  __IM  u32  RESERVED5[8];
  
  union {
    __IOM u32 TXBC;             
    
    struct {
            u32            : 24;
      __IOM u32 TFQM       : 1; 
            u32            : 7;
    } TXBC_b;
  } ;
  
  union {
    __IM  u32 TXFQS;
    
    struct {
      __IM  u32 TFFL       : 3; 
            u32            : 5;
      __IM  u32 TFGI       : 2; 
            u32            : 6;
      __IM  u32 TFQPI      : 2; 
            u32            : 3;
      __IM  u32 TFQF       : 1; 
            u32            : 10;
    } TXFQS_b;
  } ;
  
  union {
    __IM  u32 TXBRP;            
    
    struct {
      __IM  u32 TRP        : 3; 
            u32            : 29;
    } TXBRP_b;
  } ;
  
  union {
    __IOM u32 TXBAR;            
    
    struct {
      __IOM u32 AR         : 3; 
            u32            : 29;
    } TXBAR_b;
  } ;
  
  union {
    __IOM u32 TXBCR;            
    
    struct {
      __IOM u32 CR         : 3; 
            u32            : 29;
    } TXBCR_b;
  } ;
  
  union {
    __IM  u32 TXBTO;            
    
    struct {
      __IM  u32 TO         : 3; 
            u32            : 29;
    } TXBTO_b;
  } ;
  
  union {
    __IM  u32 TXBCF;            
    
    struct {
      __IM  u32 CF         : 3; 
            u32            : 29;
    } TXBCF_b;
  } ;
  
  union {
    __IOM u32 TXBTIE;
      
    
    struct {
      __IOM u32 TIE        : 3; 
            u32            : 29;
    } TXBTIE_b;
  } ;
  
  union {
    __IOM u32 TXBCIE; 
      
    
    struct {
      __IOM u32 CFIE       : 3; 
            u32            : 29;
    } TXBCIE_b;
  } ;
  
  union {
    __IM  u32 TXEFS;            
    
    struct {
      __IM  u32 EFFL       : 3; 
            u32            : 5;
      __IM  u32 EFGI       : 2; 
            u32            : 6;
      __IM  u32 EFPI       : 2; 
            u32            : 6;
      __IM  u32 EFF        : 1; 
      __IM  u32 TEFL       : 1; 
            u32            : 6;
    } TXEFS_b;
  } ;
  
  union {
    __IOM u32 TXEFA;            
    
    struct {
      __IOM u32 EFAI       : 2; 
            u32            : 30;
    } TXEFA_b;
  } ;
  __IM  u32  RESERVED6[5];
  
  union {
    __IOM u32 CKDIV;            
    
    struct {
      __IOM u32 PDIV       : 4; 
                      
            u32            : 28;
    } CKDIV_b;
  } ;

} FDCAN_HW;


/*===================== COMPARATOR AND STATUS REGISTER =============================*/

typedef struct {                    
  
  union {
    __IOM u32 CSR;      
    
    struct {
      __IOM u32 EN         : 1;
            u32            : 3;
      __IOM u32 INMSEL     : 3;
            u32            : 1;
      __IOM u32 INPSEL     : 1;
            u32            : 6;
      __IOM u32 POL        : 1;
      __IOM u32 HYST       : 3;
      __IOM u32 BLANKSEL   : 3;
      __IOM u32 BRGEN      : 1;
      __IOM u32 SCALEN     : 1;
            u32            : 6;
      __IM  u32 VALUE      : 1;
      __IOM u32 LOCK       : 1;
    } CSR_b;
  } ;

} COMP_HW;


/*================== CYCLIC REDUNDANCY CHECK CALCULATION UNIT ======================*/

typedef struct {                     
  
  __IOM u32 DR;
  
  __IOM u32 IDR;
  
  union {
    __IOM u32 CR;               
    
    struct {
      __OM  u32 RESET      : 1; 
            u32            : 2;
      __IOM u32 POLYSIZE   : 2; 
      __IOM u32 REV_IN     : 2; 
      __IOM u32 REV_OUT    : 1; 
            u32            : 24;
    } CR_b;
  } ;
  __IM  u32  RESERVED;

  __IOM u32 INIT;
 
  __IOM u32 POL;

} CRC_HW;


/*========================== CLOCK RECOVERY SYSTEM =================================*/

typedef struct {
  
  union {
    __IOM u32 CR;
    
    struct {
      __IOM u32 SYNCOKIE   : 1;
      __IOM u32 SYNCWARNIE : 1; 
      __IOM u32 ERRIE      : 1;
      __IOM u32 ESYNCIE    : 1;
            u32            : 1;
      __IOM u32 CEN        : 1;         
                                                              
      __IOM u32 AUTOTRIMEN : 1;                                                   
      __IOM u32 SWSYNC     : 1;                                            
      __IOM u32 TRIM       : 7;                                         
            u32            : 17;
    } CR_b;
  } ;
  
  union {
    __IOM u32 CFGR;                   
  
    struct {
      __IOM u32 RELOAD     : 16;                                            
      __IOM u32 FELIM      : 8;                                            
      __IOM u32 SYNCDIV    : 3;
                                                    
            u32            : 1;
      __IOM u32 SYNCSRC    : 2;                                              
            u32            : 1;
      __IOM u32 SYNCPOL    : 1;
                                                    
    } CFGR_b;
  } ;
  
  union {
    __IM  u32 ISR;
    
    struct {
      __IM  u32 SYNCOKF    : 1;                                                    
      __IM  u32 SYNCWARNF  : 1;                                                    
      __IM  u32 ERRF       : 1;                                                    
      __IM  u32 ESYNCF     : 1;                                                    
            u32            : 4;
      __IM  u32 SYNCERR    : 1;                                                    
      __IM  u32 SYNCMISS   : 1;                                                    
      __IM  u32 TRIMOVF    : 1;                                                    
            u32            : 4;
      __IM  u32 FEDIR      : 1;                                                    
      __IM  u32 FECAP      : 16;
                                                    
    } ISR_b;
  } ;
  
  union {
    __IOM u32 ICR;
    
    struct {
      __IOM u32 SYNCOKC    : 1;                                                    
      __IOM u32 SYNCWARNC  : 1;                                                     
      __IOM u32 ERRC       : 1;                                                    
      __IOM u32 ESYNCC     : 1;                                                    
            u32            : 28;
    } ICR_b;
  } ;

} CRS_HW;


/*======================= DIGITAL TO ANALOG CONVERTER ==============================*/

typedef struct {                        
  
  union {
    __IOM u32 DAC_CR;              
    
    struct {
      __IOM u32 EN1        : 1;
                            
      __IOM u32 TEN1       : 1;    
      __IOM u32 TSEL1      : 4;
                            
      __IOM u32 WAVE1      : 2;
                            
      __IOM u32 MAMP1      : 4;
                            
      __IOM u32 DMAEN1     : 1;
                            
      __IOM u32 DMAUDRIE1  : 1;
                            
      __IOM u32 CEN1       : 1;
                            
            u32            : 1;
      __IOM u32 EN2        : 1;
                            
      __IOM u32 TEN2       : 1;    
      __IOM u32 TSEL2      : 4;
                            
      __IOM u32 WAVE2      : 2;
                            
      __IOM u32 MAMP2      : 4;
                            
      __IOM u32 DMAEN2     : 1;
                            
      __IOM u32 DMAUDRIE2  : 1;
                            
      __IOM u32 CEN2       : 1;
                            
            u32            : 1;
    } DAC_CR_b;
  } ;
  
  union {
    __OM  u32 DAC_SWTRGR;          
    
    struct {
      __OM  u32 SWTRIG1    : 1;
                            
      __OM  u32 SWTRIG2    : 1;
                            
            u32            : 14;
      __OM  u32 SWTRIGB1   : 1;    
      __OM  u32 SWTRIGB2   : 1;    
            u32            : 14;
    } DAC_SWTRGR_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12R1;
            
    
    struct {
      __IOM u32 DACC1DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC1DHRB  : 12;   
            u32            : 4;
    } DAC_DHR12R1_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12L1;
            
    
    struct {
            u32            : 4;
      __IOM u32 DACC1DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC1DHRB  : 12;   
    } DAC_DHR12L1_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR8R1;
            
    
    struct {
      __IOM u32 DACC1DHR   : 8;
                            
      __IOM u32 DACC1DHRB  : 8;    
            u32            : 16;
    } DAC_DHR8R1_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12R2;
            
    
    struct {
      __IOM u32 DACC2DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC2DHRB  : 12;   
            u32            : 4;
    } DAC_DHR12R2_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12L2;
            
    
    struct {
            u32            : 4;
      __IOM u32 DACC2DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC2DHRB  : 12;   
    } DAC_DHR12L2_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR8R2;
            
    
    struct {
      __IOM u32 DACC2DHR   : 8;
                            
      __IOM u32 DACC2DHRB  : 8;    
            u32            : 16;
    } DAC_DHR8R2_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12RD;         
    
    struct {
      __IOM u32 DACC1DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC2DHR   : 12;
                            
            u32            : 4;
    } DAC_DHR12RD_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR12LD;         
    
    struct {
            u32            : 4;
      __IOM u32 DACC1DHR   : 12;
                            
            u32            : 4;
      __IOM u32 DACC2DHR   : 12;
                            
    } DAC_DHR12LD_b;
  } ;
  
  union {
    __IOM u32 DAC_DHR8RD;          
    
    struct {
      __IOM u32 DACC1DHR   : 8;
                            
      __IOM u32 DACC2DHR   : 8;
                            
            u32            : 16;
    } DAC_DHR8RD_b;
  } ;
  
  union {
    __IM  u32 DAC_DOR1;            
    
    struct {
      __IM  u32 DACC1DOR   : 12;
                            
            u32            : 4;
      __IM  u32 DACC1DORB  : 12;   
            u32            : 4;
    } DAC_DOR1_b;
  } ;
  
  union {
    __IM  u32 DAC_DOR2;            
    
    struct {
      __IM  u32 DACC2DOR   : 12;
                            
            u32            : 4;
      __IM  u32 DACC2DORB  : 12;   
            u32            : 4;
    } DAC_DOR2_b;
  } ;
  
  union {
    __IOM u32 DAC_SR;              
    
    struct {
            u32            : 11;
      __IOM u32 DAC1RDY    : 1;    
      __IOM u32 DORSTAT1   : 1;    
      __IOM u32 DMAUDR1    : 1;
                            
      __IM  u32 CAL_FLAG1  : 1;
                            
      __IM  u32 BWST1      : 1;
                            
            u32            : 11;
      __IOM u32 DAC2RDY    : 1;    
      __IOM u32 DORSTAT2   : 1;    
      __IOM u32 DMAUDR2    : 1;
                            
      __IM  u32 CAL_FLAG2  : 1;
                            
      __IM  u32 BWST2      : 1;
                            
    } DAC_SR_b;
  } ;
  
  union {
    __IOM u32 DAC_CCR;             
    
    struct {
      __IOM u32 OTRIM1     : 5;    
            u32            : 11;
      __IOM u32 OTRIM2     : 5;    
            u32            : 11;
    } DAC_CCR_b;
  } ;
  
  union {
    __IOM u32 DAC_MCR;             
    
    struct {
      __IOM u32 MODE1      : 3;
                            
            u32            : 5;
      __IOM u32 DMADOUBLE1 : 1;    
      __IOM u32 SINFORMAT1 : 1;    
            u32            : 4;
      __IOM u32 HFSEL      : 2;    
      __IOM u32 MODE2      : 3;
                            
            u32            : 5;
      __IOM u32 DMADOUBLE2 : 1;    
      __IOM u32 SINFORMAT2 : 1;    
            u32            : 6;
    } DAC_MCR_b;
  } ;
  
  union {
    __IOM u32 DAC_SHSR1;           
    
    struct {
      __IOM u32 TSAMPLE1   : 10; 
                            
            u32            : 22;
    } DAC_SHSR1_b;
  } ;
  
  union {
    __IOM u32 DAC_SHSR2;           
    
    struct {
      __IOM u32 TSAMPLE2   : 10;
                            
            u32            : 22;
    } DAC_SHSR2_b;
  } ;
  
  union {
    __IOM u32 DAC_SHHR;            
    
    struct {
      __IOM u32 THOLD1     : 10;
                            
            u32            : 6;
      __IOM u32 THOLD2     : 10;
                            
            u32            : 6;
    } DAC_SHHR_b;
  } ;
  
  union {
    __IOM u32 DAC_SHRR;            
    
    struct {
      __IOM u32 TREFRESH1  : 8;
                            
            u32            : 8;
      __IOM u32 TREFRESH2  : 8;
                            
            u32            : 8;
    } DAC_SHRR_b;
  } ;
  __IM  u32  RESERVED[2];
  
  union {
    __IOM u32 DAC_STR1;            
    
    struct {
      __IOM u32 STRSTDATA1 : 12;   
      __IOM u32 STDIR1     : 1;    
            u32            : 3;
      __IOM u32 STINCDATA1 : 16;   
    } DAC_STR1_b;
  } ;
  
  union {
    __IOM u32 DAC_STR2;            
    
    struct {
      __IOM u32 STRSTDATA2 : 12;   
      __IOM u32 STDIR2     : 1;    
            u32            : 3;
      __IOM u32 STINCDATA2 : 16;   
    } DAC_STR2_b;
  } ;
  
  union {
    __IOM u32 DAC_STMODR;          
    
    struct {
      __IOM u32 STRSTTRIGSEL1 : 4; 
            u32               : 4;
      __IOM u32 STINCTRIGSEL1 : 4; 
            u32               : 4;
      __IOM u32 STRSTTRIGSEL2 : 4; 
            u32               : 4;
      __IOM u32 STINCTRIGSEL2 : 4; 
            u32               : 4;
    } DAC_STMODR_b;
  } ;

} DAC_HW;


/*============================= DEBUG MCU SUPPORT ==================================*/

typedef struct {                          
  
  union {
    __IM  u32 IDCODE;                
    
    struct {
      __IM  u32 DEV_ID     : 16;     
      __IM  u32 REV_ID     : 16;     
    } IDCODE_b;
  } ;
  
  union {
    __IOM u32 CR;                    
    
    struct {
      __IOM u32 DBG_SLEEP  : 1;      
      __IOM u32 DBG_STOP   : 1;      
      __IOM u32 DBG_STANDBY : 1;     
            u32            : 2;
      __IOM u32 TRACE_IOEN : 1;      
      __IOM u32 TRACE_MODE : 2;      
            u32            : 24;
    } CR_b;
  } ;
  
  union {
    __IOM u32 APB1FZ;              
    
    struct {
      __IOM u32 DBG_TIMER2_STOP : 1; 
      __IOM u32 DBG_TIM3_STOP : 1;   
      __IOM u32 DBG_TIM4_STOP : 1;   
      __IOM u32 DBG_TIM5_STOP : 1;   
      __IOM u32 DBG_TIMER6_STOP : 1; 
      __IOM u32 DBG_TIM7_STOP : 1;   
            u32               : 4;
      __IOM u32 DBG_RTC_STOP  : 1;    
      __IOM u32 DBG_WWDG_STOP : 1;   
      __IOM u32 DBG_IWDG_STOP : 1;   
            u32               : 8;
      __IOM u32 DBG_I2C1_STOP : 1;   
      __IOM u32 DBG_I2C2_STOP : 1;   
            u32               : 7;
      __IOM u32 DBG_I2C3_STOP : 1;   
      __IOM u32 DBG_LPTIMER_STOP : 1;
    } APB1FZ_b;
  } ;

#if defined (STM32G473)
    union {
    __IOM u32 APB1FZR2;              
    
    struct {
            u32            : 1;
      __IOM u32 DBG_I2C4_STOP : 1;   
            u32            : 30;
     } APB1FZR2_b;
    } ;
#endif    /* STM32G473 */
  
  union {
    __IOM u32 APB2FZ;               
    
    struct {
            u32               : 11;
      __IOM u32 DBG_TIM1_STOP : 1;   
            u32               : 1;
      __IOM u32 DBG_TIM8_STOP : 1;   
            u32                : 2;
      __IOM u32 DBG_TIM15_STOP : 1;  
      __IOM u32 DBG_TIM16_STOP : 1;  
      __IOM u32 DBG_TIM17_STOP : 1;  
            u32                : 1;
      __IOM u32 DBG_TIM20_STOP : 1;  
            u32                : 5;
      __IOM u32 DBG_HRTIM0_STOP : 1; 
      __IOM u32 DBG_HRTIM1_STOP : 1; 
      __IOM u32 DBG_HRTIM2_STOP : 1; 
      __IOM u32 DBG_HRTIM3_STOP : 1; 
            u32                 : 2;
    } APB2FZ_b;
  } ;
  
} DBGMCU_HW;


/*===================== DIRECT MEMORY ACCESS CONTROLLER ============================*/

typedef struct {                    
  
  union {
    __IM  u32 ISR;             
    
    struct {
      __IM  u32 GIF1       : 1;
      __IM  u32 TCIF1      : 1;
      __IM  u32 HTIF1      : 1;
      __IM  u32 TEIF1      : 1;
      __IM  u32 GIF2       : 1;
      __IM  u32 TCIF2      : 1;
      __IM  u32 HTIF2      : 1;
      __IM  u32 TEIF2      : 1;
      __IM  u32 GIF3       : 1;
      __IM  u32 TCIF3      : 1;
      __IM  u32 HTIF3      : 1;
      __IM  u32 TEIF3      : 1;
      __IM  u32 GIF4       : 1;
      __IM  u32 TCIF4      : 1;
      __IM  u32 HTIF4      : 1;
      __IM  u32 TEIF4      : 1;
      __IM  u32 GIF5       : 1;
      __IM  u32 TCIF5      : 1;
      __IM  u32 HTIF5      : 1;
      __IM  u32 TEIF5      : 1;
      __IM  u32 GIF6       : 1;
      __IM  u32 TCIF6      : 1;
      __IM  u32 HTIF6      : 1;
      __IM  u32 TEIF6      : 1;
      __IM  u32 GIF7       : 1;
      __IM  u32 TCIF7      : 1;
      __IM  u32 HTIF7      : 1;
      __IM  u32 TEIF7      : 1;
      __IM  u32 GIF8       : 1;
      __IM  u32 TCIF8      : 1;
      __IM  u32 HTIF8      : 1;
      __IM  u32 TEIF8      : 1;
    } ISR_b;
  } ;
  
  union {
    __OM  u32 IFCR;            
    
    struct {
      __OM  u32 GIF1       : 1;
      __OM  u32 TCIF1      : 1;
      __OM  u32 HTIF1      : 1;
      __OM  u32 TEIF1      : 1;
      __OM  u32 GIF2       : 1;
      __OM  u32 TCIF2      : 1;
      __OM  u32 HTIF2      : 1;
      __OM  u32 TEIF2      : 1;
      __OM  u32 GIF3       : 1;
      __OM  u32 TCIF3      : 1;
      __OM  u32 HTIF3      : 1;
      __OM  u32 TEIF3      : 1;
      __OM  u32 GIF4       : 1;
      __OM  u32 TCIF4      : 1;
      __OM  u32 HTIF4      : 1;
      __OM  u32 TEIF4      : 1;
      __OM  u32 GIF5       : 1;
      __OM  u32 TCIF5      : 1;
      __OM  u32 HTIF5      : 1;
      __OM  u32 TEIF5      : 1;
      __OM  u32 GIF6       : 1;
      __OM  u32 TCIF6      : 1;
      __OM  u32 HTIF6      : 1;
      __OM  u32 TEIF6      : 1;
      __OM  u32 GIF7       : 1;
      __OM  u32 TCIF7      : 1;
      __OM  u32 HTIF7      : 1;
      __OM  u32 TEIF7      : 1;
      __OM  u32 GIF8       : 1;
      __OM  u32 TCIF8      : 1;
      __OM  u32 HTIF8      : 1;
      __OM  u32 TEIF8      : 1;
    } IFCR_b;
  } ;

} DMA_HW;


typedef struct {

  union {
    __IOM u32 CR;            
    
    struct {
      __IOM u32 EN         : 1;
      __IOM u32 TCIE       : 1;
      __IOM u32 HTIE       : 1;
      __IOM u32 TEIE       : 1;
      __IOM u32 DIR        : 1;
      __IOM u32 CIRC       : 1;
      __IOM u32 PINC       : 1;
      __IOM u32 MINC       : 1;
      __IOM u32 PSIZE      : 2;
      __IOM u32 MSIZE      : 2;
      __IOM u32 PL         : 2;
      __IOM u32 MEM2MEM    : 1;
            u32            : 17;
    } CR_b;
  } ;
  
  __IOM u32 NDTR;
  __IOM u32 PAR;
  __IOM u32 MAR;

} DMA_CHANNEL_HW;


/*========================= DMA REQUEST MULTIPLEXER  ===============================*/

typedef struct
{
  __IO u32   CCR;
} DMAMUX_CHANNEL_HW;

typedef struct
{
  __IO u32   CSR;
  __IO u32   CFR;
} DMAMUX_CHANNELSTATUS_HW;

typedef struct
{
  __IO u32   RGCR;
} DMAMUX_REQUESTGEN_HW;

typedef struct
{
  __IO u32   RGSR;
  __IO u32   RGCFR;
} DMAMUX_REQUESTGENSTATUS_HW; 


/*===================== EXTERNAL INTERRUPT/EVENT CONTROLLER ========================*/

typedef struct {                      
  
  union {
    __IOM u32 IMR1;              
    
    struct {
      __IOM u32 IM0        : 1;  
      __IOM u32 IM1        : 1;  
      __IOM u32 IM2        : 1;  
      __IOM u32 IM3        : 1;  
      __IOM u32 IM4        : 1;  
      __IOM u32 IM5        : 1;  
      __IOM u32 IM6        : 1;  
      __IOM u32 IM7        : 1;  
      __IOM u32 IM8        : 1;  
      __IOM u32 IM9        : 1;  
      __IOM u32 IM10       : 1;  
      __IOM u32 IM11       : 1;  
      __IOM u32 IM12       : 1;  
      __IOM u32 IM13       : 1;  
      __IOM u32 IM14       : 1;  
      __IOM u32 IM15       : 1;  
      __IOM u32 IM16       : 1;  
      __IOM u32 IM17       : 1;  
      __IOM u32 IM18       : 1;  
      __IOM u32 IM19       : 1;  
      __IOM u32 IM20       : 1;  
      __IOM u32 IM21       : 1;  
      __IOM u32 IM22       : 1;  
      __IOM u32 IM23       : 1;  
      __IOM u32 IM24       : 1;  
      __IOM u32 IM25       : 1;  
      __IOM u32 IM26       : 1;  
      __IOM u32 IM27       : 1;  
      __IOM u32 IM28       : 1;  
      __IOM u32 IM29       : 1;  
      __IOM u32 IM30       : 1;  
      __IOM u32 IM31       : 1;  
    } IMR1_b;
  } ;
  
  union {
    __IOM u32 EMR1;              
    
    struct {
      __IOM u32 EM0        : 1;  
      __IOM u32 EM1        : 1;  
      __IOM u32 EM2        : 1;  
      __IOM u32 EM3        : 1;  
      __IOM u32 EM4        : 1;  
      __IOM u32 EM5        : 1;  
      __IOM u32 EM6        : 1;  
      __IOM u32 EM7        : 1;  
      __IOM u32 EM8        : 1;  
      __IOM u32 EM9        : 1;  
      __IOM u32 EM10       : 1;  
      __IOM u32 EM11       : 1;  
      __IOM u32 EM12       : 1;  
      __IOM u32 EM13       : 1;  
      __IOM u32 EM14       : 1;  
      __IOM u32 EM15       : 1;  
      __IOM u32 EM16       : 1;  
      __IOM u32 EM17       : 1;  
      __IOM u32 EM18       : 1;  
      __IOM u32 EM19       : 1;  
      __IOM u32 EM20       : 1;  
      __IOM u32 EM21       : 1;  
      __IOM u32 EM22       : 1;  
      __IOM u32 EM23       : 1;  
      __IOM u32 EM24       : 1;  
      __IOM u32 EM25       : 1;  
      __IOM u32 EM26       : 1;  
      __IOM u32 EM27       : 1;  
      __IOM u32 EM28       : 1;  
      __IOM u32 EM29       : 1;  
      __IOM u32 EM30       : 1;  
      __IOM u32 EM31       : 1;  
    } EMR1_b;
  } ;
  
  union {
    __IOM u32 RTSR1;             
    
    struct {
      __IOM u32 RT0        : 1;  
      __IOM u32 RT1        : 1;  
      __IOM u32 RT2        : 1;  
      __IOM u32 RT3        : 1;  
      __IOM u32 RT4        : 1;  
      __IOM u32 RT5        : 1; 
      __IOM u32 RT6        : 1;  
      __IOM u32 RT7        : 1;  
      __IOM u32 RT8        : 1;  
      __IOM u32 RT9        : 1;  
      __IOM u32 RT10       : 1;  
      __IOM u32 RT11       : 1;  
      __IOM u32 RT12       : 1;  
      __IOM u32 RT13       : 1;  
      __IOM u32 RT14       : 1;  
      __IOM u32 RT15       : 1;  
      __IOM u32 RT16       : 1;  
      __IOM u32 RT18       : 1;  
      __IOM u32 RT19       : 1;  
      __IOM u32 RT20       : 1;  
      __IOM u32 RT21       : 1;  
      __IOM u32 RT22       : 1;  
            u32            : 6;
      __IOM u32 RT         : 3;  
    } RTSR1_b;
  } ;
  
  union {
    __IOM u32 FTSR1;             
    
    struct {
      __IOM u32 FT0        : 1;  
      __IOM u32 FT1        : 1;  
      __IOM u32 FT2        : 1;  
      __IOM u32 FT3        : 1;  
      __IOM u32 FT4        : 1;  
      __IOM u32 FT5        : 1;  
      __IOM u32 FT6        : 1;  
      __IOM u32 FT7        : 1;  
      __IOM u32 FT8        : 1;  
      __IOM u32 FT9        : 1;  
      __IOM u32 FT10       : 1;  
      __IOM u32 FT11       : 1;  
      __IOM u32 FT12       : 1;  
      __IOM u32 FT13       : 1;  
      __IOM u32 FT14       : 1;  
      __IOM u32 FT15       : 1;  
      __IOM u32 FT16       : 1;  
            u32            : 1;
      __IOM u32 FT18       : 1;  
      __IOM u32 FT19       : 1;  
      __IOM u32 FT20       : 1;  
      __IOM u32 FT21       : 1;  
      __IOM u32 FT22       : 1;  
            u32            : 9;
    } FTSR1_b;
  } ;
  
  union {
    __IOM u32 SWIER1;            
    
    struct {
      __IOM u32 SWI0       : 1;  
      __IOM u32 SWI1       : 1;  
      __IOM u32 SWI2       : 1;  
      __IOM u32 SWI3       : 1;  
      __IOM u32 SWI4       : 1;  
      __IOM u32 SWI5       : 1;  
      __IOM u32 SWI6       : 1;  
      __IOM u32 SWI7       : 1;  
      __IOM u32 SWI8       : 1;  
      __IOM u32 SWI9       : 1;  
      __IOM u32 SWI10      : 1;  
      __IOM u32 SWI11      : 1;  
      __IOM u32 SWI12      : 1;  
      __IOM u32 SWI13      : 1;  
      __IOM u32 SWI14      : 1;  
      __IOM u32 SWI15      : 1;  
      __IOM u32 SWI16      : 1;  
            u32            : 1;
      __IOM u32 SWI18      : 1;  
      __IOM u32 SWI19      : 1;  
      __IOM u32 SWI20      : 1;  
      __IOM u32 SWI21      : 1;  
      __IOM u32 SWI22      : 1;  
            u32            : 9;
    } SWIER1_b;
  } ;
  
  union {
    __IOM u32 PR1;               
    
    struct {
      __IOM u32 PIF0       : 1;  
      __IOM u32 PIF1       : 1;  
      __IOM u32 PIF2       : 1;  
      __IOM u32 PIF3       : 1;  
      __IOM u32 PIF4       : 1;  
      __IOM u32 PIF5       : 1;  
      __IOM u32 PIF6       : 1;  
      __IOM u32 PIF7       : 1;  
      __IOM u32 PIF8       : 1;  
      __IOM u32 PIF9       : 1;  
      __IOM u32 PIF10      : 1;  
      __IOM u32 PIF11      : 1;  
      __IOM u32 PIF12      : 1;  
      __IOM u32 PIF13      : 1;  
      __IOM u32 PIF14      : 1;  
      __IOM u32 PIF15      : 1;  
      __IOM u32 PIF16      : 1;  
            u32            : 1;
      __IOM u32 PIF18      : 1;  
      __IOM u32 PIF19      : 1;  
      __IOM u32 PIF20      : 1;  
      __IOM u32 PIF21      : 1;  
      __IOM u32 PIF22      : 1;  
            u32            : 9;
    } PR1_b;
  } ;
  __IM  u32  RESERVED[2];
  
  union {
    __IOM u32 IMR2;              
    
    struct {
      __IOM u32 IM32       : 1;  
      __IOM u32 IM33       : 1;  
      __IOM u32 IM34       : 1;  
      __IOM u32 IM35       : 1;  
      __IOM u32 IM36       : 1;  
      __IOM u32 IM37       : 1;  
      __IOM u32 IM38       : 1;  
      __IOM u32 IM39       : 1;  
      __IOM u32 IM40       : 1;  
      __IOM u32 IM41       : 1;  
      __IOM u32 IM42       : 1;  
      __IOM u32 IM43       : 1;  
            u32            : 20;
    } IMR2_b;
  } ;
  
  union {
    __IOM u32 EMR2;              
    
    struct {
      __IOM u32 EM32       : 1;  
      __IOM u32 EM33       : 1;  
      __IOM u32 EM34       : 1;  
      __IOM u32 EM35       : 1;  
      __IOM u32 EM36       : 1;  
      __IOM u32 EM37       : 1;  
      __IOM u32 EM38       : 1;  
      __IOM u32 EM39       : 1;  
      __IOM u32 EM40       : 1;  
            u32            : 23;
    } EMR2_b;
  } ;
  
  union {
    __IOM u32 RTSR2;             
    
    struct {
      __IOM u32 RT32       : 1;  
      __IOM u32 RT33       : 1;  
            u32            : 4;
      __IOM u32 RT38       : 1;  
      __IOM u32 RT39       : 1;  
      __IOM u32 RT40       : 1;  
      __IOM u32 RT41       : 1;  
            u32            : 22;
    } RTSR2_b;
  } ;
  
  union {
    __IOM u32 FTSR2;             
    
    struct {
            u32            : 3;
      __IOM u32 FT35       : 1;  
      __IOM u32 FT36       : 1;  
      __IOM u32 FT37       : 1;  
      __IOM u32 FT38       : 1;  
            u32            : 25;
    } FTSR2_b;
  } ;
  
  union {
    __IOM u32 SWIER2;            
    
    struct {
            u32            : 3;
      __IOM u32 SWI35      : 1;  
      __IOM u32 SWI36      : 1;  
      __IOM u32 SWI37      : 1;  
      __IOM u32 SWI38      : 1;  
            u32            : 25;
    } SWIER2_b;
  } ;
  
  union {
    __IOM u32 PR2;               
    
    struct {
            u32            : 3;
      __IOM u32 PIF35      : 1;  
      __IOM u32 PIF36      : 1;  
      __IOM u32 PIF37      : 1;  
      __IOM u32 PIF38      : 1;  
            u32            : 25;
    } PR2_b;
  } ;

} EXTI_HW;


/*========================== FLASH MEMORY REGISTERS ================================*/

typedef struct {                      
  
  #if defined (STM32F303)
  union {
    __IOM u32 ACR;                    
    
    struct {
      __IOM u32 LATENCY    : 3;       
            u32            : 1;
      __IOM u32 PRFTBE     : 1;       
      __IM  u32 PRFTBS     : 1;       
            u32            : 26;
    } ACR_b;
  } ;
  
  __OM  u32 KEYR; 

  __OM  u32 OPTKEYR;
  
  union {
    __IOM u32 SR;                     
    
    struct {
      __IM  u32 BSY        : 1;       
            u32            : 1;
      __IOM u32 PGERR      : 1;       
            u32            : 1;
      __IOM u32 WRPRT      : 1;       
      __IOM u32 EOP        : 1;       
            u32            : 26;
    } SR_b;
  } ;
  
  union {
    __IOM u32 CR;                     
    
    struct {
      __IOM u32 PG         : 1;       
      __IOM u32 PER        : 1;       
      __IOM u32 MER        : 1;       
            u32            : 1;
      __IOM u32 OPTPG      : 1;       
      __IOM u32 OPTER      : 1;       
      __IOM u32 STRT       : 1;       
      __IOM u32 LOCK       : 1;       
            u32            : 1;
      __IOM u32 OPTWRE     : 1;       
      __IOM u32 ERRIE      : 1;       
            u32            : 1;
      __IOM u32 EOPIE      : 1;       
      __IOM u32 FORCE_OPTLOAD : 1;    
            u32            : 18;
    } CR_b;
  } ;
  
  __OM  u32 AR; 

  __IM  u32  RESERVED;
  
  union {
    __IM  u32 OBR;                    
    
    struct {
      __IM  u32 OPTERR     : 1;       
      __IM  u32 LEVEL1_PROT : 1;      
      __IM  u32 LEVEL2_PROT : 1;      
            u32            : 5;
      __IM  u32 WDG_SW     : 1;       
      __IM  u32 nRST_STOP  : 1;       
      __IM  u32 nRST_STDBY : 1;       
            u32            : 1;
      __IM  u32 BOOT1      : 1;       
      __IM  u32 VDDA_MONITOR : 1;     
      __IM  u32 SRAM_PARITY_CHECK : 1;
            u32            : 1;
      __IM  u32 Data0      : 8;       
      __IM  u32 Data1      : 8;       
    } OBR_b;
  } ;
  
  __IM  u32 WRPR; 

  #elif defined (STM32G473)
  union {
    __IOM u32 ACR;               
    
    struct {
      __IOM u32 LATENCY    : 4;  
            u32            : 4;
      __IOM u32 PRFTEN     : 1;  
      __IOM u32 ICEN       : 1;  
      __IOM u32 DCEN       : 1;  
      __IOM u32 ICRST      : 1;  
      __IOM u32 DCRST      : 1;  
      __IOM u32 RUN_PD     : 1;  
      __IOM u32 SLEEP_PD   : 1;  
            u32            : 3;
      __IOM u32 DBG_SWEN   : 1;  
            u32            : 13;
    } ACR_b;
  } ;

  __OM  u32 PDKEYR;  
  
  __OM  u32 KEYR;  
 
  __OM  u32 OPTKEYR;
  
  union {
    __IOM u32 SR;                
    
    struct {
      __IOM u32 EOP        : 1;  
      __IOM u32 OPERR      : 1;  
            u32            : 1;
      __IOM u32 PROGERR    : 1;  
      __IOM u32 WRPERR     : 1;  
      __IOM u32 PGAERR     : 1;  
      __IOM u32 SIZERR     : 1;  
      __IOM u32 PGSERR     : 1;  
      __IOM u32 MISERR     : 1;  
      __IOM u32 FASTERR    : 1;  
            u32            : 4;
      __IOM u32 RDERR      : 1;  
      __IOM u32 OPTVERR    : 1;  
      __IM  u32 BSY        : 1;  
            u32            : 15;
    } SR_b;
  } ;
  
  union {
    __IOM u32 CR;                
    
    struct {
      __IOM u32 PG         : 1;  
      __IOM u32 PER        : 1;  
      __IOM u32 MER1       : 1;  
      __IOM u32 PNB        : 7;  
            u32            : 6;
      __IOM u32 STRT       : 1;  
      __IOM u32 OPTSTRT    : 1;  
      __IOM u32 FSTPG      : 1;  
            u32            : 5;
      __IOM u32 EOPIE      : 1;  
      __IOM u32 ERRIE      : 1;  
      __IOM u32 RDERRIE    : 1;  
      __IOM u32 OBL_LAUNCH : 1;  
      __IOM u32 SEC_PROT1  : 1;  
            u32            : 1;
      __IOM u32 OPTLOCK    : 1;  
      __IOM u32 LOCK       : 1;  
    } CR_b;
  } ;
  
  union {
    __IOM u32 ECCR;              
    
    struct {
      __IM  u32 ADDR_ECC   : 19; 
            u32            : 2;
      __IM  u32 BK_ECC     : 1;  
      __IM  u32 SYSF_ECC   : 1;  
            u32            : 1;
      __IOM u32 ECCIE      : 1;  
            u32            : 3;
      __IOM u32 ECCC2      : 1;  
      __IOM u32 ECCD2      : 1;  
      __IOM u32 ECCC       : 1;  
      __IOM u32 ECCD       : 1;  
    } ECCR_b;
  } ;
  __IM  u32  RESERVED;
  
  union {
    __IOM u32 OPTR;              
    
    struct {
      __IOM u32 RDP        : 8;  
      __IOM u32 BOR_LEV    : 3;  
            u32            : 1;
      __IOM u32 nRST_STOP  : 1;  
      __IOM u32 nRST_STDBY : 1;  
      __IOM u32 nRST_SHDW  : 1;  
            u32            : 1;
      __IOM u32 IWDG_SW    : 1;  
      __IOM u32 IWDG_STOP  : 1;  
      __IOM u32 IWDG_STDBY : 1;  
      __IOM u32 WWDG_SW    : 1;  
      __IOM u32 BFB2       : 1;  
            u32            : 1;
      __IOM u32 DBANK      : 1;  
      __IOM u32 nBOOT1     : 1;  
      __IOM u32 SRAM2_PE   : 1;  
      __IOM u32 CCMSRAM_RST : 1; 
      __IOM u32 nSWBOOT0   : 1;  
      __IOM u32 nBOOT0     : 1;  
      __IOM u32 NRST_MODE  : 2;  
      __IOM u32 IRHEN      : 1;  
            u32            : 1;
    } OPTR_b;
  } ;
  
  union {
    __IOM u32 PCROP1SR;          
    
    struct {
      __IOM u32 PCROP1_STRT : 15;
            u32            : 17;
    } PCROP1SR_b;
  } ;
  
  union {
    __IOM u32 PCROP1ER;          
    
    struct {
      __IOM u32 PCROP1_END : 15; 
            u32            : 16;
      __IOM u32 PCROP_RDP  : 1;  
    } PCROP1ER_b;
  } ;
  
  union {
    __IOM u32 WRP1AR;            
    
    struct {
      __IOM u32 WRP1A_STRT : 7;  
            u32            : 9;
      __IOM u32 WRP1A_END  : 7;  
            u32            : 9;
    } WRP1AR_b;
  } ;
  
  union {
    __IOM u32 WRP1BR;            
    
    struct {
      __IOM u32 WRP1B_STRT : 7;  
            u32            : 9;
      __IOM u32 WRP1B_END  : 7;  
            u32            : 9;
    } WRP1BR_b;
  } ;
  __IM  u32  RESERVED1[15];
  
  union {
    __IOM u32 SEC1R;             
    
    struct {
      __IOM u32 SEC_SIZE1  : 8;  
            u32            : 8;
      __IOM u32 BOOT_LOCK  : 1;  
            u32            : 15;
    } SEC1R_b;
  } ;

  union {
    __IOM u32 SEC2R;             
    
    struct {
      __IOM u32 SEC_SIZE1  : 8;  
            u32            : 8;
      __IOM u32 BOOT_LOCK  : 1;  
            u32            : 15;
    } SEC2R_b;
  } ;
  #endif    /* STM32G473*/

} FLASH_HW;


/*========================== FILTER MATH ACCELERATOR ===============================*/

typedef struct {                     
  
  union {
    __IOM u32 X1BUFCFG;         
    
    struct {
      __IOM u32 X1_BASE    : 8; 
      __IOM u32 X1_BUF_SIZE : 8;
            u32            : 8;
      __IOM u32 FULL_WM    : 2; 
            u32            : 6;
    } X1BUFCFG_b;
  } ;
  
  union {
    __IOM u32 X2BUFCFG;         
    
    struct {
      __IOM u32 X2_BASE    : 8; 
      __IOM u32 X2_BUF_SIZE : 8;
            u32            : 16;
    } X2BUFCFG_b;
  } ;
  
  union {
    __IOM u32 YBUFCFG;          
    
    struct {
      __IOM u32 Y_BASE     : 8; 
      __IOM u32 Y_BUF_SIZE : 8; 
            u32            : 8;
      __IOM u32 EMPTY_WM   : 2; 
            u32            : 6;
    } YBUFCFG_b;
  } ;
  
  union {
    __IOM u32 PARAM;            
    
    struct {
      __IOM u32 P          : 8; 
      __IOM u32 Q          : 8; 
      __IOM u32 R          : 8; 
      __IOM u32 FUNC       : 7; 
      __IOM u32 START      : 1; 
    } PARAM_b;
  } ;
  
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 RIEN       : 1; 
      __IOM u32 WIEN       : 1; 
      __IOM u32 OVFLIEN    : 1; 
      __IOM u32 UNFLIEN    : 1; 
      __IOM u32 SATIEN     : 1; 
            u32            : 3;
      __IOM u32 DMAREN     : 1; 
      __IOM u32 DMAWEN     : 1; 
            u32            : 5;
      __IOM u32 CLIPEN     : 1; 
      __IOM u32 RESET      : 1; 
            u32            : 15;
    } CR_b;
  } ;
  
  union {
    __IM  u32 SR;               
    
    struct {
      __IM  u32 YEMPTY     : 1; 
      __IM  u32 X1FULL     : 1; 
            u32            : 6;
      __IM  u32 OVFL       : 1; 
      __IM  u32 UNFL       : 1; 
      __IM  u32 SAT        : 1; 
            u32            : 21;
    } SR_b;
  } ;
  
  union {
    __OM  u32 WDATA;            
    
    struct {
      __OM  u32 WDATA      : 16;
            u32            : 16;
    } WDATA_b;
  } ;
  
  union {
    __IM  u32 RDATA;            
    
    struct {
      __IM  u32 RDATA      : 16;
            u32            : 16;
    } RDATA_b;
  } ;

} FMAC_HW;


/*=========================== GENERAL PURPOSE I/O ==================================*/

typedef struct {                      
  
  union {
    __IOM u32 MODER;             
    
    struct {
      __IOM u32 MODER0     : 2;  
      __IOM u32 MODER1     : 2;  
      __IOM u32 MODER2     : 2;  
      __IOM u32 MODER3     : 2;  
      __IOM u32 MODER4     : 2;  
      __IOM u32 MODER5     : 2;  
      __IOM u32 MODER6     : 2;  
      __IOM u32 MODER7     : 2;  
      __IOM u32 MODER8     : 2;  
      __IOM u32 MODER9     : 2;  
      __IOM u32 MODER10    : 2;  
      __IOM u32 MODER11    : 2;  
      __IOM u32 MODER12    : 2;  
      __IOM u32 MODER13    : 2;  
      __IOM u32 MODER14    : 2;  
      __IOM u32 MODER15    : 2;  
    } MODER_b;
  } ;
  
  union {
    __IOM u32 OTYPER;            
    
    struct {
      __IOM u32 OT0        : 1;  
      __IOM u32 OT1        : 1;  
      __IOM u32 OT2        : 1;  
      __IOM u32 OT3        : 1;  
      __IOM u32 OT4        : 1;  
      __IOM u32 OT5        : 1;  
      __IOM u32 OT6        : 1;  
      __IOM u32 OT7        : 1;  
      __IOM u32 OT8        : 1;  
      __IOM u32 OT9        : 1;  
      __IOM u32 OT10       : 1;  
      __IOM u32 OT11       : 1;  
      __IOM u32 OT12       : 1;  
      __IOM u32 OT13       : 1;  
      __IOM u32 OT14       : 1;  
      __IOM u32 OT15       : 1;  
            u32            : 16;
    } OTYPER_b;
  } ;
  
  union {
    __IOM u32 OSPEEDR;           
    
    struct {
      __IOM u32 OSPEEDR0   : 2;  
      __IOM u32 OSPEEDR1   : 2;  
      __IOM u32 OSPEEDR2   : 2;  
      __IOM u32 OSPEEDR3   : 2;  
      __IOM u32 OSPEEDR4   : 2;  
      __IOM u32 OSPEEDR5   : 2;  
      __IOM u32 OSPEEDR6   : 2;  
      __IOM u32 OSPEEDR7   : 2;  
      __IOM u32 OSPEEDR8   : 2;  
      __IOM u32 OSPEEDR9   : 2;  
      __IOM u32 OSPEEDR10  : 2;  
      __IOM u32 OSPEEDR11  : 2;  
      __IOM u32 OSPEEDR12  : 2;  
      __IOM u32 OSPEEDR13  : 2;  
      __IOM u32 OSPEEDR14  : 2;  
      __IOM u32 OSPEEDR15  : 2;  
    } OSPEEDR_b;
  } ;
  
  union {
    __IOM u32 PUPDR;             
    
    struct {
      __IOM u32 PUPDR0     : 2;  
      __IOM u32 PUPDR1     : 2;  
      __IOM u32 PUPDR2     : 2;  
      __IOM u32 PUPDR3     : 2;  
      __IOM u32 PUPDR4     : 2;  
      __IOM u32 PUPDR5     : 2;  
      __IOM u32 PUPDR6     : 2;  
      __IOM u32 PUPDR7     : 2;  
      __IOM u32 PUPDR8     : 2;  
      __IOM u32 PUPDR9     : 2;  
      __IOM u32 PUPDR10    : 2;  
      __IOM u32 PUPDR11    : 2;  
      __IOM u32 PUPDR12    : 2;  
      __IOM u32 PUPDR13    : 2;  
      __IOM u32 PUPDR14    : 2;  
      __IOM u32 PUPDR15    : 2;  
    } PUPDR_b;
  } ;
  
  union {
    __IM  u32 IDR;               
    
    struct {
      __IM  u32 IDR0       : 1;  
      __IM  u32 IDR1       : 1;  
      __IM  u32 IDR2       : 1;  
      __IM  u32 IDR3       : 1;  
      __IM  u32 IDR4       : 1;  
      __IM  u32 IDR5       : 1;  
      __IM  u32 IDR6       : 1;  
      __IM  u32 IDR7       : 1;  
      __IM  u32 IDR8       : 1;  
      __IM  u32 IDR9       : 1;  
      __IM  u32 IDR10      : 1;  
      __IM  u32 IDR11      : 1;  
      __IM  u32 IDR12      : 1;  
      __IM  u32 IDR13      : 1;  
      __IM  u32 IDR14      : 1;  
      __IM  u32 IDR15      : 1;  
            u32            : 16;
    } IDR_b;
  } ;
  
  union {
    __IOM u32 ODR;               
    
    struct {
      __IOM u32 ODR0       : 1;  
      __IOM u32 ODR1       : 1;  
      __IOM u32 ODR2       : 1;  
      __IOM u32 ODR3       : 1;  
      __IOM u32 ODR4       : 1;  
      __IOM u32 ODR5       : 1;  
      __IOM u32 ODR6       : 1;  
      __IOM u32 ODR7       : 1;  
      __IOM u32 ODR8       : 1;  
      __IOM u32 ODR9       : 1;  
      __IOM u32 ODR10      : 1;  
      __IOM u32 ODR11      : 1;  
      __IOM u32 ODR12      : 1;  
      __IOM u32 ODR13      : 1;  
      __IOM u32 ODR14      : 1;  
      __IOM u32 ODR15      : 1;  
            u32            : 16;
    } ODR_b;
  } ;
  
  union {
    __OM  u32 BSRR;              
    
    struct {
      __OM  u32 BS0        : 1;  
      __OM  u32 BS1        : 1;  
      __OM  u32 BS2        : 1;  
      __OM  u32 BS3        : 1;  
      __OM  u32 BS4        : 1;  
      __OM  u32 BS5        : 1;  
      __OM  u32 BS6        : 1;  
      __OM  u32 BS7        : 1;  
      __OM  u32 BS8        : 1;  
      __OM  u32 BS9        : 1;  
      __OM  u32 BS10       : 1;  
      __OM  u32 BS11       : 1;  
      __OM  u32 BS12       : 1;  
      __OM  u32 BS13       : 1;  
      __OM  u32 BS14       : 1;  
      __OM  u32 BS15       : 1;  
      __OM  u32 BR0        : 1;  
      __OM  u32 BR1        : 1;  
      __OM  u32 BR2        : 1;  
      __OM  u32 BR3        : 1;  
      __OM  u32 BR4        : 1;  
      __OM  u32 BR5        : 1;  
      __OM  u32 BR6        : 1;  
      __OM  u32 BR7        : 1;  
      __OM  u32 BR8        : 1;  
      __OM  u32 BR9        : 1;  
      __OM  u32 BR10       : 1;  
      __OM  u32 BR11       : 1;  
      __OM  u32 BR12       : 1;  
      __OM  u32 BR13       : 1;  
      __OM  u32 BR14       : 1;  
      __OM  u32 BR15       : 1;  
    } BSRR_b;
  } ;
  
  union {
    __IOM u32 LCKR;              
    
    struct {
      __IOM u32 LCK0       : 1;  
      __IOM u32 LCK1       : 1;  
      __IOM u32 LCK2       : 1;  
      __IOM u32 LCK3       : 1;  
      __IOM u32 LCK4       : 1;  
      __IOM u32 LCK5       : 1;  
      __IOM u32 LCK6       : 1;  
      __IOM u32 LCK7       : 1;  
      __IOM u32 LCK8       : 1;  
      __IOM u32 LCK9       : 1;  
      __IOM u32 LCK10      : 1;  
      __IOM u32 LCK11      : 1;  
      __IOM u32 LCK12      : 1;  
      __IOM u32 LCK13      : 1;  
      __IOM u32 LCK14      : 1;  
      __IOM u32 LCK15      : 1;  
      __IOM u32 LCKK       : 1;  
            u32            : 15;
    } LCKR_b;
  } ;
 
  __IOM u32 AFR[2];
  
  union {
    __OM  u32 BRR;               
    
    struct {
      __OM  u32 BR0        : 1;  
      __OM  u32 BR1        : 1;  
      __OM  u32 BR2        : 1;  
      __OM  u32 BR3        : 1;  
      __OM  u32 BR4        : 1;  
      __OM  u32 BR5        : 1;  
      __OM  u32 BR6        : 1;  
      __OM  u32 BR7        : 1;  
      __OM  u32 BR8        : 1;  
      __OM  u32 BR9        : 1;  
      __OM  u32 BR10       : 1;  
      __OM  u32 BR11       : 1;  
      __OM  u32 BR12       : 1;  
      __OM  u32 BR13       : 1;  
      __OM  u32 BR14       : 1;  
      __OM  u32 BR15       : 1;  
            u32            : 16;
    } BRR_b;
  } ;

} GPIO_HW;


/*========================== INTER INTEGRATED CIRCUIT ==============================*/

typedef struct {                      
  
  union {
    __IOM u32 CR1;               
    
    struct {
      __IOM u32 PE         : 1;  
      __IOM u32 TXIE       : 1;  
      __IOM u32 RXIE       : 1;  
      __IOM u32 ADDRIE     : 1;  
      __IOM u32 NACKIE     : 1;  
      __IOM u32 STOPIE     : 1;  
      __IOM u32 TCIE       : 1;  
      __IOM u32 ERRIE      : 1;  
      __IOM u32 DNF        : 4;  
      __IOM u32 ANFOFF     : 1;  
            u32            : 1;
      __IOM u32 TXDMAEN    : 1;  
      __IOM u32 RXDMAEN    : 1;  
      __IOM u32 SBC        : 1;  
      __IOM u32 NOSTRETCH  : 1;  
      __IOM u32 WUPEN      : 1;  
      __IOM u32 GCEN       : 1;  
      __IOM u32 SMBHEN     : 1;  
      __IOM u32 SMBDEN     : 1;  
      __IOM u32 ALERTEN    : 1;  
      __IOM u32 PECEN      : 1;  
            u32            : 8;
    } CR1_b;
  } ;
  
  union {
    __IOM u32 CR2;               
    
    struct {
      __IOM u32 SADD       : 10; 
      __IOM u32 RD_WRN     : 1;  
      __IOM u32 ADD10      : 1;  
      __IOM u32 HEAD10R    : 1; 
      __IOM u32 START      : 1;  
      __IOM u32 STOP       : 1;  
      __IOM u32 NACK       : 1;  
      __IOM u32 NBYTES     : 8;  
      __IOM u32 RELOAD     : 1;  
      __IOM u32 AUTOEND    : 1;  
      __IOM u32 PECBYTE    : 1;  
            u32            : 5;
    } CR2_b;
  } ;
  
  union {
    __IOM u32 OAR1;              
    
    struct {
      __IOM u32 OA1        : 10; 
      __IOM u32 OA1MODE    : 1;  
            u32            : 4;
      __IOM u32 OA1EN      : 1;  
            u32            : 16;
    } OAR1_b;
  } ;
  
  union {
    __IOM u32 OAR2;              
    
    struct {
            u32            : 1;
      __IOM u32 OA2        : 7;  
      __IOM u32 OA2MSK     : 3;  
            u32            : 4;
      __IOM u32 OA2EN      : 1;  
            u32            : 16;
    } OAR2_b;
  } ;
  
  union {
    __IOM u32 TIMINGR;           
    
    struct {
      __IOM u32 SCLL       : 8;  
      __IOM u32 SCLH       : 8;  
      __IOM u32 SDADEL     : 4;  
      __IOM u32 SCLDEL     : 4;  
            u32            : 4;
      __IOM u32 PRESC      : 4;  
    } TIMINGR_b;
  } ;
  
  union {
    __IOM u32 TIMEOUTR;          
    
    struct {
      __IOM u32 TIMEOUTA   : 12; 
      __IOM u32 TIDLE      : 1;  
            u32            : 2;
      __IOM u32 TIMOUTEN   : 1;  
      __IOM u32 TIMEOUTB   : 12; 
            u32            : 3;
      __IOM u32 TEXTEN     : 1;  
    } TIMEOUTR_b;
  } ;
  
  union {
    __IOM u32 ISR;               
    
    struct {
      __IOM u32 TXE        : 1;  
      __IOM u32 TXIS       : 1;  
      __IM  u32 RXNE       : 1;  
      __IM  u32 ADDR       : 1;  
      __IM  u32 NACKF      : 1;  
      __IM  u32 STOPF      : 1;  
      __IM  u32 TC         : 1;  
      __IM  u32 TCR        : 1;  
      __IM  u32 BERR       : 1;  
      __IM  u32 ARLO       : 1;  
      __IM  u32 OVR        : 1;  
      __IM  u32 PECERR     : 1;  
      __IM  u32 TIMEOUT    : 1;  
      __IM  u32 ALERT      : 1;  
            u32            : 1;
      __IM  u32 BUSY       : 1;  
      __IM  u32 DIR        : 1;  
      __IM  u32 ADDCODE    : 7;  
            u32            : 8;
    } ISR_b;
  } ;
  
  union {
    __OM  u32 ICR;               
    
    struct {
            u32            : 3;
      __OM  u32 ADDRCF     : 1;  
      __OM  u32 NACKCF     : 1;  
      __OM  u32 STOPCF     : 1;  
            u32            : 2;
      __OM  u32 BERRCF     : 1;  
      __OM  u32 ARLOCF     : 1;  
      __OM  u32 OVRCF      : 1;  
      __OM  u32 PECCF      : 1;  
      __OM  u32 TIMOUTCF   : 1;  
      __OM  u32 ALERTCF    : 1;  
            u32            : 18;
    } ICR_b;
  } ;
  
  union {
    __IM  u32 PECR;              
    
    struct {
      __IM  u32 PEC        : 8;  
            u32            : 24;
    } PECR_b;
  } ;
  
  union {
    __IM  u32 RXDR;              
    
    struct {
      __IM  u32 RXDATA     : 8;  
            u32            : 24;
    } RXDR_b;
  } ;
  
  union {
    __IOM u32 TXDR;              
    
    struct {
      __IOM u32 TXDATA     : 8;  
            u32            : 24;
    } TXDR_b;
  } ;

} I2C_HW;


/*============================ INDEPENDANT WATCHDOG ================================*/

typedef struct {                     
  
  union {
    __OM  u32 KR;               
    
    struct {
      __OM  u32 KEY        : 16;
            u32            : 16;
    } KR_b;
  } ;
  
  union {
    __IOM u32 PR;               
    
    struct {
      __IOM u32 PR         : 3; 
            u32            : 29;
    } PR_b;
  } ;
  
  union {
    __IOM u32 RLR;              
    
    struct {
      __IOM u32 RL         : 12;
            u32            : 20;
    } RLR_b;
  } ;
  
  union {
    __IM  u32 SR;               
    
    struct {
      __IM  u32 PVU        : 1; 
      __IM  u32 RVU        : 1; 
      __IM  u32 WVU        : 1; 
            u32            : 29;
    } SR_b;
  } ;
  
  union {
    __IOM u32 WINR;             
    
    struct {
      __IOM u32 WIN        : 12;
            u32            : 20;
    } WINR_b;
  } ;

} IWDG_HW;


/*============================== LOW POWER TIMER ===================================*/

typedef struct {                     
  
  union {
    __IM  u32 ISR;              
    
    struct {
      __IM  u32 CMPM       : 1; 
      __IM  u32 ARRM       : 1; 
      __IM  u32 EXTTRIG    : 1; 
      __IM  u32 CMPOK      : 1; 
      __IM  u32 ARROK      : 1; 
      __IM  u32 UP         : 1; 
      __IM  u32 DOWN       : 1; 
            u32            : 25;
    } ISR_b;
  } ;
  
  union {
    __OM  u32 ICR;              
    
    struct {
      __OM  u32 CMPMCF     : 1; 
      __OM  u32 ARRMCF     : 1; 
      __OM  u32 EXTTRIGCF  : 1; 
      __OM  u32 CMPOKCF    : 1; 
      __OM  u32 ARROKCF    : 1; 
      __OM  u32 UPCF       : 1; 
      __OM  u32 DOWNCF     : 1; 
            u32            : 25;
    } ICR_b;
  } ;
  
  union {
    __IOM u32 IER;              
    
    struct {
      __IOM u32 CMPMIE     : 1; 
      __IOM u32 ARRMIE     : 1; 
      __IOM u32 EXTTRIGIE  : 1; 
      __IOM u32 CMPOKIE    : 1; 
      __IOM u32 ARROKIE    : 1; 
      __IOM u32 UPIE       : 1; 
      __IOM u32 DOWNIE     : 1; 
            u32            : 25;
    } IER_b;
  } ;
  
  union {
    __IOM u32 CFGR;             
    
    struct {
      __IOM u32 CKSEL      : 1; 
      __IOM u32 CKPOL      : 2; 
      __IOM u32 CKFLT      : 2; 
            u32            : 1;
      __IOM u32 TRGFLT     : 2; 
            u32            : 1;
      __IOM u32 PRESC      : 3; 
            u32            : 1;
      __IOM u32 TRIGSEL    : 4; 
      __IOM u32 TRIGEN     : 2; 
      __IOM u32 TIMOUT     : 1; 
      __IOM u32 WAVE       : 1; 
      __IOM u32 WAVPOL     : 1; 
      __IOM u32 PRELOAD    : 1; 
      __IOM u32 COUNTMODE  : 1; 
      __IOM u32 ENC        : 1; 
            u32            : 7;
    } CFGR_b;
  } ;
  
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 ENABLE     : 1; 
      __IOM u32 SNGSTRT    : 1; 
      __IOM u32 CNTSTRT    : 1; 
      __IOM u32 COUNTRST   : 1; 
      __IOM u32 RSTARE     : 1; 
            u32            : 27;
    } CR_b;
  } ;
  
  union {
    __IOM u32 CMP;              
    
    struct {
      __IOM u32 CMP        : 16;
            u32            : 16;
    } CMP_b;
  } ;
  
  union {
    __IOM u32 ARR;              
    
    struct {
      __IOM u32 ARR        : 16;
            u32            : 16;
    } ARR_b;
  } ;
  
  union {
    __IM  u32 CNT;              
    
    struct {
      __IM  u32 CNT        : 16;
            u32            : 16;
    } CNT_b;
  } ;
  
  union {
    __IOM u32 OR;               
    
    struct {
      __IOM u32 IN1        : 1; 
      __IOM u32 IN2        : 1; 
      __IOM u32 IN1_2_1    : 2; 
      __IOM u32 IN2_2_1    : 2; 
            u32            : 26;
    } OR_b;
  } ;

} LPTIM_HW;


/*=========================== OPERATIONNAL AMPLFIER ================================*/

typedef struct {
  
  union {
    __IOM u32 CSR;
    
    struct {
      __IOM u32 OPAEN      : 1; 
      __IOM u32 FORCE_VP   : 1; 
      __IOM u32 VP_SEL     : 2; 
      __IOM u32 USERTRIM   : 1; 
      __IOM u32 VM_SEL     : 2; 
      __IOM u32 OPAHSM     : 1; 
      __IOM u32 OPAINTOEN  : 1; 
            u32            : 2;
      __IOM u32 CALON      : 1; 
      __IOM u32 CALSEL     : 2; 
      __IOM u32 PGA_GAIN   : 5; 
      __IOM u32 TRIMOFFSETP : 5;
      __IOM u32 TRIMOFFSETN : 5;
            u32            : 1;
      __IOM u32 CALOUT     : 1; 
      __IOM u32 LOCK       : 1; 
    } CSR_b;
  } ;

  __IOM u32 RESERVED[5];

  union {
    __IOM u32 TCMR;      
    
    struct {
      __IOM u32 VMS_SEL    : 1; 
      __IOM u32 VPS_SEL    : 2; 
      __IOM u32 T1CM_EN    : 1; 
      __IOM u32 T8CM_EN    : 1; 
      __IOM u32 T20CM_EN   : 1; 
            u32            : 25;
      __IOM u32 LOCK       : 1; 
    } TCMR_b;
  } ;

} OPAMP_HW;


/*=============================== POWER CONTROL ====================================*/

typedef struct{

  #if defined (STM32F303)
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 LPDS       : 1; 
      __IOM u32 PDDS       : 1; 
      __IOM u32 CWUF       : 1; 
      __IOM u32 CSBF       : 1; 
      __IOM u32 PVDE       : 1; 
      __IOM u32 PLS        : 3; 
      __IOM u32 DBP        : 1; 
            u32            : 23;
    } CR_b;
  } ;
  
  union {
    __IOM u32 CSR;              
    
    struct {
      __IM  u32 WUF        : 1; 
      __IM  u32 SBF        : 1; 
      __IM  u32 PVDO       : 1; 
            u32            : 5;
      __IOM u32 EWUP1      : 1; 
      __IOM u32 EWUP2      : 1; 
            u32            : 22;
    } CSR_b;
  } ;
  #elif defined (STM32G473)
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 LPDS       : 1; 
      __IOM u32 PDDS       : 1; 
      __IOM u32 CWUF       : 1; 
      __IOM u32 CSBF       : 1; 
      __IOM u32 PVDE       : 1; 
      __IOM u32 PLS        : 3; 
      __IOM u32 DBP        : 1;
      __IOM u32 FPDS       : 1;
      __IOM u32 LPLVDS     : 1;
      __IOM u32 MRLVDS     : 1;
            u32            : 1;
      __IOM u32 ADCDC1     : 1;
      __IOM u32 VOS        : 2;
            u32            : 4;
      __IOM u32 FMSSR      : 1;
      __IOM u32 FISSR      : 1;
            u32            : 10;
    } CR_b;
  } ;  
  __IO u32 CR2;  
  __IO u32 CR3;  
  __IO u32 CR4;  
  __IO u32 SR1;  
  __IO u32 SR2;  
  __IO u32 SCR;  
  u32 RESERVED;  
  __IO u32 PUCRA;
  __IO u32 PDCRA;
  __IO u32 PUCRB;
  __IO u32 PDCRB;
  __IO u32 PUCRC;
  __IO u32 PDCRC;
  __IO u32 PUCRD;
  __IO u32 PDCRD;
  __IO u32 PUCRE;
  __IO u32 PDCRE;
  __IO u32 PUCRF;
  __IO u32 PDCRF;
  __IO u32 PUCRG;
  __IO u32 PDCRG;
  u32 RESERVED1[10];
  __IO u32 CR5;  
  #endif    /* STM32F303 */

} PWR_HW;


/*========================== RESET AND CLOCK CONTROL ===============================*/

typedef struct {                     
  
  #if defined (STM32F303)
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 HSION      : 1; 
      __IM  u32 HSIRDY     : 1; 
            u32            : 1;
      __IOM u32 HSITRIM    : 5; 
      __IM  u32 HSICAL     : 8; 
      __IOM u32 HSEON      : 1; 
      __IM  u32 HSERDY     : 1; 
      __IOM u32 HSEBYP     : 1; 
      __IOM u32 CSSON      : 1; 
            u32            : 4;
      __IOM u32 PLLON      : 1; 
      __IM  u32 PLLRDY     : 1; 
            u32            : 6;
    } CR_b;
  } ;
  
  union {
    __IOM u32 CFGR;             
    
    struct {
      __IOM u32 SW         : 2; 
      __IM  u32 SWS        : 2; 
      __IOM u32 HPRE       : 4; 
      __IOM u32 PPRE1      : 3; 
      __IOM u32 PPRE2      : 3; 
            u32            : 1;
      __IOM u32 PLLSRC     : 2; 
      __IOM u32 PLLXTPRE   : 1; 
      __IOM u32 PLLMUL     : 4; 
      __IOM u32 USBPRES    : 1; 
      __IOM u32 I2SSRC     : 1; 
      __IOM u32 MCO        : 3; 
            u32            : 1;
      __IM  u32 MCOF       : 1; 
            u32            : 3;
    } CFGR_b;
  } ;
  
  union {
    __IOM u32 CIR;              
    
    struct {
      __IM  u32 LSIRDYF    : 1; 
      __IM  u32 LSERDYF    : 1; 
      __IM  u32 HSIRDYF    : 1; 
      __IM  u32 HSERDYF    : 1; 
      __IM  u32 PLLRDYF    : 1; 
            u32            : 2;
      __IM  u32 CSSF       : 1; 
      __IOM u32 LSIRDYIE   : 1; 
      __IOM u32 LSERDYIE   : 1; 
      __IOM u32 HSIRDYIE   : 1; 
      __IOM u32 HSERDYIE   : 1; 
      __IOM u32 PLLRDYIE   : 1; 
            u32            : 3;
      __OM  u32 LSIRDYC    : 1; 
      __OM  u32 LSERDYC    : 1; 
      __OM  u32 HSIRDYC    : 1; 
      __OM  u32 HSERDYC    : 1; 
      __OM  u32 PLLRDYC    : 1; 
            u32            : 2;
      __OM  u32 CSSC       : 1; 
            u32            : 8;
    } CIR_b;
  } ;
  
  union {
    __IOM u32 APB2RSTR;         
    
    struct {
      __IOM u32 SYSCFGRST  : 1; 
            u32            : 10;
      __IOM u32 TIM1RST    : 1; 
      __IOM u32 SPI1RST    : 1; 
      __IOM u32 TIM8RST    : 1; 
      __IOM u32 USART1RST  : 1; 
            u32            : 1;
      __IOM u32 TIM15RST   : 1; 
      __IOM u32 TIM16RST   : 1; 
      __IOM u32 TIM17RST   : 1; 
            u32            : 13;
    } APB2RSTR_b;
  } ;
  
  union {
    __IOM u32 APB1RSTR;         
    
    struct {
      __IOM u32 TIM2RST    : 1; 
      __IOM u32 TIM3RST    : 1; 
      __IOM u32 TIM4RST    : 1; 
            u32            : 1;
      __IOM u32 TIM6RST    : 1; 
      __IOM u32 TIM7RST    : 1; 
            u32            : 5;
      __IOM u32 WWDGRST    : 1; 
            u32            : 2;
      __IOM u32 SPI2RST    : 1; 
      __IOM u32 SPI3RST    : 1; 
            u32            : 1;
      __IOM u32 USART2RST  : 1; 
      __IOM u32 USART3RST  : 1; 
      __IOM u32 UART4RST   : 1; 
      __IOM u32 UART5RST   : 1; 
      __IOM u32 I2C1RST    : 1; 
      __IOM u32 I2C2RST    : 1; 
      __IOM u32 USBRST     : 1; 
            u32            : 1;
      __IOM u32 CANRST     : 1; 
            u32            : 2;
      __IOM u32 PWRRST     : 1; 
      __IOM u32 DACRST     : 1; 
      __IOM u32 I2C3RST    : 1; 
            u32            : 1;
    } APB1RSTR_b;
  } ;

  union {
    __IOM u32 AHB1ENR;           
    
    struct {
      __IOM u32 DMAEN      : 1; 
      __IOM u32 DMA2EN     : 1; 
      __IOM u32 SRAMEN     : 1; 
            u32            : 1;
      __IOM u32 FLITFEN    : 1; 
      __IOM u32 FMCEN      : 1; 
      __IOM u32 CRCEN      : 1; 
            u32            : 9;
      __IOM u32 GPIOHEN    : 1; 
      __IOM u32 GPIOAEN    : 1; 
      __IOM u32 GPIOBEN    : 1; 
      __IOM u32 GPIOCEN    : 1; 
      __IOM u32 GPIODEN    : 1; 
      __IOM u32 GPIOEEN    : 1; 
      __IOM u32 GPIOFEN    : 1; 
      __IOM u32 GPIOGEN    : 1; 
      __IOM u32 TSCEN      : 1; 
            u32            : 3;
      __IOM u32 ADC12EN    : 1; 
      __IOM u32 ADC34EN    : 1; 
            u32            : 2;
    } AHBENR_b;
  } ;
  
  union {
    __IOM u32 APB2ENR;          
    
    struct {
      __IOM u32 SYSCFGEN   : 1; 
            u32            : 10;
      __IOM u32 TIM1EN     : 1; 
      __IOM u32 SPI1EN     : 1; 
      __IOM u32 TIM8EN     : 1; 
      __IOM u32 USART1EN   : 1; 
            u32            : 1;
      __IOM u32 TIM15EN    : 1; 
      __IOM u32 TIM16EN    : 1; 
      __IOM u32 TIM17EN    : 1; 
            u32            : 13;
    } APB2ENR_b;
  } ;
  
  union {
    __IOM u32 APB1ENR;          
    
    struct {
      __IOM u32 TIM2EN     : 1; 
      __IOM u32 TIM3EN     : 1; 
      __IOM u32 TIM4EN     : 1; 
            u32            : 1;
      __IOM u32 TIM6EN     : 1; 
      __IOM u32 TIM7EN     : 1; 
            u32            : 5;
      __IOM u32 WWDGEN     : 1; 
            u32            : 2;
      __IOM u32 SPI2EN     : 1; 
      __IOM u32 SPI3EN     : 1; 
            u32            : 1;
      __IOM u32 USART2EN   : 1; 
      __IOM u32 USART3EN   : 1; 
      __IOM u32 USART4EN   : 1; 
      __IOM u32 USART5EN   : 1; 
      __IOM u32 I2C1EN     : 1; 
      __IOM u32 I2C2EN     : 1; 
      __IOM u32 USBEN      : 1; 
            u32            : 1;
      __IOM u32 CANEN      : 1; 
      __IOM u32 DAC2EN     : 1; 
            u32            : 1;
      __IOM u32 PWREN      : 1; 
      __IOM u32 DACEN      : 1; 
      __IOM u32 I2C3EN     : 1; 
            u32            : 1;
    } APB1ENR_b;
  } ;
  
  union {
    __IOM u32 BDCR;             
    
    struct {
      __IOM u32 LSEON      : 1; 
      __IM  u32 LSERDY     : 1; 
      __IOM u32 LSEBYP     : 1; 
      __IOM u32 LSEDRV     : 2; 
            u32            : 3;
      __IOM u32 RTCSEL     : 2; 
            u32            : 5;
      __IOM u32 RTCEN      : 1; 
      __IOM u32 BDRST      : 1; 
            u32            : 15;
    } BDCR_b;
  } ;
  
  union {
    __IOM u32 CSR;              
    
    struct {
      __IOM u32 LSION      : 1; 
      __IM  u32 LSIRDY     : 1; 
            u32            : 22;
      __IOM u32 RMVF       : 1; 
      __IOM u32 OBLRSTF    : 1; 
      __IOM u32 PINRSTF    : 1; 
      __IOM u32 PORRSTF    : 1; 
      __IOM u32 SFTRSTF    : 1; 
      __IOM u32 IWDGRSTF   : 1; 
      __IOM u32 WWDGRSTF   : 1; 
      __IOM u32 LPWRRSTF   : 1; 
    } CSR_b;
  } ;
  
  union {
    __IOM u32 AHB1RSTR;          
    
    struct {
            u32            : 5;
      __IOM u32 FMCRST     : 1; 
            u32            : 10;
      __IOM u32 GPIOHRST   : 1; 
      __IOM u32 GPIOARST   : 1; 
      __IOM u32 GPIOBRST   : 1; 
      __IOM u32 GPIOCRST   : 1; 
      __IOM u32 GPIODRST   : 1; 
      __IOM u32 GPIOERST   : 1; 
      __IOM u32 GPIOFRST   : 1; 
      __IOM u32 GPIOGRST   : 1; 
      __IOM u32 TSCRST     : 1; 
            u32            : 3;
      __IOM u32 ADC12RST   : 1; 
      __IOM u32 ADC34RST   : 1; 
            u32            : 2;
    } AHBRSTR_b;
  } ;
  
  union {
    __IOM u32 CFGR2;            
    
    struct {
      __IOM u32 PREDIV     : 4; 
      __IOM u32 ADC12PRES  : 5; 
      __IOM u32 ADC34PRES  : 5; 
            u32            : 18;
    } CFGR2_b;
  } ;
  
  union {
    __IOM u32 CFGR3;            
    
    struct {
      __IOM u32 USART1SW   : 2; 
            u32            : 2;
      __IOM u32 I2C1SW     : 1; 
      __IOM u32 I2C2SW     : 1; 
      __IOM u32 I2C3SW     : 1; 
            u32            : 1;
      __IOM u32 TIM1SW     : 1; 
      __IOM u32 TIM8SW     : 1; 
            u32            : 6;
      __IOM u32 USART2SW   : 2; 
      __IOM u32 USART3SW   : 2; 
      __IOM u32 UART4SW    : 2; 
      __IOM u32 UART5SW    : 2; 
            u32            : 8;
    } CFGR3_b;
  } ;

  #elif defined (STM32G473)
  union {
    __IOM u32 CR;
    
    struct {
            u32            : 8;
      __IOM u32 HSION      : 1;
      __IOM u32 HSIKERON   : 1;
      __IM  u32 HSIRDY     : 1;
            u32            : 5;
      __IOM u32 HSEON      : 1;
      __IM  u32 HSERDY     : 1;
      __IOM u32 HSEBYP     : 1;
      __IOM u32 CSSON      : 1; 
            u32            : 4;
      __IOM u32 PLLON      : 1; 
      __IM  u32 PLLRDY     : 1; 
            u32            : 6;
    } CR_b;
  } ;
  
  union {
    __IOM u32 ICSCR;
    
    struct {
            u32            : 16;
      __IM  u32 HSICAL     : 8;
      __IOM u32 HSITRIM    : 7;
            u32            : 1;
    } ICSCR_b;
  } ;
  
  union {
    __IOM u32 CFGR;
    
    struct {
      __IOM u32 SW         : 2; 
      __IM  u32 SWS        : 2; 
      __IOM u32 HPRE       : 4;
      __IOM u32 PPRE1      : 3;
      __IOM u32 PPRE2      : 3;
            u32            : 10;
      __IOM u32 MCOSEL     : 4; 
      __IOM u32 MCOPRE     : 3;
            u32            : 1;
    } CFGR_b;
  } ;
  

  union {
    __IOM u32 PLLCFGR; 
    
    struct {
      __IOM u32 PLLSRC     : 2;
            u32            : 2;
      __IOM u32 PLLM       : 4;
      __IOM u32 PLLN       : 7;
            u32            : 1;
      __IOM u32 PLLPEN     : 1; 
      __IOM u32 PLLP       : 1; 
            u32            : 2;
      __IOM u32 PLLQEN     : 1; 
      __IOM u32 PLLQ       : 2; 
            u32            : 1;
      __IOM u32 PLLREN     : 1;
      __IOM u32 PLLR       : 2; 
      __IOM u32 PLLPDIV    : 5;
    } PLLCFGR_b;
  } ;
  __IM  u32  RESERVED[2];
  
  union {
    __IOM u32 CIER;
    
    struct {
      __IOM u32 LSIRDYIE   : 1;  
      __IOM u32 LSERDYIE   : 1;
            u32            : 1;
      __IOM u32 HSIRDYIE   : 1;
      __IOM u32 HSERDYIE   : 1;
      __IOM u32 PLLRDYIE   : 1; 
            u32            : 3;
      __IOM u32 LSECSSIE   : 1;
      __IOM u32 HSI48RDYIE : 1;
            u32            : 21;
    } CIER_b;
  } ;
  
  union {
    __IOM u32 CIFR;
    
    struct {
      __IM  u32 LSIRDYF    : 1;
      __IM  u32 LSERDYF    : 1; 
            u32            : 1;
      __IM  u32 HSIRDYF    : 1;
      __IM  u32 HSERDYF    : 1;
      __IM  u32 PLLRDYF    : 1;
            u32            : 2;
      __IM  u32 CSSF       : 1;
      __IM  u32 LSECSSF    : 1; 
      __IM  u32 HSI48RDYF  : 1;
            u32            : 21;
    } CIFR_b;
  } ;
  
  union {
    __IOM u32 CICR;
    
    struct {
      __OM  u32 LSIRDYC    : 1;
      __OM  u32 LSERDYC    : 1;
            u32            : 1;
      __OM  u32 HSIRDYC    : 1;
      __OM  u32 HSERDYC    : 1;
      __OM  u32 PLLRDYC    : 1;
            u32            : 2;
      __OM  u32 CSSC       : 1;
      __OM  u32 LSECSSC    : 1;
      __OM  u32 HSI48RDYC  : 1;
            u32            : 21;
    } CICR_b;
  } ;
  __IM  u32  RESERVED1;
  
  union {
    __IOM u32 AHB1RSTR;
    
    struct {
      __IOM u32 DMA1RST    : 1;
      __IOM u32 DMA2RST    : 1; 
      __IOM u32 DMAMUX1RST : 1; 
      __IOM u32 CORDICRST  : 1; 
      __IOM u32 FMACRST    : 1; 
            u32            : 3;
      __IOM u32 FLASHRST   : 1; 
            u32            : 3;
      __IOM u32 CRCRST     : 1;  
            u32            : 19;
    } AHB1RSTR_b;
  } ;
  
  union {
    __IOM u32 AHB2RSTR;
    
    struct {
      __IOM u32 GPIOARST   : 1; 
      __IOM u32 GPIOBRST   : 1; 
      __IOM u32 GPIOCRST   : 1; 
      __IOM u32 GPIODRST   : 1; 
      __IOM u32 GPIOERST   : 1; 
      __IOM u32 GPIOFRST   : 1; 
      __IOM u32 GPIOGRST   : 1; 
            u32            : 6;
      __IOM u32 ADC12RST   : 1; 
      __IOM u32 ADC345RST  : 1; 
            u32            : 1;
      __IOM u32 DAC1RST    : 1; 
      __IOM u32 DAC2RST    : 1; 
      __IOM u32 DAC3RST    : 1; 
      __IOM u32 DAC4RST    : 1; 
            u32            : 4;
      __IOM u32 AESRST     : 1; 
            u32            : 1;
      __IOM u32 RNGRST     : 1; 
            u32            : 5;
    } AHB2RSTR_b;
  } ;
  
  union {
    __IOM u32 AHB3RSTR;     
    
    struct {
      __IOM u32 FMCRST     : 1;
                      
            u32            : 7;
      __IOM u32 QSPIRST    : 1; 
            u32            : 23;
    } AHB3RSTR_b;
  } ;
  __IM  u32  RESERVED2;
  
  union {
    __IOM u32 APB1RSTR1;    
    
    struct {
      __IOM u32 TIM2RST    : 1; 
      __IOM u32 TIM3RST    : 1; 
      __IOM u32 TIM4RST    : 1; 
      __IOM u32 TIM5RST    : 1; 
      __IOM u32 TIM6RST    : 1; 
      __IOM u32 TIM7RST    : 1; 
            u32            : 2;
      __IOM u32 CRSRST     : 1; 
            u32            : 5;
      __IOM u32 SPI2RST    : 1; 
      __IOM u32 SPI3RST    : 1; 
            u32            : 1;
      __IOM u32 USART2RST  : 1; 
      __IOM u32 USART3RST  : 1; 
      __IOM u32 UART4RST   : 1; 
      __IOM u32 UART5RST   : 1; 
      __IOM u32 I2C1RST    : 1; 
      __IOM u32 I2C2RST    : 1; 
      __IOM u32 USBRST     : 1; 
            u32            : 1;
      __IOM u32 FDCANRST   : 1; 
            u32            : 2;
      __IOM u32 PWRRST     : 1; 
            u32            : 1;
      __IOM u32 I2C3RST    : 1; 
      __IOM u32 LPTIM1RST  : 1; 
    } APB1RSTR1_b;
  } ;
  
  union {
    __IOM u32 APB1RSTR2;    
    
    struct {
      __IOM u32 LPUART1RST : 1; 
      __IOM u32 I2C4RST    : 1; 
            u32            : 6;
      __IOM u32 UCPD1RST   : 1; 
            u32            : 23;
    } APB1RSTR2_b;
  } ;
  
  union {
    __IOM u32 APB2RSTR;     
    
    struct {
      __IOM u32 SYSCFGRST  : 1; 
            u32            : 10;
      __IOM u32 TIM1RST    : 1; 
      __IOM u32 SPI1RST    : 1; 
      __IOM u32 TIM8RST    : 1; 
      __IOM u32 USART1RST  : 1; 
      __IOM u32 SPI4RST    : 1; 
      __IOM u32 TIM15RST   : 1; 
      __IOM u32 TIM16RST   : 1; 
      __IOM u32 TIM17RST   : 1; 
            u32            : 1;
      __IOM u32 TIM20RST   : 1; 
      __IOM u32 SAI1RST    : 1;
                      
            u32            : 4;
      __IOM u32 HRTIM1RST  : 1; 
            u32            : 5;
    } APB2RSTR_b;
  } ;
  __IM  u32  RESERVED3;
  
  union {
    __IOM u32 AHB1ENR;
    
    struct {
      __IOM u32 DMA1EN     : 1;  
      __IOM u32 DMA2EN     : 1;  
      __IOM u32 DMAMUX1EN  : 1;  
      __IOM u32 CORDICEN   : 1;  
      __IOM u32 FMACEN     : 1;  
            u32            : 3;
      __IOM u32 FLASHEN    : 1;
                        
            u32            : 3;
      __IOM u32 CRCEN      : 1;  
            u32            : 19;
    } AHB1ENR_b;
  } ;
  
  union {
    __IOM u32 AHB2ENR;       
    
    struct {
      __IOM u32 GPIOAEN    : 1;  
      __IOM u32 GPIOBEN    : 1;  
      __IOM u32 GPIOCEN    : 1;  
      __IOM u32 GPIODEN    : 1;  
      __IOM u32 GPIOEEN    : 1;  
      __IOM u32 GPIOFEN    : 1;  
      __IOM u32 GPIOGEN    : 1;  
            u32            : 6;
      __IOM u32 ADC12EN    : 1;  
      __IOM u32 ADC345EN   : 1;  
            u32            : 1;
      __IOM u32 DAC1EN     : 1;  
      __IOM u32 DAC2EN     : 1;  
      __IOM u32 DAC3EN     : 1;  
      __IOM u32 DAC4EN     : 1;  
            u32            : 4;
      __IOM u32 AESEN      : 1;  
            u32            : 1;
      __IOM u32 RNGEN      : 1;  
            u32            : 5;
    } AHB2ENR_b;
  } ;
  
  union {
    __IOM u32 AHB3ENR;       
    
    struct {
      __IOM u32 FMCEN      : 1;
                        
            u32            : 7;
      __IOM u32 QSPIEN     : 1;
                        
            u32            : 23;
    } AHB3ENR_b;
  } ;
  __IM  u32  RESERVED4;
  
  union {
    __IOM u32 APB1ENR;      
    
    struct {
      __IOM u32 TIM2EN     : 1;  
      __IOM u32 TIM3EN     : 1;  
      __IOM u32 TIM4EN     : 1;  
      __IOM u32 TIM5EN     : 1;  
      __IOM u32 TIM6EN     : 1;  
      __IOM u32 TIM7EN     : 1;  
            u32            : 2;
      __IOM u32 CRSEN      : 1;  
            u32            : 1;
      __IOM u32 RTCAPBEN   : 1;  
      __IOM u32 WWDGEN     : 1;   
            u32            : 2;
      __IOM u32 SPI2EN     : 1;  
      __IOM u32 SPI3EN     : 1;  
            u32            : 1;
      __IOM u32 USART2EN   : 1;  
      __IOM u32 USART3EN   : 1;  
      __IOM u32 UART4EN    : 1;  
      __IOM u32 UART5EN    : 1;  
      __IOM u32 I2C1EN     : 1;  
      __IOM u32 I2C2EN     : 1;  
      __IOM u32 USBEN      : 1;  
            u32            : 1;
      __IOM u32 FDCANEN    : 1;  
            u32            : 2;
      __IOM u32 PWREN      : 1;  
            u32            : 1;
      __IOM u32 I2C3EN     : 1;  
      __IOM u32 LPTIM1EN   : 1;  
    } APB1ENR_b;
  } ;
  
  union {
    __IOM u32 APB1ENR2;      
    
    struct {
      __IOM u32 LPUART1EN  : 1;  
      __IOM u32 I2C4EN     : 1;  
            u32            : 6;
      __IOM u32 UCPD1EN    : 1;  
            u32            : 23;
    } APB1ENR2_b;
  } ;
  
  union {
    __IOM u32 APB2ENR;       
    
    struct {
      __IOM u32 SYSCFGEN   : 1;
                        
            u32            : 10;
      __IOM u32 TIM1EN     : 1;  
      __IOM u32 SPI1EN     : 1;  
      __IOM u32 TIM8EN     : 1;  
      __IOM u32 USART1EN   : 1;  
      __IOM u32 SPI4EN     : 1;  
      __IOM u32 TIM15EN    : 1;  
      __IOM u32 TIM16EN    : 1;  
      __IOM u32 TIM17EN    : 1;  
            u32            : 1;
      __IOM u32 TIM20EN    : 1;  
      __IOM u32 SAI1EN     : 1;  
            u32            : 4;
      __IOM u32 HRTIM1EN   : 1;  
            u32            : 5;
    } APB2ENR_b;
  } ;
  __IM  u32  RESERVED5;
  
  union {
    __IOM u32 AHB1SMENR;
        
    
    struct {
      __IOM u32 DMA1SMEN   : 1;  
                        
      __IOM u32 DMA2SMEN   : 1;  
                        
      __IOM u32 DMAMUX1SMEN : 1;
                        
      __IOM u32 CORDICSMEN : 1;  
      __IOM u32 FMACSMEN   : 1;  
            u32            : 3;
      __IOM u32 FLASHSMEN  : 1;  
                        
      __IOM u32 SRAM1SMEN  : 1;          
                        
            u32            : 2;
      __IOM u32 CRCSMEN    : 1;   
                        
            u32            : 19;
    } AHB1SMENR_b;
  } ;
  
  union {
    __IOM u32 AHB2SMENR;             
        
    
    struct {
      __IOM u32 GPIOASMEN  : 1;
      __IOM u32 GPIOBSMEN  : 1;
      __IOM u32 GPIOCSMEN  : 1;  
      __IOM u32 GPIODSMEN  : 1;
      __IOM u32 GPIOESMEN  : 1;
      __IOM u32 GPIOFSMEN  : 1;
      __IOM u32 GPIOGSMEN  : 1;
            u32            : 2;
      __IOM u32 CCMSRAMSMEN : 1;
      __IOM u32 SRAM2SMEN  : 1; 
            u32            : 2;
      __IOM u32 ADC12SMEN  : 1;
      __IOM u32 ADC345SMEN : 1; 
            u32            : 1;
      __IOM u32 DAC1SMEN   : 1;
      __IOM u32 DAC2SMEN   : 1;
      __IOM u32 DAC3SMEN   : 1;
      __IOM u32 DAC4SMEN   : 1;
            u32            : 4;
      __IOM u32 AESSMEN    : 1;
            u32            : 1;
      __IOM u32 RNGEN      : 1;
            u32            : 5;
    } AHB2SMENR_b;
  } ;
  
  union {
    __IOM u32 AHB3SMENR; 
    
    struct {
      __IOM u32 FMCSMEN    : 1;
            u32            : 7;
      __IOM u32 QSPISMEN   : 1;
            u32            : 23;
    } AHB3SMENR_b;
  } ;
  __IM  u32  RESERVED6;
  
  union {
    __IOM u32 APB1SMENR1;
    
    struct {
      __IOM u32 TIM2SMEN   : 1;
      __IOM u32 TIM3SMEN   : 1;
      __IOM u32 TIM4SMEN   : 1;
      __IOM u32 TIM5SMEN   : 1; 
      __IOM u32 TIM6SMEN   : 1;
      __IOM u32 TIM7SMEN   : 1;
            u32            : 2;
      __IOM u32 CRSSMEN    : 1; 
            u32            : 1;
      __IOM u32 RTCAPBSMEN : 1;
      __IOM u32 WWDGSMEN   : 1;
            u32            : 2;
      __IOM u32 SPI2SMEN   : 1; 
      __IOM u32 SPI3SMEN   : 1;
            u32            : 1;
      __IOM u32 USART2SMEN : 1; 
      __IOM u32 USART3SMEN : 1; 
      __IOM u32 UART4SMEN  : 1;  
      __IOM u32 UART5SMEN  : 1; 
      __IOM u32 I2C1SMEN   : 1;
      __IOM u32 I2C2SMEN   : 1; 
      __IOM u32 USBSMEN    : 1; 
            u32            : 1;
      __IOM u32 FDCANSMEN  : 1;
            u32            : 2;
      __IOM u32 PWRSMEN    : 1; 
            u32            : 1;
      __IOM u32 I2C3SMEN   : 1;
      __IOM u32 LPTIM1SMEN : 1; 
    } APB1SMENR1_b;
  } ;
  
  union {
    __IOM u32 APB1SMENR2;
    
    struct {
      __IOM u32 LPUART1SMEN : 1;
      __IOM u32 I2C4SMEN   : 1; 
            u32            : 6;
      __IOM u32 UCPD1SMEN  : 1; 
            u32            : 23;
    } APB1SMENR2_b;
  } ;
  
  union {
    __IOM u32 APB2SMENR; 
    
    struct {
      __IOM u32 SYSCFGSMEN : 1;
            u32            : 10;
      __IOM u32 TIM1SMEN   : 1;
      __IOM u32 SPI1SMEN   : 1; 
      __IOM u32 TIM8SMEN   : 1;
      __IOM u32 USART1SMEN : 1; 
      __IOM u32 SPI4SMEN   : 1;
      __IOM u32 TIM15SMEN  : 1;
      __IOM u32 TIM16SMEN  : 1;
      __IOM u32 TIM17SMEN  : 1;
            u32            : 1;
      __IOM u32 TIM20SMEN  : 1;
      __IOM u32 SAI1SMEN   : 1;  
            u32            : 4;
      __IOM u32 HRTIM1SMEN : 1;
            u32            : 5;
    } APB2SMENR_b;
  } ;
  __IM  u32  RESERVED7;
  
  union {
    __IOM u32 CCIPR;
    
    struct {
      __IOM u32 USART1SEL  : 2;
      __IOM u32 USART2SEL  : 2;
      __IOM u32 USART3SEL  : 2; 
      __IOM u32 UART4SEL   : 2; 
      __IOM u32 UART5SEL   : 2;
      __IOM u32 LPUART1SEL : 2; 
      __IOM u32 I2C1SEL    : 2; 
      __IOM u32 I2C2SEL    : 2;
      __IOM u32 I2C3SEL    : 2; 
      __IOM u32 LPTIM1SEL  : 2; 
      __IOM u32 SAI1SEL    : 2; 
      __IOM u32 I2S23SEL   : 2; 
      __IOM u32 FDCANSEL   : 2;  
      __IOM u32 CLK48SEL   : 2; 
      __IOM u32 ADC12SEL   : 2; 
      __IOM u32 ADC345SEL  : 2;  
    } CCIPR_b;
  } ;
  __IM  u32  RESERVED8;
  
  union {
    __IOM u32 BDCR;
    
    struct {
      __IOM u32 LSEON      : 1;
      __IM  u32 LSERDY     : 1; 
      __IOM u32 LSEBYP     : 1;  
      __IOM u32 LSEDRV     : 2; 
      __IOM u32 LSECSSON   : 1; 
      __IM  u32 LSECSSD    : 1; 
            u32            : 1;
      __IOM u32 RTCSEL     : 2; 
            u32            : 5;
      __IOM u32 RTCEN      : 1; 
      __IOM u32 BDRST      : 1; 
            u32            : 7;
      __IOM u32 LSCOEN     : 1; 
      __IOM u32 LSCOSEL    : 1;
            u32            : 6;
    } BDCR_b;
  } ;
  
  union {
    __IOM u32 CSR;
    
    struct {
      __IOM u32 LSION      : 1;
      __IM  u32 LSIRDY     : 1; 
            u32            : 21;
      __IOM u32 RMVF       : 1;  
            u32            : 1;
      __IM  u32 OBLRSTF    : 1; 
      __IM  u32 PINRSTF    : 1;
      __IM  u32 BORRSTF    : 1; 
      __IM  u32 SFTRSTF    : 1; 
      __IM  u32 IWDGRSTF   : 1; 
      __IM  u32 WWDGRSTF   : 1;
      __IM  u32 LPWRRSTF   : 1;
    } CSR_b;
  } ;
  
  union {
    __IOM u32 CRRCR; 
    
    struct {
      __IOM u32 HSI48ON    : 1;
      __IM  u32 HSI48RDY   : 1; 
            u32            : 5;
      __IM  u32 HSI48CAL   : 9;
            u32            : 16;
    } CRRCR_b;
  } ;
  
  union {
    __IOM u32 CCIPR2; 
    
    struct {
      __IOM u32 I2C4SEL    : 2;  
            u32            : 18;
      __IOM u32 QSPISEL    : 2; 
            u32            : 10;
    } CCIPR2_b;
  } ;
  #endif    /* STM32F303 */

} RCC_HW;


/*============================== REAL TIME CLOCK ===================================*/

typedef struct
{
  #if defined (STM32F303)
  __IO u32 TR;      
  __IO u32 DR;      
  __IO u32 CR;      
  __IO u32 ISR;     
  __IO u32 PRER;    
  __IO u32 WUTR;    
  u32 RESERVED0;    
  __IO u32 ALRMAR;  
  __IO u32 ALRMBR;  
  __IO u32 WPR;     
  __IO u32 SSR;     
  __IO u32 SHIFTR;  
  __IO u32 TSTR;    
  __IO u32 TSDR;    
  __IO u32 TSSSR;   
  __IO u32 CALR;    
  __IO u32 TAFCR;   
  __IO u32 ALRMASSR;
  __IO u32 ALRMBSSR;
  u32 RESERVED7;    
  __IO u32 BKP0R;   
  __IO u32 BKP1R;   
  __IO u32 BKP2R;   
  __IO u32 BKP3R;   
  __IO u32 BKP4R;   
  __IO u32 BKP5R;   
  __IO u32 BKP6R;   
  __IO u32 BKP7R;   
  __IO u32 BKP8R;   
  __IO u32 BKP9R;   
  __IO u32 BKP10R;  
  __IO u32 BKP11R;  
  __IO u32 BKP12R;  
  __IO u32 BKP13R;  
  __IO u32 BKP14R;  
  __IO u32 BKP15R;

#elif defined (STM32G473)
  __IO u32 TR;       
  __IO u32 DR;       
  __IO u32 SSR;      
  __IO u32 ICSR;     
  __IO u32 PRER;     
  __IO u32 WUTR;     
  __IO u32 CR;       
       u32 RESERVED0;
       u32 RESERVED1;
  __IO u32 WPR;      
  __IO u32 CALR;     
  __IO u32 SHIFTR;   
  __IO u32 TSTR;     
  __IO u32 TSDR;     
  __IO u32 TSSSR;    
       u32 RESERVED2;
  __IO u32 ALRMAR;   
  __IO u32 ALRMASSR; 
  __IO u32 ALRMBR;   
  __IO u32 ALRMBSSR; 
  __IO u32 SR;       
  __IO u32 MISR;     
       u32 RESERVED3;
  __IO u32 SCR;
  #endif    /* STM32F303 */     

} RTC_HW;


/*====================== TAMPER AND BACKUP REGISTERS ===============================*/

typedef struct
{
  __IO u32 CR1;      
  __IO u32 CR2;      
       u32 RESERVED0;
  __IO u32 FLTCR;    
       u32 RESERVED1[6];    
       u32 RESERVED2;
  __IO u32 IER;      
  __IO u32 SR;       
  __IO u32 MISR;     
       u32 RESERVED3;
  __IO u32 SCR;      
       u32 RESERVED4[48];    
  __IO u32 BKP0R;     
  __IO u32 BKP1R;     
  __IO u32 BKP2R;     
  __IO u32 BKP3R;     
  __IO u32 BKP4R;     
  __IO u32 BKP5R;     
  __IO u32 BKP6R;     
  __IO u32 BKP7R;     
  __IO u32 BKP8R;     
  __IO u32 BKP9R;     
  __IO u32 BKP10R;    
  __IO u32 BKP11R;    
  __IO u32 BKP12R;    
  __IO u32 BKP13R;    
  __IO u32 BKP14R;    
  __IO u32 BKP15R;    
  __IO u32 BKP16R;    
  __IO u32 BKP17R;    
  __IO u32 BKP18R;    
  __IO u32 BKP19R;    
  __IO u32 BKP20R;    
  __IO u32 BKP21R;    
  __IO u32 BKP22R;    
  __IO u32 BKP23R;    
  __IO u32 BKP24R;    
  __IO u32 BKP25R;    
  __IO u32 BKP26R;    
  __IO u32 BKP27R;    
  __IO u32 BKP28R;    
  __IO u32 BKP29R;    
  __IO u32 BKP30R;    
  __IO u32 BKP31R;

} TAMP_HW;


/*====================== SERIAL PERIPHERAL INTERFACE ===============================*/

typedef struct {                      
  
  union {
    __IOM u32 CR1;               
    
    struct {
      __IOM u32 CPHA       : 1;  
      __IOM u32 CPOL       : 1;  
      __IOM u32 MSTR       : 1;  
      __IOM u32 BR         : 3;  
      __IOM u32 SPE        : 1;  
      __IOM u32 LSBFIRST   : 1;  
      __IOM u32 SSI        : 1;  
      __IOM u32 SSM        : 1;  
      __IOM u32 RXONLY     : 1;  
      __IOM u32 DFF        : 1;  
      __IOM u32 CRCNEXT    : 1;  
      __IOM u32 CRCEN      : 1;  
      __IOM u32 BIDIOE     : 1;  
      __IOM u32 BIDIMODE   : 1;  
            u32            : 16;
    } CR1_b;
  } ;
  
  union {
    __IOM u32 CR2;               
    
    struct {
      __IOM u32 RXDMAEN    : 1;  
      __IOM u32 TXDMAEN    : 1;  
      __IOM u32 SSOE       : 1;  
      __IOM u32 NSSP       : 1;  
      __IOM u32 FRF        : 1;  
      __IOM u32 ERRIE      : 1;  
      __IOM u32 RXNEIE     : 1;  
      __IOM u32 TXEIE      : 1;  
      __IOM u32 DS         : 4;  
      __IOM u32 FRXTH      : 1;  
      __IOM u32 LDMA_RX    : 1;  
      __IOM u32 LDMA_TX    : 1;  
            u32            : 17;
    } CR2_b;
  } ;
  
  union {
    __IOM u32 SR;                
    
    struct {
      __IM  u32 RXNE       : 1;  
      __IM  u32 TXE        : 1;  
            u32            : 2;
      __IOM u32 CRCERR     : 1;  
      __IM  u32 MODF       : 1;  
      __IM  u32 OVR        : 1;  
      __IM  u32 BSY        : 1;  
      __IM  u32 TIFRFE     : 1;  
      __IM  u32 FRLVL      : 2;  
      __IM  u32 FTLVL      : 2;  
            u32            : 19;
    } SR_b;
  } ;
  
  union {
    __IOM u32 DR;                
    
    struct {
      __IOM u32 DR         : 16; 
            u32            : 16;
    } DR_b;
  } ;
  
  union {
    __IOM u32 CRCPR;             
    
    struct {
      __IOM u32 CRCPOLY    : 16; 
            u32            : 16;
    } CRCPR_b;
  } ;
  
  union {
    __IM  u32 RXCRCR;            
    
    struct {
      __IM  u32 RxCRC      : 16; 
            u32            : 16;
    } RXCRCR_b;
  } ;
  
  union {
    __IM  u32 TXCRCR;            
    
    struct {
      __IM  u32 TxCRC      : 16; 
            u32            : 16;
    } TXCRCR_b;
  } ;
  
  union {
    __IOM u32 I2SCFGR;           
    
    struct {
      __IOM u32 CHLEN      : 1;  
      __IOM u32 DATLEN     : 2;  
      __IOM u32 CKPOL      : 1;  
      __IOM u32 I2SSTD     : 2;  
            u32            : 1;
      __IOM u32 PCMSYNC    : 1;  
      __IOM u32 I2SCFG     : 2;  
      __IOM u32 I2SE       : 1;  
      __IOM u32 I2SMOD     : 1;  
            u32            : 20;
    } I2SCFGR_b;
  } ;
  
  union {
    __IOM u32 I2SPR;             
    
    struct {
      __IOM u32 I2SDIV     : 8;  
      __IOM u32 ODD        : 1;  
      __IOM u32 MCKOE      : 1;  
            u32            : 22;
    } I2SPR_b;
  } ;

} SPI_HW;


/*===================== SYSTEM CONFIGURATION CONTROLLER ============================*/

typedef struct {                      
  
  #if defined (STM32F303)
  union {
    __IOM u32 CFGR1;           
    
    struct {
      __IOM u32 MEM_MODE   : 2;       
            u32            : 3;
      __IOM u32 USB_IT_RMP : 1;       
      __IOM u32 TIM1_ITR_RMP : 1;     
      __IOM u32 DAC_TRIG_RMP : 1;     
      __IOM u32 ADC24_DMA_RMP : 1;    
            u32            : 2;
      __IOM u32 TIM16_DMA_RMP : 1;    
      __IOM u32 TIM17_DMA_RMP : 1;    
      __IOM u32 TIM6_DAC1_DMA_RMP : 1;
      __IOM u32 TIM7_DAC2_DMA_RMP : 1;
            u32            : 1;
      __IOM u32 I2C_PB6_FM : 1; 
                                  
      __IOM u32 I2C_PB7_FM : 1; 
                                  
      __IOM u32 I2C_PB8_FM : 1;
                                  
      __IOM u32 I2C_PB9_FM : 1; 
                                  
      __IOM u32 I2C1_FM    : 1;       
      __IOM u32 I2C2_FM    : 1;       
      __IOM u32 ENCODER_MODE : 2;     
            u32            : 2;
      __IOM u32 FPU_IT     : 6;       
    } CFGR1_b;
  } ;
  
  union {
    __IOM u32 RCR;             
    
    struct {
      __IOM u32 PAGE0_WP   : 1;       
      __IOM u32 PAGE1_WP   : 1;       
      __IOM u32 PAGE2_WP   : 1;       
      __IOM u32 PAGE3_WP   : 1;       
      __IOM u32 PAGE4_WP   : 1;       
      __IOM u32 PAGE5_WP   : 1;       
      __IOM u32 PAGE6_WP   : 1;       
      __IOM u32 PAGE7_WP   : 1;       
            u32            : 24;
    } RCR_b;
  } ;
  
  union {
    __IOM u32 EXTICR1;         
    
    struct {
      __IOM u32 EXTI0      : 4;       
      __IOM u32 EXTI1      : 4;       
      __IOM u32 EXTI2      : 4;       
      __IOM u32 EXTI3      : 4;       
            u32            : 16;
    } EXTICR1_b;
  } ;
  
  union {
    __IOM u32 EXTICR2;         
    
    struct {
      __IOM u32 EXTI4      : 4;       
      __IOM u32 EXTI5      : 4;       
      __IOM u32 EXTI6      : 4;       
      __IOM u32 EXTI7      : 4;       
            u32            : 16;
    } EXTICR2_b;
  } ;
  
  union {
    __IOM u32 EXTICR3;         
    
    struct {
      __IOM u32 EXTI8      : 4;       
      __IOM u32 EXTI9      : 4;       
      __IOM u32 EXTI10     : 4;       
      __IOM u32 EXTI11     : 4;       
            u32            : 16;
    } EXTICR3_b;
  } ;
  
  union {
    __IOM u32 EXTICR4;         
    
    struct {
      __IOM u32 EXTI12     : 4;       
      __IOM u32 EXTI13     : 4;       
      __IOM u32 EXTI14     : 4;       
      __IOM u32 EXTI15     : 4;       
            u32            : 16;
    } EXTICR4_b;
  } ;
  
  union {
    __IOM u32 CFGR2;           
    
    struct {
      __IOM u32 LOCUP_LOCK : 1;       
      __IOM u32 SRAM_PARITY_LOCK : 1; 
      __IOM u32 PVD_LOCK   : 1;       
            u32            : 1;
      __IOM u32 BYP_ADD_PAR : 1;      
            u32            : 3;
      __IOM u32 SRAM_PEF   : 1;       
            u32            : 23;
    } CFGR2_b;
  } ;

  #elif defined (STM32G473)
  union {
    __IOM u32 MEMRMP;            
    
    struct {
      __IOM u32 MEM_MODE   : 3;  
            u32            : 5;
      __IOM u32 FB_mode    : 1;  
            u32            : 23;
    } MEMRMP_b;
  } ;
  
  union {
    __IOM u32 CFGR1;             
    
    struct {
            u32            : 8;
      __IOM u32 BOOSTEN    : 1;  
      __IOM u32 ANASWVDD   : 1;  
            u32            : 6;
      __IOM u32 I2C_PB6_FMP : 1; 
      __IOM u32 I2C_PB7_FMP : 1; 
      __IOM u32 I2C_PB8_FMP : 1; 
      __IOM u32 I2C_PB9_FMP : 1; 
      __IOM u32 I2C1_FMP   : 1;  
      __IOM u32 I2C2_FMP   : 1;  
      __IOM u32 I2C3_FMP   : 1;  
      __IOM u32 I2C4_FMP   : 1;  
            u32            : 2;
      __IOM u32 FPU_IE     : 6;  
    } CFGR1_b;
  } ;
  
  union {
    __IOM u32 EXTICR1;           
    
    struct {
      __IOM u32 EXTI0      : 4;  
      __IOM u32 EXTI1      : 4;  
      __IOM u32 EXTI2      : 4;  
      __IOM u32 EXTI3      : 4;  
            u32            : 16;
    } EXTICR1_b;
  } ;
  
  union {
    __IOM u32 EXTICR2;           
    
    struct {
      __IOM u32 EXTI4      : 4;  
      __IOM u32 EXTI5      : 4;  
      __IOM u32 EXTI6      : 4;  
      __IOM u32 EXTI7      : 4;  
            u32            : 16;
    } EXTICR2_b;
  } ;
  
  union {
    __IOM u32 EXTICR3;           
    
    struct {
      __IOM u32 EXTI8      : 4;  
      __IOM u32 EXTI9      : 4;  
      __IOM u32 EXTI10     : 4;  
      __IOM u32 EXTI11     : 4;  
            u32            : 16;
    } EXTICR3_b;
  } ;
  
  union {
    __IOM u32 EXTICR4;           
    
    struct {
      __IOM u32 EXTI12     : 4;  
      __IOM u32 EXTI13     : 4;  
      __IOM u32 EXTI14     : 4;  
      __IOM u32 EXTI15     : 4;  
            u32            : 16;
    } EXTICR4_b;
  } ;
  
  union {
    __IOM u32 SCSR;              
    
    struct {
      __IOM u32 CCMER      : 1;  
      __IM  u32 CCMBSY     : 1;  
            u32            : 30;
    } SCSR_b;
  } ;
  
  union {
    __IOM u32 CFGR2;             
    
    struct {
      __IOM u32 CLL        : 1;  
      __IOM u32 SPL        : 1;  
      __IOM u32 PVDL       : 1;  
      __IOM u32 ECCL       : 1;  
            u32            : 4;
      __IOM u32 SPF        : 1;  
            u32            : 23;
    } CFGR2_b;
  } ;
  
  union {
    __IOM u32 SWPR;              
    
    struct {
      __IOM u32 Page0_WP   : 1;  
      __IOM u32 Page1_WP   : 1;  
      __IOM u32 Page2_WP   : 1;  
      __IOM u32 Page3_WP   : 1;  
      __IOM u32 Page4_WP   : 1;  
      __IOM u32 Page5_WP   : 1;  
      __IOM u32 Page6_WP   : 1;  
      __IOM u32 Page7_WP   : 1;  
      __IOM u32 Page8_WP   : 1;  
      __IOM u32 Page9_WP   : 1;  
      __IOM u32 Page10_WP  : 1;  
      __IOM u32 Page11_WP  : 1;  
      __IOM u32 Page12_WP  : 1;  
      __IOM u32 Page13_WP  : 1;  
      __IOM u32 Page14_WP  : 1;  
      __IOM u32 Page15_WP  : 1;  
      __IOM u32 Page16_WP  : 1;  
      __IOM u32 Page17_WP  : 1;  
      __IOM u32 Page18_WP  : 1;  
      __IOM u32 Page19_WP  : 1;  
      __IOM u32 Page20_WP  : 1;  
      __IOM u32 Page21_WP  : 1;  
      __IOM u32 Page22_WP  : 1;  
      __IOM u32 Page23_WP  : 1;  
      __IOM u32 Page24_WP  : 1;  
      __IOM u32 Page25_WP  : 1;  
      __IOM u32 Page26_WP  : 1;  
      __IOM u32 Page27_WP  : 1;  
      __IOM u32 Page28_WP  : 1;  
      __IOM u32 Page29_WP  : 1;  
      __IOM u32 Page30_WP  : 1;  
      __IOM u32 Page31_WP  : 1;  
    } SWPR_b;
  } ;
  
  union {
    __OM  u32 SKR;               
    
    struct {
      __OM  u32 KEY        : 8;  
            u32            : 24;
    } SKR_b;
  } ;
  #endif    /* STM32F303 */

} SYSCFG_HW;


/*============================= TIMER REGISTERS ====================================*/

typedef struct {                      
  
  union {
    __IOM u32 CR1;               
    
    struct {
      __IOM u32 CEN        : 1;  
      __IOM u32 UDIS       : 1;  
      __IOM u32 URS        : 1;  
      __IOM u32 OPM        : 1;  
      __IOM u32 DIR        : 1;  
      __IOM u32 CMS        : 2;  
      __IOM u32 ARPE       : 1;  
      __IOM u32 CKD        : 2;  
            u32            : 1;
      __IOM u32 UIFREMAP   : 1;  
      __IOM u32 DITHEN     : 1;  
            u32            : 19;
    } CR1_b;
  } ;
  
  union {
    __IOM u32 CR2;               
    
    struct {
      __IOM u32 CCPC       : 1;  
            u32            : 1;
      __IOM u32 CCUS       : 1;  
      __IOM u32 CCDS       : 1;  
      __IOM u32 MMS        : 3;  
      __IOM u32 TI1S       : 1;  
      __IOM u32 OIS1       : 1;  
      __IOM u32 OIS1N      : 1;  
      __IOM u32 OIS2       : 1;  
      __IOM u32 OIS2N      : 1;  
      __IOM u32 OIS3       : 1;  
      __IOM u32 OIS3N      : 1;  
      __IOM u32 OIS4       : 1;  
      __IOM u32 OIS4N      : 1;  
      __IOM u32 OIS5       : 1;  
            u32            : 1;
      __IOM u32 OIS6       : 1;  
            u32            : 1;
      __IOM u32 MMS2       : 4;  
            u32            : 1;
      __IOM u32 MMS_3      : 1;  
            u32            : 6;
    } CR2_b;
  } ;
  
  union {
    __IOM u32 SMCR;              
    
    struct {
      __IOM u32 SMS        : 3;  
      __IOM u32 OCCS       : 1;  
      __IOM u32 TS         : 3;  
      __IOM u32 MSM        : 1;  
      __IOM u32 ETF        : 4;  
      __IOM u32 ETPS       : 2;  
      __IOM u32 ECE        : 1;  
      __IOM u32 ETP        : 1;  
      __IOM u32 SMS_3      : 1;  
            u32            : 3;
      __IOM u32 TS_4_3     : 2;  
            u32            : 2;
      __IOM u32 SMSPE      : 1;  
      __IOM u32 SMSPS      : 1;  
            u32            : 6;
    } SMCR_b;
  } ;
  
  union {
    __IOM u32 DIER;              
    
    struct {
      __IOM u32 UIE        : 1;  
      __IOM u32 CC1IE      : 1;  
      __IOM u32 CC2IE      : 1;  
      __IOM u32 CC3IE      : 1;  
      __IOM u32 CC4IE      : 1;  
      __IOM u32 COMIE      : 1;  
      __IOM u32 TIE        : 1;  
      __IOM u32 BIE        : 1;  
      __IOM u32 UDE        : 1;  
      __IOM u32 CC1DE      : 1;  
      __IOM u32 CC2DE      : 1;  
      __IOM u32 CC3DE      : 1;  
      __IOM u32 CC4DE      : 1;  
      __IOM u32 COMDE      : 1;  
      __IOM u32 TDE        : 1;  
            u32            : 5;
      __IOM u32 IDXIE      : 1;  
      __IOM u32 DIRIE      : 1;  
      __IOM u32 IERRIE     : 1;  
      __IOM u32 TERRIE     : 1;  
            u32            : 8;
    } DIER_b;
  } ;
  
  union {
    __IOM u32 SR;                
    
    struct {
      __IOM u32 UIF        : 1;  
      __IOM u32 CC1IF      : 1;  
      __IOM u32 CC2IF      : 1;  
      __IOM u32 CC3IF      : 1;  
      __IOM u32 CC4IF      : 1;  
      __IOM u32 COMIF      : 1;  
      __IOM u32 TIF        : 1;  
      __IOM u32 BIF        : 1;  
      __IOM u32 B2IF       : 1;  
      __IOM u32 CC1OF      : 1;  
      __IOM u32 CC2OF      : 1;  
      __IOM u32 CC3OF      : 1;  
      __IOM u32 CC4OF      : 1;  
      __IOM u32 SBIF       : 1;  
            u32            : 2;
      __IOM u32 CC5IF      : 1;  
      __IOM u32 CC6IF      : 1;  
            u32            : 2;
      __IOM u32 IDXF       : 1;  
      __IOM u32 DIRF       : 1;  
      __IOM u32 IERRF      : 1;  
      __IOM u32 TERRF      : 1;  
            u32            : 8;
    } SR_b;
  } ;
  
  union {
    __OM  u32 EGR;               
    
    struct {
      __OM  u32 UG         : 1;  
      __OM  u32 CC1G       : 1;  
      __OM  u32 CC2G       : 1;  
      __OM  u32 CC3G       : 1;  
      __OM  u32 CC4G       : 1;  
      __OM  u32 COMG       : 1;  
      __OM  u32 TG         : 1;  
      __OM  u32 BG         : 1;  
      __OM  u32 B2G        : 1;  
            u32            : 23;
    } EGR_b;
  } ;
  
  union {
    union {
      __IOM u32 CCMR1_Output;    
      
      struct {
        __IOM u32 CC1S     : 2;  
        __IOM u32 OC1FE    : 1;  
        __IOM u32 OC1PE    : 1;  
        __IOM u32 OC1M     : 3;  
        __IOM u32 OC1CE    : 1;  
        __IOM u32 CC2S     : 2;  
        __IOM u32 OC2FE    : 1;  
        __IOM u32 OC2PE    : 1;  
        __IOM u32 OC2M     : 3;  
        __IOM u32 OC2CE    : 1;  
        __IOM u32 OC1M_3   : 1;  
              u32          : 7;
        __IOM u32 OC2M_3   : 1;  
              u32          : 7;
      } CCMR1_Output_b;
    } ;
    
    union {
      __IOM u32 CCMR1_Input;     
      
      struct {
        __IOM u32 CC1S     : 2;  
        __IOM u32 ICPCS    : 2;  
        __IOM u32 IC1F     : 4;  
        __IOM u32 CC2S     : 2;  
        __IOM u32 IC2PSC   : 2;  
        __IOM u32 IC2F     : 4;  
              u32          : 16;
      } CCMR1_Input_b;
    } ;
  };
  
  union {
    union {
      __IOM u32 CCMR2_Output;    
      
      struct {
        __IOM u32 CC3S     : 2;  
        __IOM u32 OC3FE    : 1;  
        __IOM u32 OC3PE    : 1;  
        __IOM u32 OC3M     : 3;  
        __IOM u32 OC3CE    : 1;  
        __IOM u32 CC4S     : 2;  
        __IOM u32 OC4FE    : 1;  
        __IOM u32 OC4PE    : 1;  
        __IOM u32 OC4M     : 3;  
        __IOM u32 OC4CE    : 1;  
        __IOM u32 OC3M_3   : 1;  
              u32          : 7;
        __IOM u32 OC4M_3   : 1;  
              u32          : 7;
      } CCMR2_Output_b;
    } ;
    
    union {
      __IOM u32 CCMR2_Input;     
      
      struct {
        __IOM u32 CC3S     : 2;  
        __IOM u32 IC3PSC   : 2;  
        __IOM u32 IC3F     : 4;  
        __IOM u32 CC4S     : 2;  
        __IOM u32 IC4PSC   : 2;  
        __IOM u32 IC4F     : 4;  
              u32          : 16;
      } CCMR2_Input_b;
    } ;
  };
  
  union {
    __IOM u32 CCER;              
    
    struct {
      __IOM u32 CC1E       : 1;  
      __IOM u32 CC1P       : 1;  
      __IOM u32 CC1NE      : 1;  
      __IOM u32 CC1NP      : 1;  
      __IOM u32 CC2E       : 1;  
      __IOM u32 CC2P       : 1;  
      __IOM u32 CC2NE      : 1;  
      __IOM u32 CC2NP      : 1;  
      __IOM u32 CC3E       : 1;  
      __IOM u32 CC3P       : 1;  
      __IOM u32 CC3NE      : 1;  
      __IOM u32 CC3NP      : 1;  
      __IOM u32 CC4E       : 1;  
      __IOM u32 CC4P       : 1;  
      __IOM u32 CC4NE      : 1;  
      __IOM u32 CC4NP      : 1;  
      __IOM u32 CC5E       : 1;  
      __IOM u32 CC5P       : 1;  
            u32            : 2;
      __IOM u32 CC6E       : 1;  
      __IOM u32 CC6P       : 1;  
            u32            : 10;
    } CCER_b;
  } ;
  
  union {
    __IOM u32 CNT;               
    
    struct {
      __IOM u32 CNT        : 16; 
            u32            : 15;
      __IM  u32 UIFCPY     : 1;  
    } CNT_b;
  } ;
  
  union {
    __IOM u32 PSC;               
    
    struct {
      __IOM u32 PSC        : 16; 
            u32            : 16;
    } PSC_b;
  } ;
  
  union {
    __IOM u32 ARR;               
    
    struct {
      __IOM u32 ARR        : 16; 
            u32            : 16;
    } ARR_b;
  } ;
  
  union {
    __IOM u32 RCR;               
    
    struct {
      __IOM u32 REP        : 16; 
            u32            : 16;
    } RCR_b;
  } ;
  
  union {
    __IOM u32 CCR1;              
    
    struct {
      __IOM u32 CCR1       : 16; 
            u32            : 16;
    } CCR1_b;
  } ;
  
  union {
    __IOM u32 CCR2;              
    
    struct {
      __IOM u32 CCR2       : 16; 
            u32            : 16;
    } CCR2_b;
  } ;
  
  union {
    __IOM u32 CCR3;              
    
    struct {
      __IOM u32 CCR3       : 16; 
            u32            : 16;
    } CCR3_b;
  } ;
  
  union {
    __IOM u32 CCR4;              
    
    struct {
      __IOM u32 CCR4       : 16; 
            u32            : 16;
    } CCR4_b;
  } ;
  
  union {
    __IOM u32 BDTR;              
    
    struct {
      __IOM u32 DTG        : 8;  
      __IOM u32 LOCK       : 2;  
      __IOM u32 OSSI       : 1;  
      __IOM u32 OSSR       : 1;  
      __IOM u32 BKE        : 1;  
      __IOM u32 BKP        : 1;  
      __IOM u32 AOE        : 1;  
      __IOM u32 MOE        : 1;  
      __IOM u32 BKF        : 4;  
      __IOM u32 BK2F       : 4;  
      __IOM u32 BK2E       : 1;  
      __IOM u32 BK2P       : 1;  
      __IOM u32 BKDSRM     : 1;  
      __IOM u32 BK2DSRM    : 1;  
      __IOM u32 BKBID      : 1;  
      __IOM u32 BK2ID      : 1;  
            u32            : 2;
    } BDTR_b;
  } ;

#if defined (STM32F303)
  union {
    __IOM u32 DCR;               
    
    struct {
      __IOM u32 DBA        : 5;  
            u32            : 3;
      __IOM u32 DBL        : 5;  
            u32            : 19;
    } DCR_b;
  } ;
  
  union {
    __IOM u32 DMAR;              
    
    struct {
      __IOM u32 DMAB       : 16; 
            u32            : 16;
    } DMAR_b;
  } ;

  __IOM  u32  OR;
  
  union {
    __IOM u32 CCMR3_Output;      
    
    struct {
            u32            : 2;
      __IOM u32 OC5FE      : 1;  
      __IOM u32 OC5PE      : 1;  
      __IOM u32 OC5M       : 3;  
      __IOM u32 OC5CE      : 1;  
            u32            : 2;
      __IOM u32 OC6FE      : 1;  
      __IOM u32 OC6PE      : 1;  
      __IOM u32 OC6M       : 3;  
      __IOM u32 OC6CE      : 1;  
      __IOM u32 OC5M_3     : 1;  
            u32            : 7;
      __IOM u32 OC6M_3     : 1;  
            u32            : 7;
    } CCMR3_Output_b;
  } ;
  
  union {
    __IOM u32 CCR5;              
    
    struct {
      __IOM u32 CCR5       : 16; 
            u32            : 13;
      __IOM u32 GC5C1      : 1;  
      __IOM u32 GC5C2      : 1;  
      __IOM u32 GC5C3      : 1;  
    } CCR5_b;
  } ;
  
  union {
    __IOM u32 CCR6;              
    
    struct {
      __IOM u32 CCR6       : 16; 
            u32            : 16;
    } CCR6_b;
  } ;

#elif defined (STM32G473)
  union {
    __IOM u32 CCR5;              
    
    struct {
      __IOM u32 CCR5       : 16; 
            u32            : 13;
      __IOM u32 GC5C1      : 1;  
      __IOM u32 GC5C2      : 1;  
      __IOM u32 GC5C3      : 1;  
    } CCR5_b;
  } ;
  
  union {
    __IOM u32 CCR6;              
    
    struct {
      __IOM u32 CCR6       : 16; 
            u32            : 16;
    } CCR6_b;
  } ;
  
  union {
    __IOM u32 CCMR3_Output;      
    
    struct {
            u32            : 2;
      __IOM u32 OC5FE      : 1;  
      __IOM u32 OC5PE      : 1;  
      __IOM u32 OC5M       : 3;  
      __IOM u32 OC5CE      : 1;  
            u32            : 2;
      __IOM u32 OC6FE      : 1;  
      __IOM u32 OC6PE      : 1;  
      __IOM u32 OC6M       : 3;  
      __IOM u32 OC6CE      : 1;  
      __IOM u32 OC5M_bit3  : 3;  
            u32            : 5;
      __IOM u32 OC6M_bit3  : 1;  
            u32            : 7;
    } CCMR3_Output_b;
  } ;
  
  union {
    __IOM u32 DTR2;              
    
    struct {
      __IOM u32 DTGF       : 8;  
            u32            : 8;
      __IOM u32 DTAE       : 1;  
      __IOM u32 DTPE       : 1;  
            u32            : 14;
    } DTR2_b;
  } ;
  
  union {
    __IOM u32 ECR;               
    
    struct {
      __IOM u32 IE         : 1;  
      __IOM u32 IDIR       : 2;  
      __IOM u32 IBLK       : 2;  
      __IOM u32 FIDX       : 1;  
      __IOM u32 IPOS       : 2;  
            u32            : 8;
      __IOM u32 PW         : 8;  
      __IOM u32 PWPRSC     : 3;  
            u32            : 5;
    } ECR_b;
  } ;
  
  union {
    __IOM u32 TISEL;             
    
    struct {
      __IOM u32 TI1SEL     : 4;  
            u32            : 4;
      __IOM u32 TI2SEL     : 4;  
            u32            : 4;
      __IOM u32 TI3SEL     : 4;  
            u32            : 4;
      __IOM u32 TI4SEL     : 4;  
            u32            : 4;
    } TISEL_b;
  } ;
  
  union {
    __IOM u32 AF1;               
    
    struct {
      __IOM u32 BKINE      : 1;  
      __IOM u32 BKCMP1E    : 1;  
      __IOM u32 BKCMP2E    : 1;  
      __IOM u32 BKCMP3E    : 1;  
      __IOM u32 BKCMP4E    : 1;  
      __IOM u32 BKCMP5E    : 1;  
      __IOM u32 BKCMP6E    : 1;  
      __IOM u32 BKCMP7E    : 1;  
            u32            : 1;
      __IOM u32 BKINP      : 1;  
      __IOM u32 BKCMP1P    : 1;  
      __IOM u32 BKCMP2P    : 1;  
      __IOM u32 BKCMP3P    : 1;  
      __IOM u32 BKCMP4P    : 1;  
      __IOM u32 ETRSEL     : 4;  
            u32            : 14;
    } AF1_b;
  } ;
  
  union {
    __IOM u32 AF2;               
    
    struct {
      __IOM u32 BKINE      : 1;  
      __IOM u32 BK2CMP1E   : 1;  
      __IOM u32 BK2CMP2E   : 1;  
      __IOM u32 BK2CMP3E   : 1;  
      __IOM u32 BK2CMP4E   : 1;  
      __IOM u32 BK2CMP5E   : 1;  
      __IOM u32 BK2CMP6E   : 1;  
      __IOM u32 BK2CMP7E   : 1;  
            u32            : 1;
      __IOM u32 BK2INP     : 1;  
      __IOM u32 BK2CMP1P   : 1;  
      __IOM u32 BK2CMP2P   : 1;  
      __IOM u32 BK2CMP3P   : 1;  
      __IOM u32 BK2CMP4P   : 1;  
            u32            : 2;
      __IOM u32 OCRSEL     : 3;  
            u32            : 13;
    } AF2_b;
  } ;

  __IOM u32  OR;
  __IM  u32  RESERVED[220];
  
  union {
    __IOM u32 DCR;               
    
    struct {
      __IOM u32 DBA        : 5;  
            u32            : 3;
      __IOM u32 DBL        : 5;  
            u32            : 19;
    } DCR_b;
  } ;
  
  __IOM u32 DMAR;
  #endif  /* STM32F303 */
  
} TIM_HW;


/*=========== UNIVERSAL SYNCHRONOUS ASYNCHRONOUS RECEIVER TRANSMITTER ==============*/

typedef struct {                       
  
  union {
    __IOM u32 CR1;                
    
    struct {
      __IOM u32 UE         : 1;   
      __IOM u32 UESM       : 1;   
      __IOM u32 RE         : 1;   
      __IOM u32 TE         : 1;   
      __IOM u32 IDLEIE     : 1;   
      __IOM u32 RXNEIE     : 1;   
      __IOM u32 TCIE       : 1;   
      __IOM u32 TXEIE      : 1;   
      __IOM u32 PEIE       : 1;   
      __IOM u32 PS         : 1;   
      __IOM u32 PCE        : 1;   
      __IOM u32 WAKE       : 1;   
      __IOM u32 M0         : 1;   
      __IOM u32 MME        : 1;   
      __IOM u32 CMIE       : 1;   
      __IOM u32 OVER8      : 1;   
      __IOM u32 DEDT0      : 1;   
      __IOM u32 DEDT1      : 1;   
      __IOM u32 DEDT2      : 1;   
      __IOM u32 DEDT3      : 1;   
      __IOM u32 DEDT4      : 1;   
      __IOM u32 DEAT0      : 1;   
      __IOM u32 DEAT1      : 1;   
      __IOM u32 DEAT2      : 1;   
      __IOM u32 DEAT3      : 1;   
      __IOM u32 DEAT4      : 1;   
      __IOM u32 RTOIE      : 1;   
      __IOM u32 EOBIE      : 1;   
      __IOM u32 M1         : 1;   
      __IOM u32 FIFOEN     : 1;   
      __IOM u32 TXFEIE     : 1;   
      __IOM u32 RXFFIE     : 1;   
    } CR1_b;
  } ;
  
  union {
    __IOM u32 CR2;                
    
    struct {
      __IOM u32 SLVEN      : 1;   
            u32            : 2;
      __IOM u32 DIS_NSS    : 1;   
      __IOM u32 ADDM7      : 1;   
      __IOM u32 LBDL       : 1;   
      __IOM u32 LBDIE      : 1;   
            u32            : 1;
      __IOM u32 LBCL       : 1;   
      __IOM u32 CPHA       : 1;   
      __IOM u32 CPOL       : 1;   
      __IOM u32 CLKEN      : 1;   
      __IOM u32 STOP       : 2;   
      __IOM u32 LINEN      : 1;   
      __IOM u32 SWAP       : 1;   
      __IOM u32 RXINV      : 1;   
      __IOM u32 TXINV      : 1;   
      __IOM u32 TAINV      : 1;   
      __IOM u32 MSBFIRST   : 1;   
      __IOM u32 ABREN      : 1;   
      __IOM u32 ABRMOD0    : 1;   
      __IOM u32 ABRMOD1    : 1;   
      __IOM u32 RTOEN      : 1;   
      __IOM u32 ADD0_3     : 4;   
      __IOM u32 ADD4_7     : 4;   
    } CR2_b;
  } ;
  
  union {
    __IOM u32 CR3;                
    
    struct {
      __IOM u32 EIE        : 1;   
      __IOM u32 IREN       : 1;   
      __IOM u32 IRLP       : 1;   
      __IOM u32 HDSEL      : 1;   
      __IOM u32 NACK       : 1;   
      __IOM u32 SCEN       : 1;   
      __IOM u32 DMAR       : 1;   
      __IOM u32 DMAT       : 1;   
      __IOM u32 RTSE       : 1;   
      __IOM u32 CTSE       : 1;   
      __IOM u32 CTSIE      : 1;   
      __IOM u32 ONEBIT     : 1;   
      __IOM u32 OVRDIS     : 1;   
      __IOM u32 DDRE       : 1;   
      __IOM u32 DEM        : 1;   
      __IOM u32 DEP        : 1;   
            u32            : 1;
      __IOM u32 SCARCNT    : 3;   
      __IOM u32 WUS        : 2;   
      __IOM u32 WUFIE      : 1;   
      __IOM u32 TXFTIE     : 1;   
      __IOM u32 TCBGTIE    : 1;   
      __IOM u32 RXFTCFG    : 3;   
      __IOM u32 RXFTIE     : 1;   
      __IOM u32 TXFTCFG    : 3;   
    } CR3_b;
  } ;
  
  union {
    __IOM u32 BRR;                
    
    struct {
      __IOM u32 DIV_Fraction : 4; 
      __IOM u32 DIV_Mantissa : 12;
            u32            : 16;
    } BRR_b;
  } ;
  
  union {
    __IOM u32 GTPR;               
    
    struct {
      __IOM u32 PSC        : 8;   
      __IOM u32 GT         : 8;   
            u32            : 16;
    } GTPR_b;
  } ;
  
  union {
    __IOM u32 RTOR;               
    
    struct {
      __IOM u32 RTO        : 24;  
      __IOM u32 BLEN       : 8;   
    } RTOR_b;
  } ;
  
  union {
    __OM  u32 RQR;                
    
    struct {
      __OM  u32 ABRRQ      : 1;   
      __OM  u32 SBKRQ      : 1;   
      __OM  u32 MMRQ       : 1;   
      __OM  u32 RXFRQ      : 1;   
      __OM  u32 TXFRQ      : 1;   
            u32            : 27;
    } RQR_b;
  } ;
  
  union {
    __IM  u32 ISR;                
    
    struct {
      __IM  u32 PE         : 1;   
      __IM  u32 FE         : 1;   
      __IM  u32 NF         : 1;   
      __IM  u32 ORE        : 1;   
      __IM  u32 IDLE       : 1;   
      __IM  u32 RXNE       : 1;   
      __IM  u32 TC         : 1;   
      __IM  u32 TXE        : 1;   
      __IM  u32 LBDF       : 1;   
      __IM  u32 CTSIF      : 1;   
      __IM  u32 CTS        : 1;   
      __IM  u32 RTOF       : 1;   
      __IM  u32 EOBF       : 1;   
      __IM  u32 UDR        : 1;   
      __IM  u32 ABRE       : 1;   
      __IM  u32 ABRF       : 1;   
      __IM  u32 BUSY       : 1;   
      __IM  u32 CMF        : 1;   
      __IM  u32 SBKF       : 1;   
      __IM  u32 RWU        : 1;   
      __IM  u32 WUF        : 1;   
      __IM  u32 TEACK      : 1;   
      __IM  u32 REACK      : 1;   
      __IM  u32 TXFE       : 1;   
      __IM  u32 RXFF       : 1;   
      __IM  u32 TCBGT      : 1;   
      __IM  u32 RXFT       : 1;   
      __IM  u32 TXFT       : 1;   
            u32            : 4;
    } ISR_b;
  } ;
  
  union {
    __OM  u32 ICR;                
    
    struct {
      __OM  u32 PECF       : 1;   
      __OM  u32 FECF       : 1;   
      __OM  u32 NCF        : 1;   
      __OM  u32 ORECF      : 1;   
      __OM  u32 IDLECF     : 1;   
      __OM  u32 TXFECF     : 1;   
      __OM  u32 TCCF       : 1;   
      __OM  u32 TCBGTCF    : 1;   
      __OM  u32 LBDCF      : 1;   
      __OM  u32 CTSCF      : 1;   
            u32            : 1;
      __OM  u32 RTOCF      : 1;   
      __OM  u32 EOBCF      : 1;   
      __OM  u32 UDRCF      : 1;   
            u32            : 3;
      __OM  u32 CMCF       : 1;   
            u32            : 2;
      __OM  u32 WUCF       : 1;   
            u32            : 11;
    } ICR_b;
  } ;
  
  union {
    __IM  u32 RDR;                
    
    struct {
      __IM  u32 RDR        : 9;   
            u32            : 23;
    } RDR_b;
  } ;
  
  union {
    __IOM u32 TDR;                
    
    struct {
      __IOM u32 TDR        : 9;   
            u32            : 23;
    } TDR_b;
  } ;
  
  union {
    __IOM u32 PRESC;              
    
    struct {
      __IOM u32 PRESCALER  : 4;   
            u32            : 28;
    } PRESC_b;
  } ;

} USART_HW;


/*============================= USB FULL SPEED =====================================*/

typedef struct {
  
  union {
    __IOM u32 EP0R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP0R_b;
  } ;
  
  union {
    __IOM u32 EP1R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP1R_b;
  } ;
  
  union {
    __IOM u32 EP2R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP2R_b;
  } ;
  
  union {
    __IOM u32 EP3R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP3R_b;
  } ;
  
  union {
    __IOM u32 EP4R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP4R_b;
  } ;
  
  union {
    __IOM u32 EP5R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP5R_b;
  } ;
  
  union {
    __IOM u32 EP6R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP6R_b;
  } ;
  
  union {
    __IOM u32 EP7R;
    
    struct {
      __IOM u32 EA         : 4;
      __IOM u32 STAT_TX    : 2;
      __IOM u32 DTOG_TX    : 1;
      __IOM u32 CTR_TX     : 1;
      __IOM u32 EP_KIND    : 1;
      __IOM u32 EP_TYPE    : 2;
      __IOM u32 SETUP      : 1;
      __IOM u32 STAT_RX    : 2;
      __IOM u32 DTOG_RX    : 1;
      __IOM u32 CTR_RX     : 1;
            u32            : 16;
    } EP7R_b;
  } ;
  __IM  u32  RESERVED[8];
  
  union {
    __IOM u32 CNTR;
    
    struct {
      __IOM u32 FRES       : 1;
      __IOM u32 PDWN       : 1;
      __IOM u32 LP_MODE    : 1;
      __IOM u32 FSUSP      : 1;
      __IOM u32 RESUME     : 1;
      __IOM u32 L1RESUME   : 1;
            u32            : 1;
      __IOM u32 L1REQM     : 1;
      __IOM u32 ESOFM      : 1;
      __IOM u32 SOFM       : 1;
      __IOM u32 RESETM     : 1;
      __IOM u32 SUSPM      : 1;
      __IOM u32 WKUPM      : 1;
      __IOM u32 ERRM       : 1;
      __IOM u32 PMAOVRM    : 1;
      __IOM u32 CTRM       : 1;
            u32            : 16;
    } CNTR_b;
  } ;
  
  union {
    __IOM u32 ISTR;
    
    struct {
      __IOM u32 EP_ID      : 4;
      __IOM u32 DIR        : 1;
            u32            : 2;
      __IOM u32 L1REQ      : 1;
      __IOM u32 ESOF       : 1;
      __IOM u32 SOF        : 1;
      __IOM u32 RESET      : 1;
      __IOM u32 SUSP       : 1;
      __IOM u32 WKUP       : 1;
      __IOM u32 ERR        : 1;
      __IOM u32 PMAOVR     : 1;
      __IOM u32 CTR        : 1;
            u32            : 16;
    } ISTR_b;
  } ;
  
  union {
    __IM  u32 FNR;
    
    struct {
      __IM  u32 FN         : 11;
      __IM  u32 LSOF       : 2; 
      __IM  u32 LCK        : 1; 
      __IM  u32 RXDM       : 1; 
      __IM  u32 RXDP       : 1; 
            u32            : 16;
    } FNR_b;
  } ;
  
  union {
    __IOM u32 DADDR;
    
    struct {
      __IOM u32 ADD        : 7;
      __IOM u32 EF         : 1;
            u32            : 24;
    } DADDR_b;
  } ;
  
  union {
    __IOM u32 BTABLE;
    
    struct {
            u32            : 3;
      __IOM u32 BTABLE     : 13;
            u32            : 16;
    } BTABLE_b;
  } ;

} USB_HW;


/*============================ WINDOW WATCHDDOG ====================================*/

typedef struct {                     
  
  union {
    __IOM u32 CR;               
    
    struct {
      __IOM u32 T          : 7; 
      __IOM u32 WDGA       : 1; 
            u32            : 24;
    } CR_b;
  } ;
  
  union {
    __IOM u32 CFR;              
    
    struct {
      __IOM u32 W          : 7; 
            u32            : 2;
      __IOM u32 EWI        : 1; 
            u32            : 1;
      __IOM u32 WDGTB      : 3; 
            u32            : 18;
    } CFR_b;
  } ;
  
  union {
    __IOM u32 SR;               
    
    struct {
      __IOM u32 EWIF       : 1; 
            u32            : 31;
    } SR_b;
  } ;

} WWDG_HW;


/*========================= RANDOM NUMBER GENERATOR ================================*/

typedef struct {                     
  
  union {
    __IOM u32 CR;               
    
    struct {
            u32            : 2;
      __IOM u32 RNGEN      : 1; 
      __IOM u32 IE         : 1; 
            u32            : 1;
      __IOM u32 CED        : 1; 
            u32            : 26;
    } CR_b;
  } ;
  
  union {
    __IOM u32 SR;               
    
    struct {
      __IM  u32 DRDY       : 1; 
      __IM  u32 CECS       : 1; 
      __IM  u32 SECS       : 1; 
            u32            : 2;
      __IOM u32 CEIS       : 1; 
      __IOM u32 SEIS       : 1; 
            u32            : 25;
    } SR_b;
  } ;
  
  union {
    __IM  u32 DR;               
    
    struct {
      __IM  u32 RNDATA     : 32;
    } DR_b;
  } ;

} RNG_HW;


/*================================ CORDIC ==========================================*/

typedef struct {

  union {
    __IOM u32 CSR;              
    
    struct {
      __IOM u32 FUNC       : 4; 
      __IOM u32 PRECISION  : 4; 
      __IOM u32 SCALE      : 3; 
            u32            : 5;
      __IOM u32 IEN        : 1; 
      __IOM u32 DMAREN     : 1; 
      __IOM u32 DMAWEN     : 1; 
      __IOM u32 NRES       : 1; 
      __IOM u32 NARGS      : 1; 
      __IOM u32 RESSIZE    : 1; 
      __IOM u32 ARGSIZE    : 1; 
            u32            : 8;
      __IOM u32 RRDY       : 1; 
    } CSR_b;
  } ;
 
  __IOM u32 WDATA;
  __IM  u32 RDATA;

} CORDIC_HW;


/*========================= VOLTAGE REFERENCE BUFFER ===============================*/

typedef struct {                     
  
  union {
    __IOM u32 VREFBUF_CSR;      
    
    struct {
      __IOM u32 ENVR       : 1; 
      __IOM u32 HIZ        : 1; 
            u32            : 1;
      __IM  u32 VRR        : 1; 
      __IOM u32 VRS        : 2; 
            u32            : 26;
    } VREFBUF_CSR_b;
  } ;
  
  union {
    __IOM u32 VREFBUF_CCR;      
    
    struct {
      __IOM u32 TRIM       : 6; 
            u32            : 26;
    } VREFBUF_CCR_b;
  } ;

} VREFBUF_HW;




/*==================================================================================
|                           PERIPHERALS BASE ADRRESSES                                
===================================================================================*/

#define ADC1_BASE_ADDRESS                   (0x50000000UL)
#define ADC2_BASE_ADDRESS                   (0x50000100UL)
#define ADC3_BASE_ADDRESS                   (0x50000400UL)
#define ADC4_BASE_ADDRESS                   (0x50000500UL)
#define ADC12_COMMON_BASE                   (0x50000300UL)
#define ADC345_COMMON_BASE                  (0x50000700UL)

#define GPIOA_BASE_ADDRESS                  (0x48000000UL)
#define GPIOB_BASE_ADDRESS                  (0x48000400UL)
#define GPIOC_BASE_ADDRESS                  (0x48000800UL)
#define GPIOD_BASE_ADDRESS                  (0x48000C00UL)
#define GPIOE_BASE_ADDRESS                  (0x48001000UL)
#define GPIOF_BASE_ADDRESS                  (0x48001400UL)

#define CRC_BASE_ADDRESS                    (0x40023000UL)
#define RCC_BASE_ADDRESS                    (0x40021000UL)
#define DMA1_BASE_ADDRESS                   (0x40020000UL)
#define DMA2_BASE_ADDRESS                   (0x40020400UL)
#define DMA1_CHANNEL1_ADDRESS               (0x40020008UL)
#define DMA1_CHANNEL2_ADDRESS               (0x4002001CUL)
#define DMA1_CHANNEL3_ADDRESS               (0x40020030UL)
#define DMA1_CHANNEL4_ADDRESS               (0x40020044UL)
#define DMA1_CHANNEL5_ADDRESS               (0x40020058UL)
#define DMA1_CHANNEL6_ADDRESS               (0x4002006CUL)
#define DMA1_CHANNEL7_ADDRESS               (0x40020080UL)
#define DMA2_CHANNEL1_ADDRESS               (0x40020408UL)
#define DMA2_CHANNEL2_ADDRESS               (0x4002041CUL)
#define DMA2_CHANNEL3_ADDRESS               (0x40020430UL)
#define DMA2_CHANNEL4_ADDRESS               (0x40020444UL)
#define DMA2_CHANNEL5_ADDRESS               (0x40020458UL)
#define FLASH_REG_BASE_ADDRESS              (0x40022000UL)

#define EXTI_BASE_ADDRESS                   (0x40010400UL)
#define TIM1_BASE_ADDRESS                   (0x40012C00UL)
#define SPI1_BASE_ADDRESS                   (0x40013000UL)
#define TIM8_BASE_ADDRESS                   (0x40013400UL)
#define TIM15_BASE_ADDRESS                  (0x40014000UL)
#define TIM16_BASE_ADDRESS                  (0x40014400UL)
#define TIM17_BASE_ADDRESS                  (0x40014800UL)
#define SYSCFG_BASE_ADDRESS                 (0x40010000UL)
#define USART1_BASE_ADDRESS                 (0x40013800UL)

#define USB_BASE_ADDRESS                    (0x40005C00UL)
#define PWR_BASE_ADDRESS                    (0x40007000UL)
#define RTC_BASE_ADDRESS                    (0x40002800UL)
#define TIM2_BASE_ADDRESS                   (0x40000000UL)
#define TIM3_BASE_ADDRESS                   (0x40000400UL)
#define TIM4_BASE_ADDRESS                   (0x40000800UL)
#define TIM6_BASE_ADDRESS                   (0x40001000UL)
#define TIM7_BASE_ADDRESS                   (0x40001400UL)
#define WWDG_BASE_ADDRESS                   (0x40002C00UL)
#define IWDG_BASE_ADDRESS                   (0x40003000UL)
#define SPI2_BASE_ADDRESS                   (0x40003800UL)
#define SPI3_BASE_ADDRESS                   (0x40003C00UL)
#define I2C1_BASE_ADDRESS                   (0x40005400UL)
#define I2C2_BASE_ADDRESS                   (0x40005800UL)
#define UART4_BASE_ADDRESS                  (0x40004C00UL)
#define UART5_BASE_ADDRESS                  (0x40005000UL)
#define USART2_BASE_ADDRESS                 (0x40004400UL)
#define USART3_BASE_ADDRESS                 (0x40004800UL)

#define DBGMCU_BASE_ADDRESS                 (0xE0042000UL)

#if defined (STM32G473)     /* STM32G473 specific peripherals base addresses */

#define GPIOG_BASE_ADDRESS                  (0x48001800UL)
#define ADC5_BASE_ADDRESS                   (0x50000600UL)
#define DAC1_BASE_ADDRESS                   (0x50000800UL)
#define DAC2_BASE_ADDRESS                   (0x50000C00UL)
#define DAC3_BASE_ADDRESS                   (0x50001000UL)
#define DAC4_BASE_ADDRESS                   (0x50001400UL)
#define RNG_BASE_ADDRESS                    (0x50060800UL)

#define DMAMUX_BASE_ADDRESS                 (0x40020800UL)
#define CORDIC_BASE_ADDRESS                 (0x40020C00UL)
#define DMA1_CHANNEL8_ADDRESS               (0x40020094UL)
#define DMA2_CHANNEL6_ADDRESS               (0x4002046CUL)
#define DMA2_CHANNEL7_ADDRESS               (0x40020480UL)
#define DMA2_CHANNEL8_ADDRESS               (0x40020494UL)
#define FMAC_BASE_ADDRESS                   (0x40021400UL)

#define VREFBUF_BASE_ADDRESS                (0x40010030UL)
#define COMP1_BASE_ADDRESS                  (0x40010200UL)
#define COMP2_BASE_ADDRESS                  (0x40010204UL)
#define COMP3_BASE_ADDRESS                  (0x40010208UL)
#define COMP4_BASE_ADDRESS                  (0x4001020CUL)
#define COMP5_BASE_ADDRESS                  (0x40010210UL)
#define COMP6_BASE_ADDRESS                  (0x40010214UL)
#define COMP7_BASE_ADDRESS                  (0x40010218UL)
#define OPAMP1_BASE_ADDRESS                 (0x40010300UL)
#define OPAMP2_BASE_ADDRESS                 (0x40010304UL)
#define OPAMP3_BASE_ADDRESS                 (0x40010308UL)
#define OPAMP4_BASE_ADDRESS                 (0x4001030CUL)
#define OPAMP5_BASE_ADDRESS                 (0x40010310UL)
#define OPAMP6_BASE_ADDRESS                 (0x40010314UL)

#define TIM20_BASE_ADDRESS                  (0x40015000UL)
#define SAI1_BASE_ADDRESS                   (0x40015400UL)
#define SAI1_Block_A_BASE                   (0x40015404UL)
#define SAI1_Block_B_BASE                   (0x40015424UL)

#define TIM5_BASE_ADDRESS                   (0x40000C00UL)
#define CRS_BASE_ADDRESS                    (0x40002000UL)
#define TAMP_BASE_ADDRESS                   (0x40002400UL)
#define LPTIM1_BASE_ADDRESS                 (0x40007C00UL)
#define LPUART1_BASE_ADDRESS                (0x40008000UL)
#define I2C3_BASE_ADDRESS                   (0x40007800UL)
#define I2C4_BASE_ADDRESS                   (0x40008400UL)

#define DMAMUX1_CHANNEL0_BASE_ADDRESS       (0x40020800UL)
#define DMAMUX1_CHANNEL1_BASE_ADDRESS       (0x40020804UL)
#define DMAMUX1_CHANNEL2_BASE_ADDRESS       (0x40020808UL)
#define DMAMUX1_CHANNEL3_BASE_ADDRESS       (0x4002080CUL)
#define DMAMUX1_CHANNEL4_BASE_ADDRESS       (0x40020810UL)
#define DMAMUX1_CHANNEL5_BASE_ADDRESS       (0x40020814UL)
#define DMAMUX1_CHANNEL6_BASE_ADDRESS       (0x40020818UL)
#define DMAMUX1_CHANNEL7_BASE_ADDRESS       (0x4002081CUL)
#define DMAMUX1_CHANNEL8_BASE_ADDRESS       (0x40020820UL)
#define DMAMUX1_CHANNEL9_BASE_ADDRESS       (0x40020824UL)
#define DMAMUX1_CHANNEL10_BASE_ADDRESS      (0x40020828UL)
#define DMAMUX1_CHANNEL11_BASE_ADDRESS      (0x4002082CUL)
#define DMAMUX1_CHANNEL12_BASE_ADDRESS      (0x40020830UL)
#define DMAMUX1_CHANNEL13_BASE_ADDRESS      (0x40020834UL)
#define DMAMUX1_CHANNEL14_BASE_ADDRESS      (0x40020838UL)
#define DMAMUX1_CHANNEL15_BASE_ADDRESS      (0x4002083CUL)
#define DMAMUX1_REQUESTGEN0_BASE_ADDRESS    (0x40020900UL)
#define DMAMUX1_REQUESTGEN1_BASE_ADDRESS    (0x40020904UL)
#define DMAMUX1_REQUESTGEN2_BASE_ADDRESS    (0x40020908UL)
#define DMAMUX1_REQUESTGEN3_BASE_ADDRESS    (0x4002090CUL)

#define DMAMUX1_CHANNELSTATUS_ADDRESS       (0x40020880UL)
#define DMAMUX1_REQUESTGENSTATUS_ADDRESS    (0x40020940UL)

#elif defined (STM32F303)   /* STM32F303 specific peripherals base addresses */

#define DAC1_BASE_ADDRESS                   (0x40007400UL)
#define COMP1_BASE_ADDRESS                  (0x4001001CUL)
#define COMP2_BASE_ADDRESS                  (0x40010020UL)
#define COMP3_BASE_ADDRESS                  (0x40010024UL)
#define COMP4_BASE_ADDRESS                  (0x40010028UL)
#define COMP5_BASE_ADDRESS                  (0x4001002CUL)
#define COMP6_BASE_ADDRESS                  (0x40010030UL)
#define COMP7_BASE_ADDRESS                  (0x40010034UL)
#define OPAMP1_BASE_ADDRESS                 (0x40010038UL)
#define OPAMP2_BASE_ADDRESS                 (0x4001003CUL)
#define OPAMP3_BASE_ADDRESS                 (0x40010040UL)
#define OPAMP4_BASE_ADDRESS                 (0x40010044UL)

#endif  /* STM32G473 */



/*==================================================================================
|                        PERIPHERALS POINTERS DECLARATIONS                                
===================================================================================*/

#define TIM2                       ((TIM_HW *) TIM2_BASE_ADDRESS)
#define TIM3                       ((TIM_HW *) TIM3_BASE_ADDRESS)
#define TIM4                       ((TIM_HW *) TIM4_BASE_ADDRESS)
#define TIM5                       ((TIM_HW *) TIM5_BASE_ADDRESS)
#define TIM6                       ((TIM_HW *) TIM6_BASE_ADDRESS)
#define TIM7                       ((TIM_HW *) TIM7_BASE_ADDRESS)
#define CRS                        ((CRS_HW *) CRS_BASE_ADDRESS)
#define TAMP                       ((TAMP_HW *) TAMP_BASE_ADDRESS)
#define RTC                        ((RTC_HW *) RTC_BASE_ADDRESS)
#define WWDG                       ((WWDG_HW *) WWDG_BASE_ADDRESS)
#define IWDG                       ((IWDG_HW *) IWDG_BASE_ADDRESS)
#define SPI2                       ((SPI_HW *) SPI2_BASE_ADDRESS)
#define SPI3                       ((SPI_HW *) SPI3_BASE_ADDRESS)
#define USART2                     ((USART_HW *) USART2_BASE_ADDRESS)
#define USART3                     ((USART_HW *) USART3_BASE_ADDRESS)
#define UART4                      ((USART_HW *) UART4_BASE_ADDRESS)
#define UART5                      ((USART_HW *) UART5_BASE_ADDRESS)
#define I2C1                       ((I2C_HW *) I2C1_BASE_ADDRESS)
#define I2C2                       ((I2C_HW *) I2C2_BASE_ADDRESS)
#define FDCAN1                     ((FDCAN_HW *) FDCAN1_BASE_ADDRESS)
#define FDCAN2                     ((FDCAN_HW *) FDCAN2_BASE_ADDRESS)
#define FDCAN3                     ((FDCAN_HW *) FDCAN3_BASE_ADDRESS)
#define PWR                        ((PWR_HW *) PWR_BASE_ADDRESS)
#define I2C3                       ((I2C_HW *) I2C3_BASE_ADDRESS)
#define LPTIM1                     ((LPTIM_HW *) LPTIM1_BASE_ADDRESS)
#define LPUART1                    ((USART_HW *) LPUART1_BASE_ADDRESS)
#define I2C4                       ((I2C_HW *) I2C4_BASE_ADDRESS)
#define USB                        ((USB_HW *)USB_BASE_ADDRESS)
       
#define SYSCFG                     ((SYSCFG_HW *) SYSCFG_BASE_ADDRESS)
#define VREFBUF                    ((VREFBUF_HW *) VREFBUF_BASE_ADDRESS)
#define COMP1                      ((COMP_HW *) COMP1_BASE_ADDRESS)
#define COMP2                      ((COMP_HW *) COMP2_BASE_ADDRESS)
#define COMP3                      ((COMP_HW *) COMP3_BASE_ADDRESS)
#define COMP4                      ((COMP_HW *) COMP4_BASE_ADDRESS)
#define COMP5                      ((COMP_HW *) COMP5_BASE_ADDRESS)
#define COMP6                      ((COMP_HW *) COMP6_BASE_ADDRESS)
#define COMP7                      ((COMP_HW *) COMP7_BASE_ADDRESS)
       
#define OPAMP1                     ((OPAMP_HW *) OPAMP1_BASE_ADDRESS)
#define OPAMP2                     ((OPAMP_HW *) OPAMP2_BASE_ADDRESS)
#define OPAMP3                     ((OPAMP_HW *) OPAMP3_BASE_ADDRESS)
#define OPAMP4                     ((OPAMP_HW *) OPAMP4_BASE_ADDRESS)
#define OPAMP5                     ((OPAMP_HW *) OPAMP5_BASE_ADDRESS)
#define OPAMP6                     ((OPAMP_HW *) OPAMP6_BASE_ADDRESS)
       
#define EXTI                       ((EXTI_HW *) EXTI_BASE_ADDRESS)
#define TIM1                       ((TIM_HW *) TIM1_BASE_ADDRESS)
#define SPI1                       ((SPI_HW *) SPI1_BASE_ADDRESS)
#define TIM8                       ((TIM_HW *) TIM8_BASE_ADDRESS)
#define USART1                     ((USART_HW *) USART1_BASE_ADDRESS)
#define SPI4                       ((SPI_HW *) SPI4_BASE_ADDRESS)
#define TIM15                      ((TIM_HW *) TIM15_BASE_ADDRESS)
#define TIM16                      ((TIM_HW *) TIM16_BASE_ADDRESS)
#define TIM17                      ((TIM_HW *) TIM17_BASE_ADDRESS)
#define TIM20                      ((TIM_HW *) TIM20_BASE_ADDRESS)
#define SAI1                       ((SAI_HW *) SAI1_BASE_ADDRESS)
#define DMA1                       ((DMA_HW *) DMA1_BASE_ADDRESS)
#define DMA2                       ((DMA_HW *) DMA2_BASE_ADDRESS)
#define CORDIC                     ((CORDIC_HW *) CORDIC_BASE_ADDRESS)
#define RCC                        ((RCC_HW *) RCC_BASE_ADDRESS)
#define FMAC                       ((FMAC_HW *) FMAC_BASE_ADDRESS)
#define FLASH                      ((FLASH_HW *) FLASH_REG_BASE_ADDRESS)
#define CRC                        ((CRC_HW *) CRC_BASE_ADDRESS)
       
#define GPIOA                      ((GPIO_HW *) GPIOA_BASE_ADDRESS)
#define GPIOB                      ((GPIO_HW *) GPIOB_BASE_ADDRESS)
#define GPIOC                      ((GPIO_HW *) GPIOC_BASE_ADDRESS)
#define GPIOD                      ((GPIO_HW *) GPIOD_BASE_ADDRESS)
#define GPIOE                      ((GPIO_HW *) GPIOE_BASE_ADDRESS)
#define GPIOF                      ((GPIO_HW *) GPIOF_BASE_ADDRESS)
#define GPIOG                      ((GPIO_HW *) GPIOG_BASE_ADDRESS)
#define ADC1                       ((ADC_HW *) ADC1_BASE_ADDRESS)
#define ADC2                       ((ADC_HW *) ADC2_BASE_ADDRESS)
#define ADC12_COMMON               ((ADC_Common_HW *) ADC12_COMMON_BASE)
#define ADC3                       ((ADC_HW *) ADC3_BASE_ADDRESS)
#define ADC4                       ((ADC_HW *) ADC4_BASE_ADDRESS)
#define ADC5                       ((ADC_HW *) ADC5_BASE_ADDRESS)
#define ADC345_COMMON              ((ADC_Common_HW *) ADC345_COMMON_BASE)
#define DAC1                       ((DAC_HW *) DAC1_BASE_ADDRESS)
#define DAC2                       ((DAC_HW *) DAC2_BASE_ADDRESS)
#define DAC3                       ((DAC_HW *) DAC3_BASE_ADDRESS)
#define DAC4                       ((DAC_HW *) DAC4_BASE_ADDRESS)
#define RNG                        ((RNG_HW *) RNG_BASE_ADDRESS)
       
#define DMA1_CHANNEL1              ((DMA_CHANNEL_HW *) DMA1_CHANNEL1_ADDRESS)
#define DMA1_CHANNEL2              ((DMA_CHANNEL_HW *) DMA1_CHANNEL2_ADDRESS)
#define DMA1_CHANNEL3              ((DMA_CHANNEL_HW *) DMA1_CHANNEL3_ADDRESS)
#define DMA1_CHANNEL4              ((DMA_CHANNEL_HW *) DMA1_CHANNEL4_ADDRESS)
#define DMA1_CHANNEL5              ((DMA_CHANNEL_HW *) DMA1_CHANNEL5_ADDRESS)
#define DMA1_CHANNEL6              ((DMA_CHANNEL_HW *) DMA1_CHANNEL6_ADDRESS)
#define DMA1_CHANNEL7              ((DMA_CHANNEL_HW *) DMA1_CHANNEL7_ADDRESS)
#define DMA1_CHANNEL8              ((DMA_CHANNEL_HW *) DMA1_CHANNEL8_ADDRESS)
       
#define DMA2_CHANNEL1              ((DMA_CHANNEL_HW *) DMA2_CHANNEL1_ADDRESS)
#define DMA2_CHANNEL2              ((DMA_CHANNEL_HW *) DMA2_CHANNEL2_ADDRESS)
#define DMA2_CHANNEL3              ((DMA_CHANNEL_HW *) DMA2_CHANNEL3_ADDRESS)
#define DMA2_CHANNEL4              ((DMA_CHANNEL_HW *) DMA2_CHANNEL4_ADDRESS)
#define DMA2_CHANNEL5              ((DMA_CHANNEL_HW *) DMA2_CHANNEL5_ADDRESS)
#define DMA2_CHANNEL6              ((DMA_CHANNEL_HW *) DMA2_CHANNEL6_ADDRESS)
#define DMA2_CHANNEL7              ((DMA_CHANNEL_HW *) DMA2_CHANNEL7_ADDRESS)
#define DMA2_CHANNEL8              ((DMA_CHANNEL_HW *) DMA2_CHANNEL8_ADDRESS)
       
#define DMAMUX1_CHANNEL0           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL0_BASE_ADDRESS)
#define DMAMUX1_CHANNEL1           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL1_BASE_ADDRESS)
#define DMAMUX1_CHANNEL2           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL2_BASE_ADDRESS)
#define DMAMUX1_CHANNEL3           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL3_BASE_ADDRESS)
#define DMAMUX1_CHANNEL4           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL4_BASE_ADDRESS)
#define DMAMUX1_CHANNEL5           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL5_BASE_ADDRESS)
#define DMAMUX1_CHANNEL6           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL6_BASE_ADDRESS)
#define DMAMUX1_CHANNEL7           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL7_BASE_ADDRESS)
#define DMAMUX1_CHANNEL8           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL8_BASE_ADDRESS)
#define DMAMUX1_CHANNEL9           ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL9_BASE_ADDRESS)
#define DMAMUX1_CHANNEL10          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL10_BASE_ADDRESS)
#define DMAMUX1_CHANNEL11          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL11_BASE_ADDRESS)
#define DMAMUX1_CHANNEL12          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL12_BASE_ADDRESS)
#define DMAMUX1_CHANNEL13          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL13_BASE_ADDRESS)
#define DMAMUX1_CHANNEL14          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL14_BASE_ADDRESS)
#define DMAMUX1_CHANNEL15          ((DMAMUX_CHANNEL_HW *) DMAMUX1_CHANNEL15_BASE_ADDRESS)

#define DMAMUX1_REQUESTGEN0        ((DMAMUX_REQUESTGEN_HW *) DMAMUX1_REQUESTGEN0_BASE_ADDRESS)
#define DMAMUX1_REQUESTGEN1        ((DMAMUX_REQUESTGEN_HW *) DMAMUX1_REQUESTGEN1_BASE_ADDRESS)
#define DMAMUX1_REQUESTGEN2        ((DMAMUX_REQUESTGEN_HW *) DMAMUX1_REQUESTGEN2_BASE_ADDRESS)
#define DMAMUX1_REQUESTGEN3        ((DMAMUX_REQUESTGEN_HW *) DMAMUX1_REQUESTGEN3_BASE_ADDRESS)

#define DMAMUX1_CHANNELSTATUS      ((DMAMUX_CHANNELSTATUS_HW *) DMAMUX1_CHANNELSTATUS_ADDRESS)
#define DMAMUX1_REQUESTGENSTATUS   ((DMAMUX_REQUESTGENSTATUS_HW *) DMAMUX1_REQUESTGENSTATUS_ADDRESS)

#define DBGMCU                     ((DBGMCU_HW *) DBGMCU_BASE_ADDRESS)


#ifdef __cplusplus
}
#endif


#endif      /* _PERIPHERALS_DEFS_H_ */

/*==================================================================================
|                                 END OF FILE                                
===================================================================================*/
