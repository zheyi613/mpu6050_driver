/**
 * @file stm32f767zi_hal.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief stm32f767zi hardware abstract layer
 * @date 2022-08-10
 */

#ifndef __STM32F767ZI_HAL_H
#define __STM32F767ZI_HAL_H

#include <stdint.h>

#define __IO                    volatile
#define __IOM                   volatile        // Define read/write structure member permissions
#define __IM                    volatile        // Define read/      structure member permissions
#define __OM                    volatile        // Define     /write structure member permissions

#define WRITE_REG(REG, VAL)             REG = VAL
#define MODIFY_REG(REG, MASK, VAL)      WRITE_REG(REG, (REG & ~(MASK)) | (VAL))

#define PERIPH_CLK              16000000UL                      // Frequency of peripheral (16MHz)
#define PERIPH_BASE             0x40000000UL                    // Base of 512-Mbyte Block 2 Peripherals 
#define AHB1_BASE               (PERIPH_BASE + 0x00020000UL)    // Advanced High Performance Bus 1

#define GPIOA_BASE              (AHB1_BASE)                     // Gereral Purpose Input/Output A
#define GPIOB_BASE              (AHB1_BASE + 0x0400UL)          // Gereral Purpose Input/Output B
#define GPIOC_BASE              (AHB1_BASE + 0x0800UL)          // Gereral Purpose Input/Output C
#define GPIOD_BASE              (AHB1_BASE + 0x0C00UL)          // Gereral Purpose Input/Output D
#define GPIOE_BASE              (AHB1_BASE + 0x1000UL)          // Gereral Purpose Input/Output E
#define GPIOF_BASE              (AHB1_BASE + 0x1400UL)          // Gereral Purpose Input/Output F
#define GPIOG_BASE              (AHB1_BASE + 0x1800UL)          // Gereral Purpose Input/Output G

#define RCC_BASE                (AHB1_BASE + 0x3800UL)          // Reset Clock Control

#define APB1_BASE               PERIPH_BASE

#define SPI3_BASE               (APB1_BASE + 0x3C00UL)          // SPI 3
#define USART2_BASE             (APB1_BASE + 0x4400UL)          // USART 2
#define USART3_BASE             (APB1_BASE + 0x4800UL)          // USART 3
#define I2C1_BASE               (APB1_BASE + 0x5400UL)          // I2C 1
#define I2C2_BASE               (APB1_BASE + 0x5800UL)          // I2C 2

#define APB2_BASE               (PERIPH_BASE + 0x00010000UL)    // Advanced High Performance Bus 2

#define TIM1_BASE               (APB2_BASE)                     // Timer 1
#define ADC1_BASE               (APB2_BASE + 0x2000UL)          // Analog to Digital converter
#define SPI1_BASE               (APB2_BASE + 0x3000UL)          // Serial peripheral interface
#define SYSCFG_BASE             (APB2_BASE + 0x3800UL)          // System configuration controller
#define EXTI_BASE               (APB2_BASE + 0x3C00UL)          // Extended interrupts and events controller

#define SCS_BASE                0xE000E000UL                    // System Control Space

#define SYST_BASE               (SCS_BASE + 0x0010UL)           // SysTick
#define NVIC_BASE               (SCS_BASE + 0x0100UL)           // Nested vectored interrupt controller

#define GPIOA_CLK_EN            1U              // Enable GPIOA clock at RCC AHB1EN PIN0
#define GPIOB_CLK_EN            (1U << 1)       // Enable GPIOB clock at RCC AHB1EN PIN1
#define GPIOC_CLK_EN            (1U << 2)       // Enable GPIOC clock at RCC AHB1EN PIN2
#define GPIOD_CLK_EN            (1U << 3)       // Enable GPIOD clock at RCC AHB1EN PIN3
#define GPIOE_CLK_EN            (1U << 4)       // Enable GPIOE clock at RCC AHB1EN PIN4
#define GPIOF_CLK_EN            (1U << 5)       // Enable GPIOF clock at RCC AHB1EN PIN5
#define GPIOG_CLK_EN            (1U << 6)       // Enable GPIOG clock at RCC AHB1EN PIN6

#define USART2_CLK_EN           (1U << 17)      // Enable USART3 clock at RCC APB1EN PIN17
#define USART3_CLK_EN           (1U << 18)      // Enable USART3 clock at RCC APB1EN PIN18

#define TIM1_CLK_EN             1U              // Enable timer 1 clock

#define ADC1_CLK_EN             (1U << 8)       // Enable PA4 ADC clock at RCC APB2EN PIN8
#define I2C1_CLK_EN             (1U << 21)      // Enable I2C 1 clock
#define I2C2_CLK_EN             (1U << 22)      // Enable I2C 2 clock
#define SPI1_CLK_EN             (1U << 12)      // Enable SPI 1 clock
#define SPI3_CLK_EN             (1U << 15)      // Enable SPI 3 clock

#define SYSCFG_CLK_EN           (1U << 14)      // Enable system configuration controller clock

typedef struct {
        __IO uint32_t MODER;            // GPIO port mode register                              Address offset: 0x00
        __IO uint32_t OTYPER;           // GPIO port output type register                       Address offset: 0x04
        __IO uint32_t OSPEEDR;          // GPIO port output speed register                      Address offset: 0x08
        __IO uint32_t PUPDR;            // GPIO port pull-up/pull-down register                 Address offset: 0x0C
        __IO uint32_t IDR;              // GPIO port input data register                        Address offset: 0x10
        __IO uint32_t ODR;              // GPIO port output data register                       Address offset: 0x14
        __IO uint32_t BSRR;             // GPIO port bit set/reset register                     Address offset: 0x18
        __IO uint32_t LCKR;             // GPIO port configuration lock register                Address offset: 0x1C
        __IO uint32_t AFR[2];           // AFR[0]: GPIO alternate function low register         Address offset: 0x20
                                        // AFR[1]: GPIO alternate function high register        Address offset: 0x24
} GPIO_reg_t;

typedef struct {
        __IO uint32_t DUMMY[12];        // RCC ...
        __IO uint32_t AHB1ENR;          // RCC AHB1 peripheral clock register                   Address offset: 0x30
        __IO uint32_t DUMMY2[3];        // RCC ...
        __IO uint32_t APB1ENR;          // RCC APB1 peripheral clock enable register            Address offset: 0x40
        __IO uint32_t APB2ENR;          // RCC APB2 peripheral clock enable register            Address offset: 0x44
} RCC_reg_t;

typedef struct {
        __IO uint32_t CR1;              // USART control register 1                             Address offset: 0x00
        __IO uint32_t CR2;              // USART control register 2                             Address offset: 0x04
        __IO uint32_t CR3;              // USART control register 3                             Address offset: 0x08
        __IO uint32_t BRR;              // USART baud rate register                             Address offset: 0x0C
        __IO uint32_t GTPR;             // USART guard time and prescaler register              Address offset: 0x10
        __IO uint32_t RTOR;             // USART receiver timeout register                      Address offset: 0x14
        __IO uint32_t RQR;              // USART request register                               Address offset: 0x18
        __IO uint32_t ISR;              // USART interrupt and status register                  Address offset: 0x1C
        __IO uint32_t ICR;              // USART interrupt flag clear register                  Address offset: 0x20
        __IO uint32_t RDR;              // USART receive data register                          Address offset: 0x24
        __IO uint32_t TDR;              // USART transmit data register                         Address offset: 0x28
} USART_reg_t;

typedef struct {
        __IO uint32_t SR;               // ADC sattus register                                  Address offset: 0x00
        __IO uint32_t CR1;              // ADC control register 1                               Address offset: 0x04
        __IO uint32_t CR2;              // ADC control register 2                               Address offset: 0x08
        __IO uint32_t SMPR1;            // ADC sample time register 1                           Address offset: 0x0C
        __IO uint32_t SMPR2;            // ADC sample time register 2                           Address offset: 0x10
        __IO uint32_t DUMMY[6];         // ...
        __IO uint32_t SQR1;             // ADC regular sequence register 1                      Address offset: 0x2C
        __IO uint32_t SQR2;             // ADC regular sequence register 2                      Address offset: 0x30
        __IO uint32_t SQR3;             // ADC regular sequence register 3                      Address offset: 0x34
        __IO uint32_t DUMMY2[5];        // ..
        __IO uint32_t DR;               // ADC regular data register                            Address offset: 0x4C
} ADC_reg_t;

typedef struct {
        __IO uint32_t CR1;              // TIM control register 1                               Address offset: 0x00
        __IO uint32_t CR2;              // TIM control register 2                               Address offset: 0x04
        __IO uint32_t SMCR;             // TIM slave mode control register                      Address offset: 0x08
        __IO uint32_t DIER;             // TIM DMA/interrupt enable register                    Address offset: 0x0C
        __IO uint32_t SR;               // TIM status register                                  Address offset: 0x10
        __IO uint32_t EGR;              // TIM event generation register                        Address offset: 0x14
        __IO uint32_t DUMMY[4];         // ...
        __IO uint32_t PSC;              // TIM prescaler                                        Address offset: 0x28
        __IO uint32_t ARR;              // TIM auto-reload register                             Address offset: 0x2C
} TIM_reg_t;

typedef struct {
        __IO uint32_t MEMRMP;           // SYSCFG memory remap register                         Address offset: 0x00
        __IO uint32_t PMC;              // SYSCFG peripheral mode configuration register        Address offset: 0x04
        __IO uint32_t EXTICR[4];        // SYSCFG external interrupt configuration register x   Address offset: 0x08
} SYSCFG_reg_t;

typedef struct {
        __IO uint32_t IMR;              // Interrupt mask register                              Address offset: 0x00
        __IO uint32_t EMR;              // Event mask register                                  Address offset: 0x04
        __IO uint32_t RTSR;             // Rising trigger selection register                    Address offset: 0x08
        __IO uint32_t FTSR;             // Falling trigger selection register                   Address offset: 0x0C
        __IO uint32_t SWIER;            // Software interrupt event register                    Address offset: 0x10
        __IO uint32_t PR;               // Pending register                                     Address offset: 0x14            
} EXTI_reg_t;

typedef struct {
        __IO uint32_t CR1;              // Control register 1                                   Address offset: 0x00
        __IO uint32_t CR2;              // Control register 2                                   Address offset: 0x04
        __IO uint32_t OAR1;             // Own address 1 register                               Address offset: 0x08
        __IO uint32_t OAR2;             // Own address 2 register                               Address offset: 0x0C
        __IO uint32_t TIMINGR;          // Timing register                                      Address offset: 0x10
        __IO uint32_t TIMEOUTR;         // Timeout register                                     Address offset: 0x14
        __IO uint32_t ISR;              // Interrupt and status register                        Address offset: 0x18
        __IO uint32_t ICR;              // Interrupt clear register                             Address offset: 0x1C
        __IO uint32_t PECR;             // Packet error checking register                       Address offset: 0x20
        __IO uint32_t RXDR;             // Receive data register                                Address offset: 0x24
        __IO uint32_t TXDR;             // Transmit data register                               Address offset: 0x28
} I2C_reg_t;

typedef struct {
        __IO uint32_t CR1;              // Control register 1
        __IO uint32_t CR2;              // Control register 2
        __IO uint32_t SR;               // Status register
        __IO uint32_t DR;               // Data register
        __IO uint32_t CRCPR;            // CRX polynomial register
        __IO uint32_t RXCRCR;           // Rx CRC register
        __IO uint32_t TXCRCR;           // Tx CRC register
} SPI_reg_t;

typedef struct {
        __IOM uint32_t ISER[8U];        /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
        __IOM uint32_t ICER[8U];        /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
        __IOM uint32_t ISPR[8U];        /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
        __IOM uint32_t ICPR[8U];        /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
        __IOM uint32_t IABR[8U];        /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
        __IOM uint8_t  IP[240U];        /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
        __OM  uint32_t STIR;            /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_reg_t;

typedef struct {
        __IOM uint32_t CSR;             // SysTick control and status register                  Address offset: 0x00
        __IOM uint32_t RVR;             // SysTick reload value register                        Address offset: 0x04
        __IOM uint32_t CVR;             // SysTick current value register                       Address offset: 0x08
        __IM uint32_t CALIB;            // SysTick calibration value register                   Address offset: 0x0C
} SYST_reg_t;

#define GPIOA           ((GPIO_reg_t *)GPIOA_BASE)
#define GPIOB           ((GPIO_reg_t *)GPIOB_BASE)
#define GPIOC           ((GPIO_reg_t *)GPIOC_BASE)
#define GPIOD           ((GPIO_reg_t *)GPIOD_BASE)
#define GPIOE           ((GPIO_reg_t *)GPIOE_BASE)
#define GPIOF           ((GPIO_reg_t *)GPIOF_BASE)
#define GPIOG           ((GPIO_reg_t *)GPIOG_BASE)
#define RCC             ((RCC_reg_t *)RCC_BASE)
#define USART2          ((USART_reg_t *)USART2_BASE)
#define USART3          ((USART_reg_t *)USART3_BASE)
#define ADC1            ((ADC_reg_t *)ADC1_BASE)
#define TIM1            ((TIM_reg_t *)TIM1_BASE)
#define SYSCFG          ((SYSCFG_reg_t *)SYSCFG_BASE)
#define EXTI            ((EXTI_reg_t *)EXTI_BASE)
#define I2C1            ((I2C_reg_t *)I2C1_BASE)
#define I2C2            ((I2C_reg_t *)I2C2_BASE)
#define SPI1            ((SPI_reg_t *)SPI1_BASE)
#define SPI3            ((SPI_reg_t *)SPI3_BASE)

#define NVIC            ((NVIC_reg_t *)NVIC_BASE)
#define SYST            ((SYST_reg_t *)SYST_BASE)
#define CPACR           *((volatile uint32_t *)0xE000ED88UL)

#define fpu_enable()                                    \
        do {                                            \
                CPACR = (3U << 20) | (3U << 22);        \
        } while (0)

#endif