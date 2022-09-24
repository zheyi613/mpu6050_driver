/**
 * @file stm32f767zi_usart.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief usart/uart
 * @date 2022-08-10
 */

#ifndef __STM32F767ZI_USART_H
#define __STM32F767ZI_USART_H

#include "stm32f767zi_gpio.h"
#include "stm32f767zi_hal.h"
#include "stm32f767zi_systick.h"

// Control register 1
#define USART_CR1_UE_POS        0
#define USART_CR1_UE_MASK       (1U << USART_CR1_UE_POS)
#define USART_CR1_RE_POS        2
#define USART_CR1_RE_MASK       (1U << USART_CR1_RE_POS)
#define USART_CR1_TE_POS        3
#define USART_CR1_TE_MASK       (1U << USART_CR1_TE_POS)
#define USART_CR1_RXNEIE_POS    5
#define USART_CR1_RXNEIE_MASK   (1U << USART_CR1_RXNEIE_POS)
#define USART_CR1_TXEIE_POS     7
#define USART_CR1_TXEIE_MASK    (1U << USART_CR1_TXEIE_POS)
#define USART_CR1_PS_POS        9
#define USART_CR1_PS_MASK       (1U << USART_CR1_PS_POS)
#define USART_CR1_PCE_POS       10
#define USART_CR1_PCE_MASK      (1U << USART_CR1_PCE_POS)
#define USART_CR1_M0_POS        12
#define USART_CR1_M0_MASK       (1U << USART_CR1_M0_POS)
#define USART_CR1_M1_POS        28
#define USART_CR1_M1_MASK       (1U << USART_CR1_M1_POS)
// Control register 2
#define USART_CR2_STOP_POS      12
#define USART_CR2_STOP_MASK     (3U << USART_CR2_STOP_POS)
// Interrupt and status register
#define USART_ISR_RXNE          (1U << 5)
#define USART_ISR_TXE           (1U << 7)
// word length
#define USART_WORDLEN_7         7
#define USART_WORDLEN_8         8
#define USART_WORDLEN_9         9
// stop bits
#define USART_STOPBITS_1        0
#define USART_STOPBITS_0_5      1
#define USART_STOPBITS_2        2
#define USART_STOPBITS_1_5      3

#define USART2_AF               7U
#define USART2_TX_POS           5               // PD5
#define USART2_RX_POS           6               // PD6

#define USART3_AF               7U
#define USART3_TX_POS           8               // PD8
#define USART3_RX_POS           9               // PD9

#define USART3_IRQn             39U

void usart2_default_init(void);
void usart3_default_init(void);
void usart3_interrupt_default_init(void);
void usart_write(USART_reg_t *USARTx,uint8_t value);
uint8_t usart_read(USART_reg_t *USARTx);

#endif