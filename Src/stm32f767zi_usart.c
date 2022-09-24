/**
 * @file stm32f767zi_usart.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief usart/uart
 * @date 2022-08-10
 */

#include "stm32f767zi_usart.h"

void usart2_default_init(void)
{
	// Enable clock access to GPIOD.
	RCC->AHB1ENR |= GPIOD_CLK_EN;
	// Set PD5 (Tx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART2_TX_POS;
	// Set PD6 (Rx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART2_RX_POS;
	// Set alternate function to USART. GPIO_AFRH = AF7(0b0111) | AF7 << 4
	GPIOD->AFR[0] |= (USART2_AF << 20) | (USART2_AF << 24);
	// Enable clock to USART3 module
	RCC->APB1ENR |= USART2_CLK_EN;
	// Configure USART parameters (TE, RE, PS, PCE, M, STOP)
	USART2->CR1 |= (1U << USART_CR1_TE_POS) | (1U << USART_CR1_RE_POS);
	// Set baud rate
	USART2->BRR = (PERIPH_CLK + (115200 / 2U)) / 115200;
	// enable USART3
	USART2->CR1 |= 1U << USART_CR1_UE_POS;
}

// word length: 8
// transmit enable: enable
// parity: none
// stop bits: 1
// baud rate: 115200
void usart3_default_init(void)
{
	// Enable clock access to GPIOD.
	RCC->AHB1ENR |= GPIOD_CLK_EN;
	// Set PD8 (Tx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART3_TX_POS;
	// Set PD9 (Rx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART3_RX_POS;
	// Set alternate function to USART. GPIO_AFRH = AF7(0b0111) | AF7 << 4
	GPIOD->AFR[1] |= (USART3_AF << 4) | USART3_AF;
	systick_delay_ms(10);
	// Enable clock to USART3 module
	RCC->APB1ENR |= USART3_CLK_EN;
	// Configure USART parameters (TE, RE, PS, PCE, M, STOP)
	USART3->CR1 |= (1U << USART_CR1_TE_POS) | (1U << USART_CR1_RE_POS);
	// Set baud rate
	USART3->BRR = (PERIPH_CLK + (115200 / 2U)) / 115200;
	// enable USART3
	USART3->CR1 |= 1U << USART_CR1_UE_POS;
}

void usart3_interrupt_default_init(void)
{
	// Enable clock access to GPIOD.
	RCC->AHB1ENR |= GPIOD_CLK_EN;
	// Set PD8 (Tx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART3_TX_POS;
	// Set PD9 (Rx) mode to alternate function.
	GPIOD->MODER |= GPIO_MODE_ALTFN << 2 * USART3_RX_POS;
	// Set alternate function to USART. GPIO_AFRH = AF7(0b0111) | AF7 << 4
	GPIOD->AFR[1] |= (USART3_AF << 4) | USART3_AF;
	// Enable clock to USART3 module
	RCC->APB1ENR |= USART3_CLK_EN;
	// Configure USART parameters (TE, RE, PS, PCE, M, STOP)
	USART3->CR1 |= (1U << USART_CR1_TE_POS) | (1U << USART_CR1_RE_POS); 
	// Set baud rate
	USART3->BRR = (PERIPH_CLK + (115200 / 2U)) / 115200;
	// enable USART3
	USART3->CR1 |= 1U << USART_CR1_UE_POS;
	// Enable USART TXE interrupt
	USART3->CR1 |= 1U << USART_CR1_TXEIE_POS;
	// Enable USART RXNE interrupt
	USART3->CR1 |= 1 << USART_CR1_RXNEIE_POS;
	// Enable USART interrupt in NVIC
	NVIC->ISER[(((uint32_t)USART3_IRQn) >> 5UL)] =
		(uint32_t)(1UL << (((uint32_t)USART3_IRQn) & 0x1FUL));
}

void usart_write(USART_reg_t *USARTx, uint8_t value)
{
	// Make sure transmit data register is empty
	while (!(USARTx->ISR & USART_ISR_TXE))
		;
	// Write value into transmit data register
	USARTx->TDR = value;
}

uint8_t usart_read(USART_reg_t *USARTx)
{
	while (!(USARTx->ISR & USART_ISR_RXNE))
		;
	return USARTx->RDR & 0xFFU;
}

int __io_putchar(int ch)
{
	usart_write(USART3, ch);
	return ch;
}

int __io_getchar(void)
{
	return usart_read(USART3);
}