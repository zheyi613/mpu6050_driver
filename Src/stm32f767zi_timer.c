/**
 * @file stm32f767zi_timer.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief timer
 * @date 2022-08-18
 */

#include "stm32f767zi_timer.h"

void tim1_1Hz_init(void)
{
	// Enable clock access to TIM1
	RCC->APB2ENR |= TIM1_CLK_EN;
	// Set the prescaler
	TIM1->PSC = 1600 - 1; // 16000000 / 1600 = 10000
	// Set auto-reload value
	TIM1->ARR = 10000 - 1;
	// Enable timer
	TIM1->CR1 |= TIM_CR1_CEN;
}

void tim1_1Hz_interrupt_init(void)
{
	// Enable clock access to TIM1
	RCC->APB2ENR |= TIM1_CLK_EN;
	// Set the prescaler
	TIM1->PSC = 1600 - 1; // 16000000 / 1600 = 10000
	// Set auto-reload value
	TIM1->ARR = 10000 - 1;
	// Enable TIM1 update interrupt
	TIM1->DIER |= TIM_DIER_UIE;
	// Enable TIM1 update interrupt in NVIC
	NVIC->ISER[(((uint32_t)TIM1_UP_TIM10_IRQn) >> 5UL)] =
		(uint32_t)(1UL << (((uint32_t)TIM1_UP_TIM10_IRQn) & 0x1FUL));
	// Enable timer
	TIM1->CR1 |= TIM_CR1_CEN;
	// Force update generation
	TIM1->EGR |= TIM_EGR_UG;
}

void tim1_interrupt_init(uint16_t freq)
{
	// Enable clock access to TIM1
	RCC->APB2ENR |= TIM1_CLK_EN;
	// Set the prescaler
	TIM1->PSC = 1600 - 1; // 16000000 / 1600 = 10000
	// Set auto-reload value
	TIM1->ARR = (10000 / freq) - 1;
	// Enable TIM1 update interrupt
	TIM1->DIER |= TIM_DIER_UIE;
	// Enable TIM1 update interrupt in NVIC
	NVIC->ISER[(((uint32_t)TIM1_UP_TIM10_IRQn) >> 5UL)] =
		(uint32_t)(1UL << (((uint32_t)TIM1_UP_TIM10_IRQn) & 0x1FUL));
	// Enable timer
	TIM1->CR1 |= TIM_CR1_CEN;
	// Force update generation
	TIM1->EGR |= TIM_EGR_UG;
}