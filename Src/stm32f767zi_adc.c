/**
 * @file stm32f767zi_adc.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief adc
 * @date 2022-08-15
 */

#include "stm32f767zi_adc.h"

void pa4_adc1_init(void)
{
	/*      Configure the ADC GPIO pin      */

	// Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOA_CLE_EN;
	// Set PA4 mode to analog mode
	GPIOA->MODER |= (GPIO_MODE_ANALOG << 8);

	/*      Configure the ADC module        */

	// Enable clock access to the ADC module
	RCC->APB2ENR |= ADC1_CLK_EN;
	// Set conversion to continuous
	// ADC1->CR2 |= ADC_CR2_CONT;
	// Set sequence length to 1
	ADC1->SQR1 |= ADC_SEQLEN_1;
	// Set adc channel
	ADC1->SQR3 |= CH4_RANK1;
	// Enable adc module
	ADC1->CR2 |= ADC_CR2_ADON;
}

void pa4_adc1_interrupt_init(void)
{
	/*      Configure the ADC GPIO pin      */

	// Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOA_CLE_EN;
	// Set PA4 mode to analog mode
	GPIOA->MODER |= (GPIO_MODE_ANALOG << 8);

	/*      Configure the ADC module        */

	// Enable clock access to the ADC module
	RCC->APB2ENR |= ADC1_CLK_EN;
	// Set conversion to continuous
	ADC1->CR2 |= ADC_CR2_CONT;
	// Set sequence length to 1
	ADC1->SQR1 |= ADC_SEQLEN_1;
	// Set adc channel
	ADC1->SQR3 |= CH4_RANK1;
	// Enable adc module
	ADC1->CR2 |= ADC_CR2_ADON;

	/*      Configure the ADC interrupt     */

	// Enable end-of-conversion interrupt
	ADC1->CR1 |= ADC_CR1_EOCIE;
	// Enable ADC interrupt in NVIC
	NVIC->ISER[(((uint32_t)ADC_IRQn) >> 5UL)] =
		(uint32_t)(1UL << (((uint32_t)ADC_IRQn) & 0x1FUL));
}

void adc1_start_conversion(void)
{
	// Start conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint32_t adc1_get_data(void)
{
	// Wait for End-of-Conversion flag
	while (!(ADC1->SR & ADC_SR_EOC))
		;
	// Read results
	return ADC1->DR;
}