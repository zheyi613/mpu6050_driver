/**
 * @file stm32f767zi_adc.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief adc
 * @date 2022-08-15
 */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f767zi_hal.h"
#include "stm32f767zi_gpio.h"
// Status register
#define ADC_SR_EOC      (1U << 1)   
// Control register 1
#define ADC_CR1_EOCIE   (1U << 5)   
// Control register 2
#define ADC_CR2_ADON    (1U << 0)
#define ADC_CR2_CONT    (1U << 1)
#define ADC_CR2_SWSTART (1U << 30)
// Regular sequence register 1
#define ADC_SEQLEN_1    0
// Regular sequence register 3
#define CH4_RANK1       (1U << 2)

#define ADC_IRQn        18U

void pa4_adc1_init(void);
void pa4_adc1_interrupt_init(void);
void adc1_start_conversion(void);
uint32_t adc1_get_data(void);

#endif