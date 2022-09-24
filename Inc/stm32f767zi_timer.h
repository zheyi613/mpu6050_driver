/**
 * @file stm32f767zi_timer.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief timer
 * @date 2022-08-18
 */

#ifndef __STM32F767ZI_TIMER_H
#define __STM32F767ZI_TIMER_H

#include "stm32f767zi_hal.h"

#define TIM_CR1_CEN             1U
#define TIM_SR_UIF              1U
#define TIM_DIER_UIE            1U
#define TIM_EGR_UG              1U


#define TIM1_UP_TIM10_IRQn      25U

void tim1_1Hz_init(void);
void tim1_1Hz_interrupt_init(void);
void tim1_interrupt_init(uint16_t freq);

#endif