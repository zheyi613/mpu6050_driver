/**
 * @file stm32f767zi_systick.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief systick
 * @date 2022-08-18
 */

#ifndef __STM32F767ZI_SYSTICK_H
#define __STM32F767ZI_SYSTICK_H

#include "stm32f767zi_hal.h"

#define SYST_CSR_ENABLE         (1U << 0)       // Enable the counter
#define SYST_CSR_TICKINT        (1U << 1)       // Enable SysTick exception request
#define SYST_CSR_CLKSRC         (1U << 2)       // Select processor clock source
#define SYST_CSR_COUNTFLAG      (1U << 16)      // Return 1 if timer counted to 0 since last time this was read

#define SYSTICK_LOAD_VAL        15999           // 16000000Hz / (SYSTICK_LOAD_VAL + 1) = 1000Hz
#define ONE_SEC_LOAD_VAL        15999999        // 16000000Hz / (ONE_SEC_LOAD_VAL + 1) = 1Hz

void systick_delay_ms(int delay);
void systick_1hz_interrupt(void);

#endif