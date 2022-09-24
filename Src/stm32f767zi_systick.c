/**
 * @file stm32f767zi_systick.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief systick
 * @date 2022-08-18
 */

#include "stm32f767zi_systick.h"

void systick_delay_ms(int delay)
{
	// Reload the number of clocks per millisecond
	SYST->RVR = SYSTICK_LOAD_VAL;
	// Clear the current value register
	SYST->CVR = 0;
	// Select clock source and enable systick
	SYST->CSR = SYST_CSR_ENABLE | SYST_CSR_CLKSRC;

	for (int i = 0; i < delay; i++)
		while (!(SYST->CSR & SYST_CSR_COUNTFLAG))
			;

	SYST->CSR = 0;
}

void systick_1hz_interrupt(void)
{
	// Reload the number of clocks per millisecond
	SYST->RVR = ONE_SEC_LOAD_VAL;
	// Clear the current value register
	SYST->CVR = 0;
	// Select clock source and enable systick
	SYST->CSR = SYST_CSR_ENABLE | SYST_CSR_CLKSRC;
	// Enable systick interrupt
	SYST->CSR |= SYST_CSR_TICKINT;
}
