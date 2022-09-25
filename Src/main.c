/**
 * @file main.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief
 * @date 2022-09-23
 */

#include "mpu6050.h"
#include "stm32f767zi_gpio.h"
#include "stm32f767zi_hal.h"
#include "stm32f767zi_i2c.h"
#include "stm32f767zi_systick.h"
#include "stm32f767zi_timer.h"
#include "stm32f767zi_usart.h"
#include <stdbool.h>
#include <stdio.h>

bool flag_get = false;

int main(void)
{
	fpu_enable();
	float get[7] = {0};
	float err[6] = {0};
	float state[3][2] = {0};
	float angle[3] = {0};
	uint8_t rate = 250;
	float dt = 1.0 / (float)rate;
	
	usart3_default_init();
	i2c1_master_default_init();
	tim1_interrupt_init(rate);
	mpu6050_power_on();

	// mpu6050_selftest(err);
	// printf("Change from Factory Trim:\r\n");

	// for (int i = 0; i < 6; i++) {
	// 	printf("%.2f, ", err[i]);
	// }

	// printf("\r\n");
	mpu6050_init(rate, 2, 250);

	while (1) {
		if (flag_get) {
			mpu6050_get_all(get);
			mpu6050_static_attitude(get, angle);
			mpu6050_kalman(state, angle, &get[4], dt);

			for (int i = 0; i < 3; i++) {
				printf("%.3f,", get[i]);
			}

			for (int i = 0; i < 2; i++) {
				printf("%.2f,", state[i][0]);
			}

			printf("%.2f", state[2][0]);
			printf("\n");
			flag_get = false;
		}
	}

	return 0;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	// Check whether update interrupt is pending
	if ((TIM1->SR & TIM_SR_UIF) != 0) {
		// Clear the update interrupt flag
		TIM1->SR &= ~TIM_SR_UIF;
		// enable get data flag
		flag_get = true;
	}
}