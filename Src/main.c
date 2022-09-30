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

float get[7] = {0};
float state[3][2] = {0};
float angle[3] = {0};
float dt = 0;

int main(void)
{
	fpu_enable();
	float err[6] = {0};
	uint16_t rate = 200;
	dt = 1.0 / (float)rate;
	/* Baud rate: 460800 */
	usart3_default_init();
	/* Fast mode I2C
	 * Packet size: 146 bits
	 * Cost time / packet: packet size / 400 kHz = 0.3ms
	 */
	i2c1_master_default_init();
	GPIOB->MODER |= GPIO_MODE_OUTPUT;
	GPIOB->BSRR |= 1U << 16;
	tim1_interrupt_init(rate);
	mpu6050_power_on();
	// mpu6050_selftest(err);
	// printf("Change from Factory Trim:\r\n");

	// for (int i = 0; i < 6; i++) {
	// 	printf("%.2f, ", err[i]);
	// }

	// printf("\r\n");
	mpu6050_init(rate, 2, 250);
	flag_get = true;
	while (1) {

	}

	return 0;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	/* Check whether update interrupt is pending */
	if ((TIM1->SR & TIM_SR_UIF) != 0) {
		/* Clear the update interrupt flag */
		TIM1->SR &= ~TIM_SR_UIF;
		/* enable get data flag */
		if (flag_get) {
			/* 500 us */
			mpu6050_get_all(get);
			/* 56 us */
			mpu6050_static_attitude(get, angle);
			/* 106 us */
			mpu6050_kalman(state, angle, &get[4], dt);
			GPIOB->BSRR |= 1U;
			// for (int i = 0; i < 3; i++) 
			// 	printf("%.3f,", get[i]);
				// printf("%d,", (int)(get[i] * 10000));

			// for (int i = 0; i < 2; i++) 
			// 	printf("%.2f,", state[i][0]);
				// printf("%d,", (int)(state[i][0] * 10000));

			// printf("%.2f", state[2][0]);
			// printf("%d", (int)(state[2][0] * 10000));
			// putchar('\n');
			// printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
			// 	get[0], get[1], get[2],
			// 	state[0][0], state[1][0], state[2][0]);
			// printf("%d,%d,%d,%d,%d,%d\n",
			// 	(int)get[0], (int)get[1], (int)get[2],
			// 	(int)state[0][0], (int)state[1][0], (int)state[2][0]);
			// printf("%f,%f,%f,%f,%f,%f\n",
			// 	get[0], get[1], get[2],
			// 	state[0][0], state[1][0], state[2][0]);

			GPIOB->BSRR |= 1U << 16;
		}
	}
}