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

data_t get[7] = {0};
float state[3][2] = {0};
float angle[3] = {0};
float dt = 0;
uint8_t buff[80];
uint8_t tx_size = 0;

int main(void)
{
	fpu_enable();
	float err[6] = {0};
	uint16_t rate = 250;
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
	mpu6050_power_on();
	// mpu6050_selftest(err);
	// printf("Change from Factory Trim:\r\n");

	// for (int i = 0; i < 6; i++) {
	// 	printf("%.2f, ", err[i]);
	// }

	// printf("\r\n");
	mpu6050_init(rate, 2, 250);
	tim1_interrupt_init(rate);

	while (1)
		;

	return 0;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	/* Check whether update interrupt is pending */
	if ((TIM1->SR & TIM_SR_UIF) != 0) {
		/* Clear the update interrupt flag */
		TIM1->SR &= ~TIM_SR_UIF;
		/* Get mpu6050 data */
		mpu6050_get_all(get);				/* 500 us */
		mpu6050_static_attitude(get->accel, angle);		/* 56 us */
		mpu6050_kalman(state, angle, get->ang_vel, dt); /* 106 us */
		/* 1500 us */
		tx_size = snprintf((char *)buff, sizeof(buff),
				   "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
				   get->accel[0], get->accel[1], get->accel[2],
				   state[0][0], state[1][0], state[2][0]);
		/* 615 us */
		// tx_size = 0;
		// uint8_t *tmp;
		// int i = 0, j = 0;
		// for (i = 0; i < 3; i++) {
		// 	tmp = (uint8_t *)&get[i];
		// 	for (j = 0; j < 4; j++)
		// 		buff[tx_size++] = tmp[j];
		// 	buff[tx_size++] = (uint8_t)',';
		// }
		// for (i = 0; i < 2; i++) {
		// 	tmp = (uint8_t *)&state[i][0];
		// 	for (j = 0; j < 4; j++)
		// 		buff[tx_size++] = tmp[j];
		// 	buff[tx_size++] = (uint8_t)',';
		// }
		// tmp = (uint8_t *)&state[2][0];
		// for (j = 0; j < 4; j++)
		// 	buff[tx_size++] = tmp[j];
		// buff[tx_size++] = (uint8_t)'\n';
		usart_w_arr(USART3, buff, tx_size);
	}
}