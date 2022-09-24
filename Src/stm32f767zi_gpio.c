/**
 * @file stm32f767zi_gpio.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief GPIO
 * @date 2022-08-10
 */

#include "stm32f767zi_gpio.h"

void gpio_set_mode(GPIO_reg_t *GPIOx, uint32_t position, uint32_t mode)
{
	GPIOx->MODER &= ~(0x03 << (position * 2));
	GPIOx->MODER |= mode << (position * 2);
}