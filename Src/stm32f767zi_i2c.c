/**
 * @file stm32f767zi_i2c.c
 * @author ufo880613@gmail.com (ufo880613@gmail.com)
 * @brief Inter-intergrated circuit
 * @date 2022-08-20
 */

#include "stm32f767zi_i2c.h"

static void i2c_set_ownAddress1(I2C_reg_t *I2Cx, uint32_t addr);

void i2c1_master_default_init(void)
{
	/*      Configure SCL/SDA line        */

	// Enable clock for GPIOB
	RCC->AHB1ENR |= GPIOB_CLK_EN;
	// Configure PB6(SCL) and PB9(SDA) mode to alternative function
	GPIOB->MODER |= GPIO_MODE_ALTFN << (2 * I2C1_SCL_POS);
	GPIOB->MODER |= GPIO_MODE_ALTFN << (2 * I2C1_SDA_POS);
	// Select alternate function type as AF4 (I2C1_SCL/I2C1_SDA)
	GPIOB->AFR[I2C1_SCL_POS >> 3] |= I2C_AF << (4 * (I2C1_SCL_POS & 0x7U));
	GPIOB->AFR[I2C1_SDA_POS >> 3] |= I2C_AF << (4 * (I2C1_SDA_POS & 0x7U));
	// Set pin output type to open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OPEN_DRAIN << I2C1_SCL_POS;
	GPIOB->OTYPER |= GPIO_OTYPER_OPEN_DRAIN << I2C1_SDA_POS;
	// Enable internal pull-up register
	GPIOB->PUPDR |= GPIO_PUPDR_PULL_UP << (2 * I2C1_SCL_POS);
	GPIOB->PUPDR |= GPIO_PUPDR_PULL_UP << (2 * I2C1_SDA_POS);

	/*        Configure I2C parameters        */

	// Enable clock access to the I2C module
	RCC->APB1ENR |= I2C1_CLK_EN;
	// Disable I2C module
	I2C1->CR1 &= ~I2C_CR1_PE;
	// Set timing
	I2C1->TIMINGR |= I2C_TIMING;
	// Enable I2C module
	I2C1->CR1 |= I2C_CR1_PE;
}

void i2c2_slave_default_init(void)
{
	/*        Configure SCL/SDA line        */

	// Enable clock for GPIOB
	RCC->AHB1ENR |= GPIOB_CLK_EN;
	// Configure PB10(SCL) and PB11(SDA) mode to alternative function
	GPIOB->MODER |= GPIO_MODE_ALTFN << (2 * I2C2_SCL_POS);
	GPIOB->MODER |= GPIO_MODE_ALTFN << (2 * I2C2_SDA_POS);
	// Select alternate function type as AF4 (I2C2_SCL/I2C_SDA)
	GPIOB->AFR[I2C2_SCL_POS >> 3] |= I2C_AF << (4 * (I2C2_SCL_POS & 0x7U));
	GPIOB->AFR[I2C2_SDA_POS >> 3] |= I2C_AF << (4 * (I2C2_SDA_POS & 0x7U));
	// Set pin output type to open-drain
	GPIOB->OTYPER |= GPIO_OTYPER_OPEN_DRAIN << I2C2_SCL_POS;
	GPIOB->OTYPER |= GPIO_OTYPER_OPEN_DRAIN << I2C2_SDA_POS;
	// Enable internal pull-up register
	GPIOB->PUPDR |= GPIO_PUPDR_PULL_UP << (2 * I2C2_SCL_POS);
	GPIOB->PUPDR |= GPIO_PUPDR_PULL_UP << (2 * I2C2_SDA_POS);

	/*        Configure I2C parameters        */

	// Enable clock access to the I2C module
	RCC->APB1ENR |= I2C2_CLK_EN;
	// Enable I2c interrupt in the NVIC
	NVIC->ISER[(((uint32_t)I2C2_EV_IRQn) >> 5UL)] =
		(uint32_t)(1UL << (((uint32_t)I2C2_EV_IRQn) & 0x1FUL));
	// Disable I2C module
	I2C2->CR1 &= ~I2C_CR1_PE;
	// Set device address
	i2c_set_ownAddress1(I2C2, I2C_SLAVE_OWN_ADDRESS);
	// Enable own address 1
	I2C2->OAR1 |= I2C_OAR1_OA1EN;
	// Enable ADDR interrupt
	I2C2->CR1 |= I2C_CR1_ADDRIE;
	// Enable NACK interrupt
	I2C2->CR1 |= I2C_CR1_NACKIE;
	// Enable STOP interrupt
	I2C2->CR1 |= I2C_CR1_STOPIE;
	// Enable I2C module
	I2C2->CR1 |= I2C_CR1_PE;
}

static void i2c_set_ownAddress1(I2C_reg_t *I2Cx, uint32_t addr)
{
	addr = (addr & ~0x7FU) ? (addr | I2C_OAR1_OA1MODE) : (addr << 1);
	MODIFY_REG(I2Cx->OAR1, I2C_OAR1_OA1 | I2C_OAR1_OA1MODE, addr);
}

uint32_t i2c_get_addr_matchcode(I2C_reg_t *I2Cx)
{
	return (uint32_t)((I2Cx->ISR & I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_POS);
}

void i2c_write_reg(I2C_reg_t *I2Cx, const uint16_t addr, const uint8_t reg,
		   const uint8_t size, uint8_t *data)
{
	// Clear CR2
	I2Cx->CR2 &= 0U;
	// Set slave address
	I2Cx->CR2 |= (addr & ~0x7FU) ? (addr | I2C_CR2_ADD10) : (addr << 1);
	// Set transfer size
	I2Cx->CR2 |= (1 + (uint8_t)size) << I2C_CR2_NBYTES_POS;
	// Set automatic end mode
	I2Cx->CR2 |= I2C_CR2_AUTOEND;
	// Set data transmit register before start
	I2Cx->TXDR = reg;
	// Generate start condition
	I2Cx->CR2 |= I2C_CR2_START;
	// Loop until STOP flag is raised
	while (!(I2Cx->ISR & I2C_ISR_STOPF)) {
		// check TXIS flag value in ISR register
		if (I2Cx->ISR & I2C_ISR_TXIS)
			I2Cx->TXDR = *(data++);
	}
	// Clear stop flag
	I2C1->ICR |= I2C_ICR_STOPCF;
}

void i2c_read_reg(I2C_reg_t *I2Cx, const uint16_t addr, const uint8_t reg,
		  const uint8_t size, uint8_t *data)
{
	// Clear CR2
	I2Cx->CR2 &= 0U;
	// Set slave address
	I2Cx->CR2 |= (addr & ~0x7FU) ? (addr | I2C_CR2_ADD10) : (addr << 1);

	/*        Transmit register address        */

	// Set transfer size (first transmit only register address)
	I2Cx->CR2 |= 1 << I2C_CR2_NBYTES_POS;
	// Set data transmit register before start
	I2Cx->TXDR = reg;
	// Generate start condition
	I2Cx->CR2 |= I2C_CR2_START;
	// Loop until TC flag is raised
	while (!(I2Cx->ISR & I2C_ISR_TC))
		;

	/*        Receive data        */

	// Set transfer direction to read
	I2Cx->CR2 |= I2C_CR2_RD_WRN;
	// Set receive size
	I2Cx->CR2 &= ~I2C_CR2_NBYTES_MASK;
	I2Cx->CR2 |= (uint8_t)size << I2C_CR2_NBYTES_POS;
	// Generate start condition
	I2Cx->CR2 |= I2C_CR2_START;
	while (!(I2Cx->ISR & I2C_ISR_TC)) {
		if (I2Cx->ISR & I2C_ISR_RXNE)
			*(data++) = I2Cx->RXDR;
	}
	// Set stop condition
	I2Cx->CR2 |= I2C_CR2_STOP;
	// Clear stop flag
	I2C1->ICR |= I2C_ICR_STOPCF;
}