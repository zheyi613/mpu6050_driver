/**
 * @file stm32f767zi_i2c.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Inter-intergrated circuit
 * @date 2022-08-20
 */

#ifndef __STM32F767ZI_I2C_H
#define __STM32F767ZI_I2C_H

#include "stm32f767zi_gpio.h"
#include "stm32f767zi_hal.h"
// Control register 1
#define I2C_CR1_PE              1U              // Peripheral enable
#define I2C_CR1_TXIE            (1U << 1)       // TX Interrupt enable
#define I2C_CR1_RXIE            (1U << 2)       // RX Interrupt enable
#define I2C_CR1_ADDRIE          (1U << 3)       // Address match interrupt enable (slave only)
#define I2C_CR1_NACKIE          (1U << 4)       // Not acknowledge receive interrupt enable
#define I2C_CR1_STOPIE          (1U << 5)       // STOP detection interrupt enable
// Control register 2
#define I2C_CR2_SADD_MASK       0x1FFU          // Slave address mask
#define I2C_CR2_RD_WRN          (1U << 10)      // Transfer direction (master mode)
#define I2C_CR2_ADD10           (1U << 11)      // 10-bit addressing mode (master mode)
#define I2C_CR2_HEAD10R         (1U << 12)      // 10-bit address header only read direction (master receiver mode)
#define I2C_CR2_START           (1U << 13)      // Start generation
#define I2C_CR2_STOP            (1U << 14)      // Stop generation
#define I2C_CR2_NACK            (1U << 15)      // NACK generation (slave mode)
#define I2C_CR2_NBYTES_POS      16              // Number of bytes
#define I2C_CR2_NBYTES_MASK     (0x0FU << I2C_CR2_NBYTES_POS)
#define I2C_CR2_RELOAD          (1U << 24)      // NBYTES reload mode
#define I2C_CR2_AUTOEND         (1U << 25)      // Automatic end mode (master mode)
// Own address 1 register
#define I2C_OAR1_OA1            (0xFFU << 1)    // Interface address
#define I2C_OAR1_OA1MODE        (1U << 10)      // Own Address 1 10-bits mode
#define I2C_OAR1_OA1EN          (1U << 15)      // Own Address 1 enable
// Interrupt and status register
#define I2C_ISR_TXE             1U              // Transmit data register empty (transmitters)
#define I2C_ISR_TXIS            (1U << 1)       // Transmit interrupt status (transmitters)
#define I2C_ISR_RXNE            (1U << 2)       // Receive data register not empty (receivers)
#define I2C_ISR_ADDR            (1U << 3)       // Address match (slave mode)
#define I2C_ISR_NACKF           (1U << 4)       // Not Acknowledge received flag
#define I2C_ISR_STOPF           (1U << 5)       // Stop detection flag
#define I2C_ISR_TC              (1U << 6)       // Transfer Complete (master mode)
#define I2C_ISR_TCR             (1U << 7)       // Transfer Complete Reload
#define I2C_ISR_BERR            (1U << 8)       // Bus error
#define I2C_ISR_ADDCODE         (0x7FU << 17)   // Address match code (slave mode)
#define I2C_ISR_ADDCODE_POS     17
// Interrupt clear register
#define I2C_ICR_ADDRCF          (1U << 3)       // Address matched flag clear
#define I2C_ICR_NACKCF          (1U << 4)       // Not Acknowledge flag clear
#define I2C_ICR_STOPCF          (1U << 5)       // Stop detection flag clear
#define I2C_ICR_BERRCF          (1U << 8)       // Bus error flag clear
#define I2C_ICR_ARLOCF          (1U << 9)       // Arbitration Lost flag clear
#define I2C_ICR_OVRCF           (1U << 10)      // Overrun/Underrun flag clear
#define I2C_ICR_PECCF           (1U << 11)      // PEC Error flag clear
#define I2C_ICR_TIMOUTCF        (1U << 12)      // Timeout detection flag clear
#define I2C_ICR_ALERTCF         (1U << 13)      // Alert flag clear

#define I2C_AF                  4U              // 0b0100
#define I2C1_SCL_POS            6               // PB6
#define I2C1_SDA_POS            9               // PB9
#define I2C2_SCL_POS            10              // PB10
#define I2C2_SDA_POS            11              // PB11

#define I2C1_EV_IRQn            31U             // I2C 1 event interrupt
#define I2C2_EV_IRQn            33U             // I2C 2 event interrupt

#define I2C_SLAVE_OWN_ADDRESS   0x2D
#define I2C_TIMING              0x00303D5B      // Generate by CubeMX32 with 16MHz

void i2c2_slave_default_init(void);
void i2c1_master_default_init(void);
uint32_t i2c_get_addr_matchcode(I2C_reg_t *I2Cx);
void i2c_write_reg(I2C_reg_t *I2Cx, const uint16_t addr, const uint8_t reg,
                   const uint8_t size, uint8_t *data);
void i2c_read_reg(I2C_reg_t *I2Cx, const uint16_t addr, const uint8_t reg,
                  const uint8_t size, uint8_t *data);

#endif
