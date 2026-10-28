/*
 * I2C_driver.c
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include <assert.h>
#include "I2C_driver.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

static volatile I2C_state state[3];
// Pointers to where the data should be written or read from.
static uint8_t* volatile I2C_data[3];
// Offset for data pointers.
static volatile uint8_t offset[3];

bool active_wait_read_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t* data, uint8_t nbytes, uint32_t timeout_ms) {
	if (nbytes == 0) {
		return false;
	}
	I2C_transaction_state transaction_state = read_I2C(i2c_number, addr, data, nbytes);
	if (transaction_state == I2C_TRANSACTION_ACCEPTED) {
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < timeout_ms) {
			I2C_state I2C3_state = get_I2C_state(i2c_number);
			if (I2C3_state == I2C_STATE_SUCCESS) {
				return true;
			}
			else if (I2C3_state == I2C_STATE_ERROR) {
				break;
			}
		}
		reset_I2C(i2c_number);
		return false;
	}
	else {
		return false;
	}
}

bool active_wait_write_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t* data, uint8_t nbytes, uint32_t timeout_ms) {
	I2C_transaction_state transaction_state = write_I2C(i2c_number, addr, data, nbytes);
	if (transaction_state == I2C_TRANSACTION_ACCEPTED) {
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < timeout_ms) {
			I2C_state I2C3_state = get_I2C_state(i2c_number);
			if (I2C3_state == I2C_STATE_SUCCESS) {
				return true;
			}
			else if (I2C3_state == I2C_STATE_ERROR) {
				break;
			}
		}
		reset_I2C(i2c_number);
		return false;
	}
	else {
		return false;
	}
}


static I2C_TypeDef* get_I2C_from_number(uint8_t i2c_number) {
	switch(i2c_number) {
		case 1:
			return I2C1;
		case 2:
			return I2C2;
		case 3:
			return I2C3;
	}
	return I2C1;
}

static void configure_I2C(I2C_TypeDef* I2C, uint8_t i2c_number) {
	// Disable the I2C peripheral.
	I2C->CR1 &= ~(I2C_CR1_PE);

	// Clock configuration.
	switch(i2c_number) {
		case 1:
			// Select PCLK as clock for I2C kernel.
			RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL);
			// Enable peripheral clock for I2C.
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
			break;
		case 2:
			RCC->CCIPR &= ~(RCC_CCIPR_I2C2SEL);
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
			break;
		case 3:
			RCC->CCIPR &= ~(RCC_CCIPR_I2C3SEL);
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
			break;
	}

	I2C->CR1 &= ~(I2C_CR1_ANFOFF); // Set analog noise filter to on.
	I2C->TIMINGR &= ~(0xF0FFFFFFUL); // Clear register (not the restricted bits).
	I2C->TIMINGR |= (0x40B285C2UL); // Timing generated using the CubeMX tool.
	I2C->CR1 &= ~(I2C_CR1_NOSTRETCH); // Enable clock stretching.

	// Enable the I2C peripheral.
	I2C->CR1 |= I2C_CR1_PE;
}

static void generic_I2C_EV_handler(I2C_TypeDef* I2C, uint8_t i2c_number) {
	uint32_t isr = I2C->ISR;

	if (isr & I2C_ISR_TXIS) {
		// We need to transmit more data.
		I2C->TXDR = I2C_data[i2c_number - 1][offset[i2c_number - 1]];
		offset[i2c_number - 1]++;
	}

	if (isr & I2C_ISR_RXNE) {
		// We need to read more data.
		I2C_data[i2c_number - 1][offset[i2c_number - 1]] = I2C->RXDR;
		offset[i2c_number - 1]++;
	}

	if (isr & I2C_ISR_STOPF) {
		// STOP detected, finish transaction.
		I2C->ICR |= I2C_ICR_STOPCF; // Clear STOP.
		// Disable I2C interrupts.
		I2C->CR1 &= ~(I2C_CR1_ERRIE);
		I2C->CR1 &= ~(I2C_CR1_STOPIE);
		I2C->CR1 &= ~(I2C_CR1_TXIE);
		I2C->CR1 &= ~(I2C_CR1_RXIE);
		state[i2c_number - 1] = I2C_STATE_SUCCESS;
	}

}

static void generic_I2C_ER_handler(I2C_TypeDef* I2C, uint8_t i2c_number) {
	// Update transaction state.
	state[i2c_number - 1] = I2C_STATE_ERROR;
	// Reset the I2C peripheral.
	configure_I2C(I2C, i2c_number);
}

void I2C1_EV_IRQHandler(void)
{
    generic_I2C_EV_handler(I2C1, 1);
}

void I2C1_ER_IRQHandler(void)
{
	generic_I2C_ER_handler(I2C1, 1);
}

void I2C2_EV_IRQHandler(void)
{
    generic_I2C_EV_handler(I2C2, 2);
}

void I2C2_ER_IRQHandler(void)
{
	generic_I2C_ER_handler(I2C2, 2);
}

void I2C3_EV_IRQHandler(void)
{
    generic_I2C_EV_handler(I2C3, 3);
}

void I2C3_ER_IRQHandler(void)
{
	generic_I2C_ER_handler(I2C3, 3);
}

void initialize_I2C(uint8_t i2c_number) {
	assert(i2c_number >= 1 && i2c_number <= 3);
	I2C_TypeDef* I2C;
	// Enable interrupts from the I2C peripheral in NVIC.
	switch(i2c_number) {
		case 1:
			NVIC_EnableIRQ(I2C1_EV_IRQn);
			NVIC_EnableIRQ(I2C1_ER_IRQn);
			I2C = I2C1;
			break;
		case 2:
			NVIC_EnableIRQ(I2C2_EV_IRQn);
			NVIC_EnableIRQ(I2C2_ER_IRQn);
			I2C = I2C2;
			break;
		case 3:
			NVIC_EnableIRQ(I2C3_EV_IRQn);
			NVIC_EnableIRQ(I2C3_ER_IRQn);
			I2C = I2C3;
			break;
	}
	// We assume the GPIO pins are set up by the device driver.
	// Here, we just initialize and configure the I2C peripheral.
	configure_I2C(I2C, i2c_number);
	state[i2c_number - 1] = I2C_STATE_IDLE;
}

void reset_I2C(uint8_t i2c_number) {
	switch(i2c_number) {
		case 1:
			NVIC_DisableIRQ(I2C1_EV_IRQn);
			NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
			NVIC_DisableIRQ(I2C1_ER_IRQn);
			NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST);
			break;
		case 2:
			NVIC_DisableIRQ(I2C2_EV_IRQn);
			NVIC_ClearPendingIRQ(I2C2_EV_IRQn);
			NVIC_DisableIRQ(I2C2_ER_IRQn);
			NVIC_ClearPendingIRQ(I2C2_ER_IRQn);
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C2RST);
			break;
		case 3:
			NVIC_DisableIRQ(I2C3_EV_IRQn);
			NVIC_DisableIRQ(I2C3_ER_IRQn);
			NVIC_ClearPendingIRQ(I2C3_ER_IRQn);
			RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C3RST;
			RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C3RST);
			break;
	}
	initialize_I2C(i2c_number);
}

I2C_state get_I2C_state(uint8_t i2c_number) {
	assert(i2c_number >= 1 && i2c_number <= 3);
	I2C_state state_res = state[i2c_number - 1];
	if (state_res == I2C_STATE_SUCCESS || state_res == I2C_STATE_ERROR) {
		state[i2c_number - 1] = I2C_STATE_IDLE;
	}
	return state_res;
}

I2C_transaction_state write_I2C(uint8_t i2c_number, uint8_t addr, uint8_t *data, uint8_t nbytes) {
	assert(i2c_number >= 1 && i2c_number <= 3);
	if (!(state[i2c_number - 1] == I2C_STATE_IDLE)) {
		return I2C_TRANSACTION_DENIED;
	}
	I2C_TypeDef* I2C = get_I2C_from_number(i2c_number);
	I2C->CR2 &= ~(I2C_CR2_ADD10); // Disable 10 bit addresing.
	I2C->CR2 |= (I2C_CR2_AUTOEND); // Enable AUTOEND.

	I2C->CR2 &= ~(I2C_CR2_NBYTES);
	I2C->CR2 |= (nbytes << I2C_CR2_NBYTES_Pos); // Set nbytes.

	I2C->CR2 &= ~(I2C_CR2_SADD);
	I2C->CR2 |= (addr << 1U); // Set address.

	I2C->CR2 &= ~(I2C_CR2_RD_WRN); // Set RD_WRN to 0 -> Write mode.

	state[i2c_number - 1] = I2C_STATE_WRITE;
	offset[i2c_number - 1] = 0;
	I2C_data[i2c_number - 1] = data;

	// Enable I2C interrupts.
	I2C->CR1 |= I2C_CR1_ERRIE;
	I2C->CR1 |= I2C_CR1_STOPIE;
	I2C->CR1 |= I2C_CR1_TXIE;

	I2C->CR2 |= I2C_CR2_START; // Start transmission.
	return I2C_TRANSACTION_ACCEPTED;
}

I2C_transaction_state read_I2C(uint8_t i2c_number, uint8_t addr, uint8_t *data, uint8_t nbytes) {
	assert(i2c_number >= 1 && i2c_number <= 3);
	if (!(state[i2c_number - 1] == I2C_STATE_IDLE)) {
		return I2C_TRANSACTION_DENIED;
	}
	I2C_TypeDef* I2C = get_I2C_from_number(i2c_number);
	I2C->CR2 &= ~(I2C_CR2_ADD10); // Disable 10 bit addresing.
	I2C->CR2 |= (I2C_CR2_AUTOEND); // Enable AUTOEND.

	I2C->CR2 &= ~(I2C_CR2_NBYTES);
	I2C->CR2 |= (nbytes << I2C_CR2_NBYTES_Pos); // Set nbytes.

	I2C->CR2 &= ~(I2C_CR2_SADD);
	I2C->CR2 |= (addr << 1U); // Set address.

	I2C->CR2 |= (I2C_CR2_RD_WRN); // Set RD_WRN to 1 -> Read mode.

	state[i2c_number - 1] = I2C_STATE_READ;
	offset[i2c_number - 1] = 0;
	I2C_data[i2c_number - 1] = data;

	// Enable I2C interrupts.
	I2C->CR1 |= I2C_CR1_ERRIE;
	I2C->CR1 |= I2C_CR1_STOPIE;
	I2C->CR1 |= I2C_CR1_RXIE;

	I2C->CR2 |= I2C_CR2_START; // Start transmission.
	return I2C_TRANSACTION_ACCEPTED;
}
