/*
 * I2C_driver.h
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	I2C_STATE_UNINITIALIZED = 0,
	I2C_STATE_IDLE = 1,
	I2C_STATE_READ = 2,
	I2C_STATE_WRITE = 3,
	I2C_STATE_ERROR = 4,
	I2C_STATE_SUCCESS = 5
} I2C_state;

typedef enum {
	I2C_TRANSACTION_ACCEPTED = 0,
	I2C_TRANSACTION_DENIED = 1
} I2C_transaction_state;

void initialize_I2C(uint8_t i2c_number);
void reset_I2C(uint8_t i2c_number);
I2C_state get_I2C_state(uint8_t i2c_number);
I2C_transaction_state write_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t *data, uint8_t nbytes);
I2C_transaction_state read_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t *data, uint8_t nbytes);

bool active_wait_write_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t* data, uint8_t nbytes, uint32_t timeout_ms);
bool active_wait_read_I2C(uint8_t i2c_number, uint8_t addr,
		uint8_t* data, uint8_t nbytes, uint32_t timeout_ms);

#endif /* I2C_DRIVER_H_ */
