/*
 * BH1750_driver.c
 *
 *  Created on: 24 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include "BH1750_driver.h"
#include "I2C_driver.h"
#include "GPIO_manager.h"
#include "stm32g4xx_hal.h"

typedef enum {
	BH1750_UNINITIALIZED = 0,
	BH1750_IDLE = 1,
	BH1750_MEASUREMENT_IN_PROGRESS = 2
} BH1750_state;

static uint8_t read_buffer[2];
static const uint32_t default_timeout_ms = 5;
static const uint8_t opecode_power_down = 0;
static const uint8_t opecode_power_on = 1;
static const uint8_t opecode_h_resolution_one_time = 0b00100000U;
static const uint8_t opecode_change_measurement_time_default_high_bit = 0b01000010U;
static const uint8_t opecode_change_measurement_time_default_low_bit = 0b01100101U;

static BH1750_state state;

static bool active_wait_read(uint16_t* lux_data, uint32_t timeout_ms) {
	I2C_transaction_state transaction_state = read_I2C(3, 0x23, read_buffer, 2);
	if (transaction_state == I2C_TRANSACTION_ACCEPTED) {
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < timeout_ms) {
			I2C_state I2C3_state = get_I2C_state(3);
			if (I2C3_state == I2C_STATE_SUCCESS) {
				*lux_data = ((uint16_t)(read_buffer[0]) << 8) | (uint16_t)(read_buffer[1]);
				return true;
			}
			else if (I2C3_state == I2C_STATE_ERROR) {
				break;
			}
		}
		reset_I2C(3);
		return false;
	}
	else {
		return false;
	}
}

static bool active_wait_write(uint8_t* command, uint32_t timeout_ms) {
	I2C_transaction_state transaction_state = write_I2C(3, 0x23, command, 1);
	if (transaction_state == I2C_TRANSACTION_ACCEPTED) {
		uint32_t start = HAL_GetTick();
		while ((HAL_GetTick() - start) < timeout_ms) {
			I2C_state I2C3_state = get_I2C_state(3);
			if (I2C3_state == I2C_STATE_SUCCESS) {
				return true;
			}
			else if (I2C3_state == I2C_STATE_ERROR) {
				break;
			}
		}
		reset_I2C(3);
		return false;
	}
	else {
		return false;
	}
}

bool BH1750_initialize_sensor(void) {
	// Initialize GPIO pins.
	GPIO_config I2C_config = {
		.mode = GPIO_PIN_MODE_ALTERNATE_FUNCTION,
		.output_type = GPIO_OUTPUT_OPEN_DRAIN,
		.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH,
		.pull_up_down = GPIO_PULL_NONE,
		.AF = 0b1000U
	};

	configure_GPIO(2, 8, I2C_config);
	configure_GPIO(2, 9, I2C_config);

	// Initialize I2C.
	// Reset, since this function can be called multiple times.
	reset_I2C(3);

	// Now initialize the sensor.
	// First set the sensor to power down mode.
	if (!active_wait_write(&opecode_power_down, default_timeout_ms)) {
		return false;
	}
	// Then set default measurement time.
	if (!active_wait_write(&opecode_change_measurement_time_default_high_bit,
			default_timeout_ms)) {
		return false;
	}
	if (!active_wait_write(&opecode_change_measurement_time_default_low_bit,
			default_timeout_ms)) {
		return false;
	}
	state = BH1750_IDLE;
	return true;
}

bool BH1750_start_measurement(void) {
	if (state != BH1750_IDLE) {
		return false;
	}
	if (!active_wait_write(&opecode_power_on, default_timeout_ms)) {
		return false;
	}
	if (!active_wait_write(&opecode_h_resolution_one_time, default_timeout_ms)) {
		active_wait_write(&opecode_power_down, default_timeout_ms);
		return false;
	}
	state = BH1750_MEASUREMENT_IN_PROGRESS;
	return true;
}

bool BH1750_collect_measurement(uint16_t* lux) {
	if (state != BH1750_MEASUREMENT_IN_PROGRESS) {
		return false;
	}
	if (!active_wait_read(lux, default_timeout_ms)) {
		state = BH1750_IDLE;
		return false;
	}
	state = BH1750_IDLE;
	return true;
}