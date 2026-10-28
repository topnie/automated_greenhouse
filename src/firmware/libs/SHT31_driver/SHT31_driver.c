/*
 * SHT31_driver.c
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include "SHT31_driver.h"
#include "I2C_driver.h"
#include "GPIO_manager.h"
#include "stm32g4xx_hal.h"

typedef enum {
	SHT31_UNINITIALIZED = 0,
	SHT31_IDLE = 1,
	SHT31_MEASUREMENT_IN_PROGRESS = 2,
} SHT31_state;

static const uint32_t default_timeout_ms = 10;

static SHT31_state state;
static uint8_t command_buffer[2] = {0x24, 0x00}; // High repeatability with no clock stretching.
static uint8_t response_buffer[6];

static void SHT31_get_temp_humidity(uint16_t *temp, uint16_t *rh) {
	*temp = (((uint16_t)response_buffer[0]) << 8) + (uint16_t)response_buffer[1];
	*rh = (((uint16_t)response_buffer[3]) << 8) + (uint16_t)response_buffer[4];
}

void SHT31_initialize_sensor(void) {
	if (state != SHT31_UNINITIALIZED) {
		return;
	}

	// Initialize GPIO pins.
	GPIO_config I2C_config = {
		.mode = GPIO_PIN_MODE_ALTERNATE_FUNCTION,
		.output_type = GPIO_OUTPUT_OPEN_DRAIN,
		.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH,
		.pull_up_down = GPIO_PULL_NONE,
		.AF = 0b0100U
	};

	configure_GPIO(0, 8, I2C_config);
	configure_GPIO(0, 9, I2C_config);

	// Initialize I2C.
	reset_I2C(2);

	state = SHT31_IDLE;
}

bool SHT31_start_measurement(void) {
	if (state != SHT31_IDLE) {
		return false;
	}

	if (!active_wait_write_I2C(2, 0x45, command_buffer, 2, default_timeout_ms)) {
		return false;
	}
	state = SHT31_MEASUREMENT_IN_PROGRESS;
	return true;
}


bool SHT31_collect_measurement(uint16_t *temp, uint16_t *rh) {
	if (state != SHT31_MEASUREMENT_IN_PROGRESS) {
		return false;
	}
	state = SHT31_IDLE;
	if (!active_wait_read_I2C(2, 0x45, response_buffer, 6, default_timeout_ms)) {
		return false;
	}

	SHT31_get_temp_humidity(temp, rh);
	return true;
}
