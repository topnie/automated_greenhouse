/*
 * BME280_driver.c
 *
 *  Created on: 25 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include "BME280_driver.h"
#include "I2C_driver.h"
#include "GPIO_manager.h"

typedef enum {
	BME280_UNINITIALIZED = 0,
	BME280_IDLE = 1,
	BME280_MEASUREMENT_IN_PROGRESS = 2
} BME280_state;

static const uint8_t sensor_addr = 0x76;
static uint8_t read_buffer[8];
static const uint32_t default_timeout_ms = 5;
static const uint8_t register_dig_read1 = 0x88;
static const uint8_t register_dig_read2 = 0xE1;
static const uint8_t register_read = 0xF7;
static const uint8_t register_ctrl_hum = 0xF2;
static const uint8_t register_ctrl_meas = 0xF4;
static const uint8_t register_config = 0xF5;
// Measurement - temp and pressure oversampling x1, forced mode.
static const uint8_t measurement_command = 0b00100101U;

static int32_t t_global;

// Compensation constants for computing the measurements.
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t dig_H6;

static BME280_state state;

static bool active_wait_read_device_registers(uint8_t sensor_register, uint8_t *val, uint8_t nbytes) {
	uint8_t register_addr = sensor_register;
	if (!active_wait_write_I2C(1, sensor_addr, &register_addr, 1, default_timeout_ms)) {
		return false;
	}
	if (!active_wait_read_I2C(1, sensor_addr, val, nbytes, default_timeout_ms)) {
		return false;
	}
	return true;
}

static bool active_wait_write_device_register(uint8_t sensor_register, uint8_t val) {
	uint8_t payload[2] = {sensor_register, val};
	if (!active_wait_write_I2C(1, sensor_addr, payload, 2, default_timeout_ms)) {
		return false;
	}
	return true;
}

static inline uint16_t combine_bytes_u16(uint8_t lo, uint8_t hi) {
	return (uint16_t) (((uint16_t)hi << 8) | lo);
}

static inline int16_t combine_bytes_s16(uint8_t lo, uint8_t hi) {
	return (int16_t) (((uint16_t)hi << 8) | lo);
}

static bool initialize_digs(void) {
	uint8_t buffer[26];
	if (!active_wait_read_device_registers(register_dig_read1, buffer, 26)) {
		return false;
	}
	// Now, initialize all the digs according to the datasheet.
	dig_T1 = combine_bytes_u16(buffer[0], buffer[1]);
	dig_T2 = combine_bytes_s16(buffer[2], buffer[3]);
	dig_T3 = combine_bytes_s16(buffer[4], buffer[5]);
	dig_P1 = combine_bytes_u16(buffer[6], buffer[7]);
	dig_P2 = combine_bytes_s16(buffer[8], buffer[9]);
	dig_P3 = combine_bytes_s16(buffer[10], buffer[11]);
	dig_P4 = combine_bytes_s16(buffer[12], buffer[13]);
	dig_P5 = combine_bytes_s16(buffer[14], buffer[15]);
	dig_P6 = combine_bytes_s16(buffer[16], buffer[17]);
	dig_P7 = combine_bytes_s16(buffer[18], buffer[19]);
	dig_P8 = combine_bytes_s16(buffer[20], buffer[21]);
	dig_P9 = combine_bytes_s16(buffer[22], buffer[23]);
	dig_H1 = (uint8_t)buffer[25];
	if (!active_wait_read_device_registers(register_dig_read2, buffer, 7)) {
		return false;
	}
	dig_H2 = combine_bytes_s16(buffer[0], buffer[1]);
	dig_H3 = (uint8_t)buffer[2];
	dig_H4 = (int16_t)(((uint16_t)buffer[3] << 4) | (uint16_t)(buffer[4] & (0b00001111U)));
	dig_H5 = (int16_t)(((uint16_t)buffer[5] << 4) | (uint16_t)(buffer[4] >> 4));
	dig_H6 = (int8_t)buffer[6];
	return true;
}

// Compensation formulas below were adapted from product datasheet.

static int32_t BME280_calculate_temp(int32_t raw_temp)
{
	t_global = ((((((raw_temp>>4) - ((int32_t)dig_T1)) * ((raw_temp>>4) - ((int32_t)dig_T1)))
			>> 12) * ((int32_t)dig_T3)) >> 14) + (((((raw_temp>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11);
	return (t_global * 5 + 128) >> 8;
}

static uint32_t BME280_calculate_pressure(int32_t raw_press)
{	
	int64_t offset = (((int64_t)t_global) - 128000) * (((int64_t)t_global) - 128000) * (int64_t)dig_P6 
			+ (((((int64_t)t_global) - 128000) * (int64_t)dig_P5) << 17) + (((int64_t)dig_P4) << 35);
	int64_t denom = (((int64_t)dig_P1) * (((((int64_t)1)<<47) 
			+ (((((int64_t)t_global) - 128000) * (((int64_t)t_global) - 128000) * (int64_t)dig_P3)>>8)
			+ (((((int64_t)t_global) - 128000) * (int64_t)dig_P2)<<12)))) >> 33;
	if (denom == 0) {
		return 0;
	}
	int64_t tmp = (((((int64_t)(1048576 - raw_press)) << 31) - offset) * 3125) / denom;
	return (uint32_t) (((tmp + ((((int64_t)dig_P9) * (tmp >> 13) * (tmp >> 13)) >> 25) + ((((int64_t)dig_P8) * tmp) >> 19)) >> 8) + (((int64_t)dig_P7) << 4));
}

static inline int32_t clamp_int32(int32_t x, int32_t low, int32_t high) {
	if (x < low) {
		return low;
	}
	if (x > high) {
		return high;
	}
	return x;
}

static uint32_t BME280_calculate_humidity(int32_t raw_hum) {
	int32_t tmp = (((((raw_hum << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *
			((t_global - ((int32_t)76800))))) + ((int32_t)16384)) >> 15) * (((((((((t_global - ((int32_t)76800))) *
			((int32_t)dig_H6)) >> 10) * (((((t_global - ((int32_t)76800))) * ((int32_t)dig_H3)) >> 11) +
			((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) +
			8192) >> 14));
	tmp = (tmp - (((((tmp >> 15) * (tmp >> 15)) >> 7) *
			((int32_t)dig_H1)) >> 4));
	tmp = clamp_int32(tmp, 0, 419430400);
	return (uint32_t)(tmp >> 12);
}

bool BME280_initialize_sensor(void) {
	// Initialize GPIO pins.
	GPIO_config I2C_config = {
		.mode = GPIO_PIN_MODE_ALTERNATE_FUNCTION,
		.output_type = GPIO_OUTPUT_OPEN_DRAIN,
		.output_speed = GPIO_OUTPUT_SPEED_VERY_HIGH,
		.pull_up_down = GPIO_PULL_NONE,
		.AF = 0b0100U
	};

	configure_GPIO(1, 7, I2C_config);
	configure_GPIO(1, 8, I2C_config);

	// Reset, since this function can be called multiple times.
	reset_I2C(1);

	if (!initialize_digs()) {
		return false;
	}

	// Now initialize the sensor.
	// Read the config byte.
	uint8_t config_val;
	if (!active_wait_read_device_registers(register_config, &config_val, 1)) {
		return false;
	}
	config_val &= ~(1U << 1); // Clear all but the reserved bit.
	// Set config - no IIR filter, and no SPI interface.
	config_val |= (0b11100000U);
	if (!active_wait_write_device_register(register_config, config_val)) {
		return false;
	}
	// Now read and set the humidity control register.
	uint8_t hum_config_val;
	if (!active_wait_read_device_registers(register_ctrl_hum, &hum_config_val, 1)) {
		return false;
	}
	// Set humidity oversampling to x1.
	hum_config_val &= 0b11111000U;
	hum_config_val |= (0b00000001U);
	if (!active_wait_write_device_register(register_ctrl_hum, hum_config_val)) {
		return false;
	}
	// We're all set and ready for measurement.
	state = BME280_IDLE;
	return true;
}

bool BME280_start_measurement(void) {
	if (state != BME280_IDLE) {
		return false;
	}
	if (!active_wait_write_device_register(register_ctrl_meas, measurement_command)) {
		return false;
	}
	state = BME280_MEASUREMENT_IN_PROGRESS;
	return true;
}

bool BME280_collect_measurement(int32_t* temp, uint32_t* rh, uint32_t* press) {
	if (state != BME280_MEASUREMENT_IN_PROGRESS) {
		return false;
	}
	if (!active_wait_read_device_registers(register_read, read_buffer, 8)) {
		state = BME280_IDLE;
		return false;
	}
	// We have to decode the values from the read buffer.
	int32_t raw_press = (int32_t)(((uint32_t)read_buffer[0] << 12) |
							  ((uint32_t)read_buffer[1] << 4) |
							  ((uint32_t)read_buffer[2] >> 4));
	int32_t raw_temp = (int32_t)(((uint32_t)read_buffer[3] << 12) |
							  ((uint32_t)read_buffer[4] << 4) |
							  ((uint32_t)read_buffer[5] >> 4));
	int32_t raw_hum = (int32_t)(((uint32_t)read_buffer[6] << 8) |
							  ((uint32_t)read_buffer[7]));

	*temp = BME280_calculate_temp(raw_temp);
	*press = BME280_calculate_pressure(raw_press);
	*rh = BME280_calculate_humidity(raw_hum);

	state = BME280_IDLE;
	return true;
}
