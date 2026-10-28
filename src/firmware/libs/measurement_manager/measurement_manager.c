/*
 * measurement_manager.c
 *
 *  Created on: 4 paź 2026
 *      Author: Tomasz Pniewski
 */

#include "measurement_manager.h"

#include "SHT31_driver.h"
#include "BH1750_driver.h"
#include "BME280_driver.h"
#include "ADC_measure.h"

#include "utils.h"


static ADC_HandleTypeDef* hadc;

static uint16_t SHT31_temp;
static uint16_t SHT31_rh;
static int32_t BME280_temp;
static uint32_t BME280_rh;
static uint32_t BME280_press;
static uint16_t BH1750_lux;
static uint16_t ADC_voltage;

static bool SHT31_in_progress;
static bool BME280_in_progress;
static bool BH1750_in_progress;
static bool ADC_in_progress;

static bool DIS_off_ready;
static bool SHT31_ready;
static bool BME280_ready;
static bool BH1750_ready;
static bool ADC_ready;

static uint32_t SHT31_start;
static uint32_t BME280_start;
static uint32_t BH1750_start;
static uint32_t DIS_off_start;
static uint32_t ADC_start;

static uint8_t SHT31_try;
static uint8_t BME280_try;
static uint8_t BH1750_try;
static uint8_t ADC_try;

static const uint8_t BH1750_tries = 2;
static const uint32_t BH1750_wait_ms = 185;
static const uint8_t SHT31_tries = 2;
static const uint32_t SHT31_wait_ms = 30;
static const uint8_t BME280_tries = 2;
static const uint32_t BME280_wait_ms = 12;
static const uint8_t ADC_tries = 2;
static const uint32_t DIS_wait_ms = 1500;
static const uint32_t ADC_timeout_ms = 100;


void start_measurements(ADC_HandleTypeDef* measure_hadc) {
	hadc = measure_hadc;
	SHT31_try = 0;
	BME280_try = 0;
	BH1750_try = 0;
	ADC_try = 0;
	while (SHT31_try < SHT31_tries) {
		if (SHT31_start_measurement()) {
			break;
		}
		SHT31_try++;
	}
	SHT31_start = HAL_GetTick();
	SHT31_in_progress = true;
	while (BME280_try < BME280_tries) {
		if (BME280_start_measurement()) {
			break;
		}
		BME280_try++;
	}
	BME280_start = HAL_GetTick();
	BME280_in_progress = true;
	while (BH1750_try < BH1750_tries) {
		if (BH1750_start_measurement()) {
			break;
		}
		BH1750_try++;
	}
	BH1750_start = HAL_GetTick();
	BH1750_in_progress = true;
	DIS_off_ready = false;
	ADC_in_progress = false;
	SHT31_ready = false;
	BME280_ready = false;
	BH1750_ready = false;
	ADC_ready = false;
}

bool advance_measurements(void) {
	if (!SHT31_ready && (HAL_GetTick() - SHT31_start > SHT31_wait_ms)) {
		while (SHT31_try < SHT31_tries) {
			if (SHT31_collect_measurement(&SHT31_temp, &SHT31_rh)) {

				SHT31_ready = true;
				break;
			}
			// We start the measurement again.
			if (SHT31_try != SHT31_tries - 1) {
				while (SHT31_try < SHT31_tries) {
					if (SHT31_start_measurement()) {
						SHT31_start = HAL_GetTick();
						break;
					}
					SHT31_try++;
				}

			}
			SHT31_try++;
		}
		if (SHT31_try >= SHT31_tries) {
			SHT31_ready = true;
		}
	}
	if (!DIS_off_ready && SHT31_ready) {
		voltage_sensor_on();
		DIS_off_start = HAL_GetTick();
		DIS_off_ready = true;
	}
	if (!BH1750_ready && (HAL_GetTick() - BH1750_start > BH1750_wait_ms)) {
		while (BH1750_try < BH1750_tries) {
			if (BH1750_collect_measurement(&BH1750_lux)) {
				BH1750_ready = true;
				break;
			}
			else if (BH1750_try != BH1750_tries - 1) {
				while (BH1750_try < BH1750_tries) {
					if (BH1750_start_measurement()) {
						BH1750_start = HAL_GetTick();
						BH1750_in_progress = true;
						break;
					}
					BH1750_try++;
				}
			}
			BH1750_try++;
		}
		if (BH1750_try >= BH1750_tries) {
			BH1750_ready = true;
		}
	}
	if (!BME280_ready && (HAL_GetTick() - BME280_start > BME280_wait_ms)) {
		while (BME280_try < BME280_tries) {
			if (BME280_collect_measurement(&BME280_temp, &BME280_rh, &BME280_press)) {
				BME280_ready = true;
				break;
			}
			else if (BME280_try != BME280_tries - 1) {
				while (BME280_try < BME280_tries) {
					if (BME280_start_measurement()) {
						BME280_start = HAL_GetTick();
						BME280_in_progress = true;
						break;
					}
					BME280_try++;
				}
			}
			BME280_try++;
		}
		if (BME280_try >= BME280_tries) {
			BME280_ready = true;
		}
	}
	if (SHT31_ready && (HAL_GetTick() - DIS_off_start > DIS_wait_ms)) {
		while (ADC_try < ADC_tries) {
			if (!ADC_in_progress && ADC_try != ADC_tries - 1) {
				  if (ADC_start_measurement(hadc)) {
					  ADC_start = HAL_GetTick();
					  ADC_in_progress = true;
					  break;
				  }
			}
			else {
				  if (ADC_collect_measurement(hadc, &ADC_voltage)) {
					  ADC_ready = true;
					  ADC_in_progress = false;
					  break;
				  }
				  else {
					  if (HAL_GetTick() - ADC_start > ADC_timeout_ms) {
						  ADC_abandon_measurement(hadc);
						  ADC_in_progress = false;
					  }
					  else {
						  break;
					  }
				  }
			}
			ADC_try++;
		}
		if (ADC_try >= ADC_tries) {
			ADC_ready = true;
		}
	}
	return ((ADC_ready && BH1750_ready) && (SHT31_ready && BME280_ready));
}

void collect_measurements(uint8_t* buffer) {
	// We pack 21 bytes into the buffer.
	uint8_t flag_byte = 0;
	flag_byte |= (BME280_try >= BME280_tries) * BME280_ERR_FLAG +
			(BH1750_try >= BH1750_tries) * BH1750_ERR_FLAG +
			(SHT31_try >= SHT31_tries) * SHT31_ERR_FLAG +
			(ADC_try >= ADC_tries) * ADC_ERR_FLAG;
	buffer[0] = flag_byte;
	pack_u16(&buffer[1], SHT31_temp);
	pack_u16(&buffer[3], SHT31_rh);
	pack_u16(&buffer[5], BH1750_lux);
	pack_s32(&buffer[7], BME280_temp);
	pack_u32(&buffer[11], BME280_rh);
	pack_u32(&buffer[15], BME280_press);
	pack_u16(&buffer[19], ADC_voltage);
}

void initialize_sensors(void) {
	SHT31_initialize_sensor();
	BH1750_initialize_sensor();
	BME280_initialize_sensor();
	initialize_DIS();
}


