/*
 * ADC_measure.c
 *
 *  Created on: 1 paź 2026
 *      Author: Tomasz Pniewski
 */

#include "ADC_measure.h"

static volatile bool measurement_in_progress;
static volatile bool measurement_complete;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (measurement_in_progress) {
		measurement_complete = true;
	}
}

void initialize_DIS(void) {
	GPIO_config DIS_config = {
		.mode = GPIO_PIN_MODE_OUTPUT,
		.output_type = GPIO_OUTPUT_PUSH_PULL,
		.output_speed = GPIO_OUTPUT_SPEED_LOW,
		.pull_up_down = GPIO_PULL_NONE,
		.AF = 0
	};

	configure_GPIO(0, 1, DIS_config);
	// Disable the sensor by going high.
	GPIO_output_high(0, 1);
}

void voltage_sensor_on(void) {
	GPIO_output_low(0, 1);
}

bool ADC_start_measurement(ADC_HandleTypeDef* hadc) {
	measurement_in_progress = true;
	if (HAL_ADC_Start_IT(hadc) != HAL_OK) {
		measurement_in_progress = false;
		return false;
	}
	else {
		return true;
	}
}

bool ADC_collect_measurement(ADC_HandleTypeDef* hadc, uint16_t* val) {
	if (!measurement_complete) {
		return false;
	}
	*val = (uint16_t)HAL_ADC_GetValue(hadc);
	ADC_abandon_measurement(hadc);
	return true;
}

void ADC_abandon_measurement(ADC_HandleTypeDef* hadc) {
	measurement_in_progress = false;
	measurement_complete = false;
	HAL_ADC_Stop_IT(hadc);

	GPIO_output_high(0, 1);
}
