/*
 * ADC_measure.h
 *
 *  Created on: 1 paź 2026
 *      Author: Tomasz Pniewski
 */

#ifndef ADC_MEASURE_H_
#define ADC_MEASURE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "GPIO_manager.h"

void initialize_DIS(void);
void voltage_sensor_on(void);
bool ADC_start_measurement(ADC_HandleTypeDef* hadc);
bool ADC_collect_measurement(ADC_HandleTypeDef* hadc, uint16_t* val);
void ADC_abandon_measurement(ADC_HandleTypeDef* hadc);


#endif /* ADC_MEASURE_H_ */
