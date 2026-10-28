/*
 * measurement_manager.h
 *
 *  Created on: 4 paź 2026
 *      Author: Tomasz Pniewski
 */

#ifndef MEASUREMENT_MANAGER_H_
#define MEASUREMENT_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define SHT31_ERR_FLAG (1U)
#define BME280_ERR_FLAG (1U << 1)
#define BH1750_ERR_FLAG (1U << 2)
#define ADC_ERR_FLAG (1U << 3)

void initialize_sensors(void);
void start_measurements(ADC_HandleTypeDef* hadc);
bool advance_measurements(void);
void collect_measurements(uint8_t* buffer);

#endif /* MEASUREMENT_MANAGER_H_ */
