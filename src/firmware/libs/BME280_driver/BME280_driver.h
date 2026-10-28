/*
 * BME280_driver.h
 *
 *  Created on: 25 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef BME280_DRIVER_H_
#define BME280_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

bool BME280_initialize_sensor(void);
bool BME280_start_measurement(void);
bool BME280_collect_measurement(int32_t* temp, uint32_t* rh, uint32_t* press);


#endif /* BME280_DRIVER_H_ */
