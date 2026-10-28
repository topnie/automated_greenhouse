/*
 * SHT31_driver.h
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef SHT31_DRIVER_H_
#define SHT31_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>


void SHT31_initialize_sensor(void);
bool SHT31_start_measurement(void);
bool SHT31_collect_measurement(uint16_t *temp, uint16_t *rh);

#endif /* SHT31_DRIVER_H_ */
