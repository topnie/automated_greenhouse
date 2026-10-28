/*
 * BH1750_driver.h
 *
 *  Created on: 24 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef BH1750_DRIVER_H_
#define BH1750_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

bool BH1750_initialize_sensor(void);
bool BH1750_start_measurement(void);
bool BH1750_collect_measurement(uint16_t* lux);


#endif /* BH1750_DRIVER_H_ */
