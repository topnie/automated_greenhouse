/*
 * utils.h
 *
 *  Created on: 25 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

void pack_u16(uint8_t* buffer, uint16_t val);
void pack_s32(uint8_t* buffer, int32_t val);
void pack_u32(uint8_t* buffer, uint32_t val);

void unpack_u16(const uint8_t *buffer, uint16_t *val);
void unpack_s32(const uint8_t *buffer, int32_t *val);
void unpack_u32(const uint8_t *buffer, uint32_t *val);

int32_t convert_temp_SHT31_to_BME280(uint16_t SHT31_temp);
uint32_t convert_rh_SHT31_to_BME280(uint16_t SHT31_rh);

#endif /* UTILS_H_ */
