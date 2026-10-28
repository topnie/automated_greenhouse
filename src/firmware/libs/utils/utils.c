/*
 * utils.c
 *
 *  Created on: 25 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include "utils.h"
#include <string.h>

void pack_u16(uint8_t* buffer, uint16_t val) {
	buffer[0] = (uint8_t)(val >> 8);
	buffer[1] = (uint8_t)(val);
}

void pack_s32(uint8_t* buffer, int32_t val) {
	buffer[0] = (uint8_t)(val >> 24);
	buffer[1] = (uint8_t)(val >> 16);
	buffer[2] = (uint8_t)(val >> 8);
	buffer[3] = (uint8_t)val;
}

void pack_u32(uint8_t* buffer, uint32_t val) {
	buffer[0] = (uint8_t)(val >> 24);
	buffer[1] = (uint8_t)(val >> 16);
	buffer[2] = (uint8_t)(val >> 8);
	buffer[3] = (uint8_t)val;
}

void unpack_u16(const uint8_t *buffer, uint16_t *val) {
   *val = ((uint16_t)buffer[0] << 8) | ((uint16_t)buffer[1]);
}

void unpack_s32(const uint8_t *buffer, int32_t *val) {
   uint32_t tmp = ((uint32_t)buffer[0] << 24) |
                  ((uint32_t)buffer[1] << 16) |
                  ((uint32_t)buffer[2] << 8) |
                  ((uint32_t)buffer[3]);
   memcpy(val, &tmp, 4);
}

void unpack_u32(const uint8_t *buffer, uint32_t *val) {
   *val = ((uint32_t)buffer[0] << 24) |
          ((uint32_t)buffer[1] << 16) |
          ((uint32_t)buffer[2] << 8) |
          ((uint32_t)buffer[3]);
}

int32_t convert_temp_SHT31_to_BME280(uint16_t SHT31_temp) {
	 return -4500 + (int32_t)(((uint32_t)SHT31_temp * 17500U + 32767U) / 65535U);
}

uint32_t convert_rh_SHT31_to_BME280(uint16_t SHT31_rh) {
    return (uint32_t)(((uint64_t)SHT31_rh * 102400U + 32767U) / 65535U);
}

