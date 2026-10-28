/*
 * GPIO_manager.h
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef GPIO_MANAGER_H_
#define GPIO_MANAGER_H_

#include <stdint.h>

typedef enum
{
	GPIO_PIN_MODE_INPUT = 0b00,
	GPIO_PIN_MODE_OUTPUT = 0b01,
	GPIO_PIN_MODE_ALTERNATE_FUNCTION = 0b10,
	GPIO_PIN_MODE_ANALOG = 0b11
} GPIO_pin_mode;

typedef enum
{
	GPIO_OUTPUT_PUSH_PULL = 0,
	GPIO_OUTPUT_OPEN_DRAIN = 1
} GPIO_pin_output_type;

typedef enum
{
	GPIO_OUTPUT_SPEED_LOW = 0b00,
	GPIO_OUTPUT_SPEED_MEDIUM = 0b01,
	GPIO_OUTPUT_SPEED_HIGH = 0b10,
	GPIO_OUTPUT_SPEED_VERY_HIGH = 0b11
} GPIO_pin_output_speed;

typedef enum
{
	GPIO_PULL_NONE = 0b00,
	GPIO_PULL_UP = 0b01,
	GPIO_PULL_DOWN = 0b10
} GPIO_pin_pull_up_down;

typedef struct {
	GPIO_pin_mode mode;
	GPIO_pin_output_type output_type;
	GPIO_pin_output_speed output_speed;
	GPIO_pin_pull_up_down pull_up_down;
	uint8_t AF;
} GPIO_config;


void configure_GPIO(uint8_t port_number, uint8_t pin_number, GPIO_config pin_config);

void GPIO_output_toggle(uint8_t port_number, uint8_t pin_number);
void GPIO_output_high(uint8_t port_number, uint8_t pin_number);
void GPIO_output_low(uint8_t port_number, uint8_t pin_number);

#endif /* GPIO_MANAGER_H_ */
