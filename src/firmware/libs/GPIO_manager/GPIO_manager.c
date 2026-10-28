/*
 * GPIO_manager.c
 *
 *  Created on: 20 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include "GPIO_manager.h"
#include "stm32g431xx.h"
#include <assert.h>

typedef enum
{
	CLOCK_OFF,
	CLOCK_ON
} GPIO_port_state;

static GPIO_port_state PORT_STATE[7]; // Initialized to CLOCK_OFF.
static GPIO_TypeDef* const PORTS[7] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG};
static const uint32_t CLOCK_ENABLE_MASKS[7] = {
	RCC_AHB2ENR_GPIOAEN,
	RCC_AHB2ENR_GPIOBEN,
	RCC_AHB2ENR_GPIOCEN,
	RCC_AHB2ENR_GPIODEN,
	RCC_AHB2ENR_GPIOEEN,
	RCC_AHB2ENR_GPIOFEN,
	RCC_AHB2ENR_GPIOGEN
};

static void enable_GPIO_clock(uint8_t port_number) {
	RCC->AHB2ENR |= CLOCK_ENABLE_MASKS[port_number];
	// Wait for the earlier write to propagate.
	(void)RCC->AHB2ENR;
	PORT_STATE[port_number] = CLOCK_ON;
}

void configure_GPIO(uint8_t port_number, uint8_t pin_number, GPIO_config pin_config) {
	assert(port_number < 7);
	assert(pin_number < 16);

	if (PORT_STATE[port_number] == CLOCK_OFF) {
		enable_GPIO_clock(port_number);
	}

	GPIO_TypeDef* port = PORTS[port_number];

	// Configure alternate function, if needed.
	if (pin_config.mode == GPIO_PIN_MODE_ALTERNATE_FUNCTION) {
		assert(pin_config.AF < 16);
		uint32_t register_index = (pin_number >= 8);
		uint32_t bit_shift = (pin_number & 7) * 4;
		port->AFR[register_index] &= ~(0b1111UL << bit_shift);
		port->AFR[register_index] |= ((uint32_t)pin_config.AF << bit_shift);
	}

	// Set output type.
	port->OTYPER &= ~(1UL << pin_number);
	port->OTYPER |= (((uint32_t)pin_config.output_type) << pin_number);

	// Set output speed.
	port->OSPEEDR &= ~(0b11UL << (pin_number * 2));
	port->OSPEEDR |= ((uint32_t)pin_config.output_speed << (pin_number * 2));

	// Set internal pull-up or pull-down.
	port->PUPDR &= ~(0b11U << (pin_number * 2));
	port->PUPDR |= ((uint32_t)pin_config.pull_up_down << (pin_number * 2));

	// Set the mode register.
	port->MODER &= ~(0b11UL << (pin_number * 2));
	port->MODER |= ((uint32_t)pin_config.mode << (pin_number * 2));
}

void GPIO_output_toggle(uint8_t port_number, uint8_t pin_number) {
	assert(port_number < 7);
	assert(pin_number < 16);

	GPIO_TypeDef* port = PORTS[port_number];
	if (port->ODR & (0b1UL << pin_number))
		port->BSRR = 0b1UL << (16 + pin_number);  // Currently high, set low.
	else
		port->BSRR = 0b1UL << pin_number; // Set high.
}

void GPIO_output_high(uint8_t port_number, uint8_t pin_number) {
	assert(port_number < 7);
	assert(pin_number < 16);

	GPIO_TypeDef* port = PORTS[port_number];
	port->BSRR = 0b1UL << pin_number; // Set high.
}

void GPIO_output_low(uint8_t port_number, uint8_t pin_number) {
	assert(port_number < 7);
	assert(pin_number < 16);

	GPIO_TypeDef* port = PORTS[port_number];
	port->BSRR = 0b1UL << (16 + pin_number); // Set low.
}
