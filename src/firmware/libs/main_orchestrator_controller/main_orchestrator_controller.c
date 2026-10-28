/*
 * main_orchestrator_controller.c
 *
 *  Created on: 7 paź 2026
 *      Author: Tomasz Pniewski
 */

#include <string.h>
#include <stdbool.h>
#include "GPIO_manager.h"
#include "CAN_def.h"
#include "main_orchestrator_controller.h"
#include "stm32g4xx_hal.h"
#include "main.h"

static bool communication_pin_high;
static bool vents_open;
static volatile bool requested_vent_position;
static volatile bool vent_request_pending;

typedef enum {
	VENT_IDLE,
	VENT_STARTED,
	VENT_FIRST_MOVING,
	VENT_SECOND_MOVING
} vent_state_t;

typedef enum {
	PUMP_IDLE,
	PUMP_STARTED,
	PUMPING,
} pump_state_t;

static volatile pump_state_t pump_state;
static vent_state_t vent_state;

static uint32_t pump_time_start;
static uint32_t vent_time_start;

static TIM_HandleTypeDef* htim;
static FDCAN_HandleTypeDef* hfdcan;

// CAN variables.
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_buffer[64];

// CAN callbacks.
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
				&rx_header, rx_buffer) != HAL_OK) {
			return;
	    }
		if (rx_header.Identifier == CAN_VENTS_ID &&
				rx_header.DataLength == CAN_VENTS_DLC) {
		    if (rx_buffer[0] <= 1U) {
		    	requested_vent_position = (rx_buffer[0] != 0U);
				vent_request_pending = true;
		    }
		}
		if (rx_header.Identifier == CAN_WATER_ID &&
				rx_header.DataLength == CAN_WATER_DLC) {
			if (pump_state == PUMP_IDLE) {
				pump_state = PUMP_STARTED;
			}
		}
	}
}

// Bus-off.
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}


static void enter_stop1(void)
{
	HAL_FDCAN_Stop(hfdcan);
	HAL_SuspendTick();
    __disable_irq();

    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    SystemClock_Config();
    HAL_ResumeTick();
    __enable_irq();
    HAL_FDCAN_Start(hfdcan);
}

void initialize_controller(FDCAN_HandleTypeDef* hfdcan1, TIM_HandleTypeDef* htim3) {
	hfdcan = hfdcan1;
	htim = htim3;
	// Configure CAN.
	HAL_FDCAN_Start(hfdcan);
	HAL_FDCAN_ActivateNotification(
	    hfdcan,
	    FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	    FDCAN_IT_BUS_OFF,
		0
	);

	GPIO_config LED_config = {
		.mode = GPIO_PIN_MODE_OUTPUT,
		.output_type = GPIO_OUTPUT_PUSH_PULL,
		.output_speed = GPIO_OUTPUT_SPEED_LOW,
		.pull_up_down = GPIO_PULL_DOWN,
		.AF = 0
	};

	configure_GPIO(0, 5, LED_config);
	GPIO_output_low(0, 5); // Disable the board LED.

	configure_GPIO(2, 2, LED_config);
	GPIO_output_low(2, 2); // Disable the ventilator.

	configure_GPIO(2, 3, LED_config);
	GPIO_output_low(2, 3); // Disable the pump.

	pump_state = PUMP_IDLE;
	// Set the vent servos to the closed position.
	vent_state = VENT_STARTED;
}

void main_loop_controller(void) {
	communication_pin_high = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET);
	if (!communication_pin_high && !vent_request_pending
			&& (vent_state == VENT_IDLE) && (pump_state == PUMP_IDLE)) {
		enter_stop1();
	}

	uint16_t vent1_command;
	uint16_t vent2_command;

	if (vents_open) {
		vent1_command = VENT1_OPEN;
		vent2_command = VENT2_OPEN;
	}
	else {
		vent1_command = VENT1_CLOSE;
		vent2_command = VENT2_CLOSE;
	}

	switch (vent_state) {
		case VENT_IDLE:
			if (vent_request_pending) {
				vent_request_pending = false;
				if (vents_open != requested_vent_position) {
					vents_open = requested_vent_position;
					vent_state = VENT_STARTED;
				}
			}
			break;
		case VENT_STARTED:
			HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, vent1_command);
			HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
			vent_state = VENT_FIRST_MOVING;
			vent_time_start = HAL_GetTick();
			break;
		case VENT_FIRST_MOVING:
			if (HAL_GetTick() - vent_time_start > VENT_MOVEMENT_MS) {
				__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, vent2_command);
				HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
				vent_state = VENT_SECOND_MOVING;
				vent_time_start = HAL_GetTick();
			}
			break;
		case VENT_SECOND_MOVING:
			if (HAL_GetTick() - vent_time_start > VENT_MOVEMENT_MS) {
				HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
				if (vents_open) {
					GPIO_output_high(2, 2); // Enable the fan.
				}
				else {
					GPIO_output_low(2, 2); // Disable the fan.
				}
				vent_state = VENT_IDLE;
			}
			break;
	}

	switch (pump_state) {
		case PUMP_STARTED:
			GPIO_output_high(2, 3);
			pump_state = PUMPING;
			pump_time_start = HAL_GetTick();
			break;
		case PUMPING:
			if (HAL_GetTick() - pump_time_start > PUMP_TIME_MS) {
				GPIO_output_low(2, 3);
				pump_state = PUMP_IDLE;
			}
			break;
	}
}
