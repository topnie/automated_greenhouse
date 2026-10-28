/*
 * main_orchestrator_measurer.h
 *
 *  Created on: 7 paź 2026
 *      Author: Tomasz Pniewski
 */

#ifndef MAIN_ORCHESTRATOR_MEASURER_H_
#define MAIN_ORCHESTRATOR_MEASURER_H_

#include "stm32g4xx_hal.h"

// How much can the temperature differ from the specified amount, in hundreds of DegC.
#define TEMPERATURE_MARGIN 200
// Humidity margin, 1024 = 1%.
#define RH_MARGIN 5120

#define DEFAULT_TEMP (23 * 100)
#define DEFAULT_RH (1024 * 55)
#define DEFAULT_MOISTURE 2482

// Minimum temperature and hum ratio that counts as a difference between inside / outside.
#define TEMPERATURE_DIFF 100
#define HUM_DIFF 0.001

#define MEASUREMENT_PERIOD_SEC 60
#define MEASUREMENT_BUFFER_SIZE 576
#define DATA_PACKET_LENGTH 21
#define CONTROL_SETTINGS_LENGTH 10
#define CONTROL_SETTINGS_BUFFER_LENGTH 12

#define MINIMUM_WATERING_WAIT_MIN 60
#define MINIMUM_WATERING_WAIT_PERIODS ((60 * MINIMUM_WATERING_WAIT_MIN) / MEASUREMENT_PERIOD_SEC)

#define CTRL_VENT_ON_FLAG (1U << 6)
#define CTRL_WATER_PUMPED_FLAG (1U << 5)

// UART command IDs.
#define UART_MAX_CMD_LENGTH (1 + CONTROL_SETTINGS_LENGTH)
#define UART_GET_LAST_MEASUREMENT_ID 0x0
#define UART_GET_ALL_MEASUREMENTS_ID 0x1
#define UART_GET_CTRL_SETTINGS_ID 0x2
#define UART_CHANGE_CTRL_SETTINGS_ID 0x3
#define UART_OK 0x4
#define UART_NOT_OK 0x5

void initialize_measurer(FDCAN_HandleTypeDef* hfdcan1, ADC_HandleTypeDef* hadc1);
void main_loop_measurer(void);

#endif /* MAIN_ORCHESTRATOR_MEASURER_H_ */
