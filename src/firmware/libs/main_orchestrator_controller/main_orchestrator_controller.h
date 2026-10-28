/*
 * main_orchestrator.h
 *
 *  Created on: 7 paź 2026
 *      Author: Tomasz Pniewski
 */

#ifndef MAIN_ORCHESTRATOR_CONTROLLER_H_
#define MAIN_ORCHESTRATOR_CONTROLLER_H_

#include "stm32g4xx_hal.h"

#define VENT1_CLOSE 200
#define VENT2_CLOSE 100

#define VENT1_OPEN 100
#define VENT2_OPEN 200

#define VENT_MOVEMENT_MS 2000
#define PUMP_TIME_MS 1000

void initialize_controller(FDCAN_HandleTypeDef* hfdcan1, TIM_HandleTypeDef* htim3);
void main_loop_controller(void);

#endif /* MAIN_ORCHESTRATOR_CONTROLLER_H_ */
