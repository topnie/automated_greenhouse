/*
 * wakeup_manager.h
 *
 *  Created on: 23 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef PERIODIC_SERVICE_MANAGER_H_
#define PERIODIC_SERVICE_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

void initialize_RTC(void);
void initialize_timer(uint16_t timer_seconds);
bool is_service_needed(void);
void service_done(void);

#endif /* PERIODIC_SERVICE_MANAGER_H_ */
