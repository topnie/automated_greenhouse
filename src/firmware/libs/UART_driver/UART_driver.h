/*
 * UART_driver.h
 *
 *  Created on: 21 wrz 2026
 *      Author: Tomasz Pniewski
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include <stdint.h>

typedef enum {
	UART_READ_NOT_READY = 0,
	UART_READ_READY = 1,
	UART_READ_ERROR = 2,
	UART_BUFFER_TOO_SMALL = 3
} UART_read_result;

typedef enum {
	UART_TRANSMISSION_STARTED = 0,
	UART_TRANSMISSION_NOT_STARTED = 1
} UART_write_request;

typedef enum {
	UART_TRANSMISSION_IN_PROGRESS = 0,
	UART_TRANSMISSION_READY = 1,
	UART_TRANSMISSION_ERROR = 2
} UART_write_result;

typedef enum {
    UART_STOP_READY,
    UART_STOP_NOT_READY
} UART_stop_result;


void initialize_UART(void);
UART_write_request write_from_ring_buffer(const uint8_t* data, uint16_t tail, uint16_t head,
		uint16_t buffer_len);
UART_write_result check_write(void);
UART_read_result check_read(uint8_t* data, uint8_t* len);
UART_stop_result prepare_stop_mode_UART(void);

#endif /* UART_DRIVER_H_ */
