/*
 * UART_driver.c
 *
 *  Created on: 21 wrz 2026
 *      Author: Tomasz Pniewski
 */

#include <assert.h>
#include <stddef.h>
#include "UART_driver.h"
#include "GPIO_manager.h"
#include "stm32g431xx.h"

enum {
    UART_FRAME_BUFFER_SIZE = 5,
    UART_RX_BUFFER_SIZE = 64
};

typedef enum {
	UART_TX_UNINITIALIZED,
    UART_TX_IDLE,
    UART_TX_IN_PROGRESS,
    UART_TX_READY
} UART_tx_state;

typedef enum {
	UART_RX_UNINITIALIZED,
    UART_RX_IDLE,
    UART_RX_ACTIVE,
} UART_rx_state;


static volatile UART_rx_state rx_state;
static volatile UART_tx_state tx_state;

static volatile uint8_t read_error_pending;
static volatile uint8_t write_error_pending;

static volatile uint8_t n_frames; // Number of frames available to read.

static volatile uint8_t write_frame_id;
static volatile uint8_t read_frame_id;
// We need to keep track where in the buffer the frames live.
static volatile uint8_t frame_boundaries[UART_FRAME_BUFFER_SIZE][2];

static volatile uint8_t reception_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t buffer_tail;
static volatile uint16_t buffer_head;

static const uint8_t* volatile transmission_buffer;
static volatile uint16_t buffer_size;
static volatile uint16_t transmission_tail;
static volatile uint16_t transmission_head;

static uint8_t next_from_transmission_buf(void) {
	uint16_t tmp_index = transmission_tail;
	transmission_tail = (transmission_tail + 1) % buffer_size;
	return transmission_buffer[tmp_index];
}

static void reset_UART_peripheral(void) {
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
}

static void recover_from_error(void) {
	// Prevent interrupts.
	USART1->CR1 &= ~(USART_CR1_IDLEIE |
	                 USART_CR1_TCIE);

	USART1->CR3 &= ~(USART_CR3_RXFTIE |
	                 USART_CR3_TXFTIE |
	                 USART_CR3_EIE |
	                 USART_CR3_WUFIE);

	// Disable peripheral.
	USART1->CR1 &= ~(USART_CR1_RE |
	                 USART_CR1_TE |
	                 USART_CR1_UE);

	NVIC_DisableIRQ(USART1_IRQn);
	NVIC_ClearPendingIRQ(USART1_IRQn);
	read_error_pending = 1;
	write_error_pending = 1;
	tx_state = UART_TX_UNINITIALIZED;
	rx_state = UART_RX_UNINITIALIZED;
}

void USART1_IRQHandler(void) {
	if (USART1->ISR & USART_ISR_WUF) {

		USART1->ICR = USART_ICR_WUCF;
	    USART1->CR3 &= ~USART_CR3_WUFIE;
	    USART1->CR1 &= ~USART_CR1_UESM;
	}

    if (USART1->ISR & (USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE)) {
    	recover_from_error();
    	return;
    }

    while ((USART1->ISR & USART_ISR_TXE) &&
    		(tx_state == UART_TX_IN_PROGRESS) &&
			(transmission_tail != transmission_head)) {
    	// FIFO is not full, we transmit (if needed).
    	USART1->TDR = next_from_transmission_buf();
    }

    if ((tx_state == UART_TX_IN_PROGRESS) &&
    		(transmission_tail == transmission_head)) {
        USART1->CR3 &= ~USART_CR3_TXFTIE; // Disable transmition.
        USART1->CR1 |= USART_CR1_TCIE; // Enable completion of transmittion interrupt.
    }

    while (USART1->ISR & USART_ISR_RXNE) {
    	if (rx_state == UART_RX_IDLE) {
    		if (n_frames >= UART_FRAME_BUFFER_SIZE) {
    			recover_from_error();
    	        return;
    		}
    		rx_state = UART_RX_ACTIVE;
    		frame_boundaries[write_frame_id][0] = buffer_head;
    	}
    	reception_buffer[buffer_head] = USART1->RDR;
    	buffer_head = (buffer_head + 1) % UART_RX_BUFFER_SIZE;
    	if (buffer_head == buffer_tail) { // Buffer full, we reset the driver.
    		recover_from_error();
    		return;
    	}
    }

    if ((USART1->ISR & USART_ISR_TC) &&
    		(tx_state == UART_TX_IN_PROGRESS)) {
    	tx_state = UART_TX_READY;
    	USART1->CR3 &= ~USART_CR3_TXFTIE;
    	USART1->CR1 &= ~USART_CR1_TCIE;
    	USART1->ICR = USART_ICR_TCCF;
    }

    if ((USART1->ISR & USART_ISR_IDLE)) {
    	USART1->ICR = USART_ICR_IDLECF;
    	if (rx_state == UART_RX_ACTIVE) {
    		rx_state = UART_RX_IDLE;
    		// End the current frame, if there is any.
    		if (n_frames >= UART_FRAME_BUFFER_SIZE) {
    		// We overflow, restart the driver.
    		    recover_from_error();
    		    return;
    		}
    		frame_boundaries[write_frame_id][1] = buffer_head;
    		write_frame_id = (write_frame_id + 1) % UART_FRAME_BUFFER_SIZE;
    		n_frames++;
    	}
    }

}

static void configure_UART_peripheral(void) {
	// We use the PC4 and PC5 pins as TX and RX.
	GPIO_config UART_TX_config = {
		.mode = GPIO_PIN_MODE_ALTERNATE_FUNCTION,
		.output_type = GPIO_OUTPUT_PUSH_PULL,
		.output_speed = GPIO_OUTPUT_SPEED_HIGH,
		.pull_up_down = GPIO_PULL_NONE,
		.AF = 0b0111U
	};
	GPIO_config UART_RX_config = {
		.mode = GPIO_PIN_MODE_ALTERNATE_FUNCTION,
		.output_type = GPIO_OUTPUT_PUSH_PULL,
		.output_speed = GPIO_OUTPUT_SPEED_HIGH,
		.pull_up_down = GPIO_PULL_UP,
		.AF = 0b0111U
	};

	configure_GPIO(2, 4, UART_TX_config);
	configure_GPIO(2, 5, UART_RX_config);

	// Clock configuration.
	// Select HSI16 as clock to enable wake up from Stop mode.
	RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);
	RCC->CCIPR |= (2U << RCC_CCIPR_USART1SEL_Pos);

	// Enable peripheral clock.
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Then, we need to configure the UART peripheral.
	// Disable the peripheral.
	USART1->CR1 &= ~(USART_CR1_UE);

	// Now, configure the rest of the control registers.
	USART1->CR1 |= USART_CR1_FIFOEN; // Use hardware FIFOs.
	USART1->CR1 &= ~(USART_CR1_M1);
	USART1->CR1 &= ~(USART_CR1_M0); // Use 8 data bits.
	USART1->CR1 &= ~(USART_CR1_OVER8); // Use 16x oversampling.
	USART1->CR1 &= ~(USART_CR1_MME); // Disable mute mode.
	USART1->CR1 &= ~(USART_CR1_PCE); // No parity bit.
	USART1->BRR = 0x008B; // Set baud rate to 115200.

	USART1->CR3 &= ~(USART_CR3_RXFTCFG); // We will drain rx FIFO ASAP.
	USART1->CR3 &= ~(USART_CR3_TXFTCFG);
	// We refill transmit FIFO when it's half full.
	USART1->CR3 |= (0b010UL << USART_CR3_TXFTCFG_Pos);

	USART1->CR3 |= USART_CR3_RXFTIE; // Interrupt for read.
	USART1->CR1 |= USART_CR1_IDLEIE; // Interrupt for idle on rx.

	USART1->CR3 &= ~(USART_CR3_WUFIE); // Disable wake up interrupt.
	USART1->CR3 = (USART1->CR3 & ~USART_CR3_WUS) | USART_CR3_WUS_1; // Wake up when we receive data.
	USART1->CR3 |= USART_CR3_EIE; // Enable error interrupts to be generated.

	// Enable the peripheral.
	USART1->CR1 |= USART_CR1_UE;
	// Enable the receiver.
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_TE; // Enable transmission.
}

static void restart_UART(void) {
	reset_UART_peripheral();

	// Driver variables initialization.
	rx_state = UART_RX_IDLE;
	tx_state = UART_TX_IDLE;

	n_frames = 0;
	write_frame_id = 0;
	read_frame_id = 0;
	buffer_head = 0;
	buffer_tail = 0;

	transmission_buffer = NULL;
	transmission_tail = 0;
	transmission_head = 0;
	buffer_size = 0;

	configure_UART_peripheral();
	NVIC_ClearPendingIRQ(USART1_IRQn);
}

void initialize_UART(void) {
	NVIC_DisableIRQ(USART1_IRQn);
	restart_UART();
	read_error_pending = 0;
	write_error_pending = 0;
	NVIC_EnableIRQ(USART1_IRQn);
}

UART_stop_result prepare_stop_mode_UART(void)
{
	// Assume all interrupts are masked here.
    // We don't want to sleep while there is unfinished work.
    if (tx_state != UART_TX_IDLE ||
        rx_state != UART_RX_IDLE ||
        n_frames != 0 ||
        read_error_pending ||
        write_error_pending) {
        NVIC_EnableIRQ(USART1_IRQn);
        return UART_STOP_NOT_READY;
    }

    // Make sure the peripheral itself is idle and the receiver is enabled.
    if ((USART1->ISR & USART_ISR_BUSY) ||
        !(USART1->ISR & USART_ISR_REACK) ||
        (USART1->ISR & USART_ISR_RXNE)) {

        NVIC_EnableIRQ(USART1_IRQn);
        return UART_STOP_NOT_READY;
    }

    if (USART1->ISR & (USART_ISR_FE |
        USART_ISR_ORE | USART_ISR_NE)) {
    	return UART_STOP_NOT_READY;
    }

    if (USART1->ISR & USART_ISR_IDLE) {
        return UART_STOP_NOT_READY;
    }

    if (USART1->ISR & USART_ISR_WUF) {
        return UART_STOP_NOT_READY;
    }

    // Enable USART wake-up from Stop mode.
    USART1->CR1 |= USART_CR1_UESM;
    USART1->CR3 |= USART_CR3_WUFIE;


    return UART_STOP_READY;
}

UART_write_request write_from_ring_buffer(const uint8_t* data, uint16_t tail, uint16_t head,
		uint16_t buffer_len) {
	if ((data == NULL) || (tail == head) || (buffer_len == 0)) {
		return UART_TRANSMISSION_NOT_STARTED;
	}
	NVIC_DisableIRQ(USART1_IRQn);
	if (tx_state != UART_TX_IDLE || write_error_pending) {
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_TRANSMISSION_NOT_STARTED;
	}
	tx_state = UART_TX_IN_PROGRESS;
	transmission_buffer = data;
	transmission_tail = tail;
	transmission_head = head;
	buffer_size = buffer_len;

    // Prime the TX FIFO.
    while ((USART1->ISR & USART_ISR_TXE) &&
           (transmission_tail != transmission_head)) {
        USART1->TDR = next_from_transmission_buf();
    }
    if (transmission_tail == transmission_head) {
        // Everything is already in the FIFO.
        USART1->CR1 |= USART_CR1_TCIE;
    }
    else {
        // More data remains, interrupt when FIFO reaches threshold.
        USART1->CR3 |= USART_CR3_TXFTIE;
    }
	NVIC_EnableIRQ(USART1_IRQn);
	return UART_TRANSMISSION_STARTED;
}


UART_write_result check_write(void) {
	NVIC_DisableIRQ(USART1_IRQn);
	if (write_error_pending) {
		if (write_error_pending && read_error_pending) {
			restart_UART();
		}
		write_error_pending = 0;
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_TRANSMISSION_ERROR;
	}
	if (tx_state == UART_TX_READY) {
		tx_state = UART_TX_IDLE;
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_TRANSMISSION_READY;
	}
	else {
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_TRANSMISSION_IN_PROGRESS;
	}
}

UART_read_result check_read(uint8_t* data, uint8_t* len) {
	NVIC_DisableIRQ(USART1_IRQn);
	if (read_error_pending) {
		if (write_error_pending && read_error_pending) {
			restart_UART();
		}
		read_error_pending = 0;
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_READ_ERROR;
	}
	else if (n_frames == 0) {
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_READ_NOT_READY;
	}
	// First, compute, how big the frame is.
	uint8_t frame_size;
	uint8_t frame_beginning = frame_boundaries[read_frame_id][0];
	uint8_t frame_ending = frame_boundaries[read_frame_id][1];
	assert(buffer_tail == frame_beginning);
	if (frame_beginning <= frame_ending) {
		frame_size = frame_ending - frame_beginning;
	}
	else {
		frame_size = (UART_RX_BUFFER_SIZE - frame_beginning) + (frame_ending);
	}
	if (*len < frame_size) {
		NVIC_EnableIRQ(USART1_IRQn);
		return UART_BUFFER_TOO_SMALL;
	}
	*len = frame_size;
	// It fits, so we copy it into the user buffer.
	uint8_t user_index = 0;
	while (buffer_tail != frame_ending) {
		data[user_index] = reception_buffer[buffer_tail];
		user_index++;
		buffer_tail++;
		buffer_tail %= UART_RX_BUFFER_SIZE;
	}

	read_frame_id = (read_frame_id + 1) % UART_FRAME_BUFFER_SIZE;
	n_frames--;
	NVIC_EnableIRQ(USART1_IRQn);
	return UART_READ_READY; // The read was successful.
}
