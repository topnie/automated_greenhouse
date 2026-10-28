/*
 * main_orchestrator_measurer.c
 *
 *  Created on: 7 paź 2026
 *      Author: Tomasz Pniewski
 */

#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "GPIO_manager.h"
#include "UART_driver.h"
#include "periodic_service_manager.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include "main_orchestrator_measurer.h"
#include "CAN_def.h"
#include "measurement_manager.h"
#include "utils.h"


static bool measurement_started;
static bool UART_idle;
static bool send_commands;
static bool new_data_available;

static volatile uint8_t tmp_meas_data[DATA_PACKET_LENGTH + CONTROL_SETTINGS_BUFFER_LENGTH];
static volatile uint8_t* ctrl_data = &tmp_meas_data[DATA_PACKET_LENGTH];
static volatile uint8_t uart_cmd_buffer[UART_MAX_CMD_LENGTH];
static uint8_t uart_response_buffer[2] = {UART_OK, UART_NOT_OK};
static volatile uint8_t len;

static uint8_t meas_data[DATA_PACKET_LENGTH * (MEASUREMENT_BUFFER_SIZE + 1)];
static uint16_t buffer_head;
static uint16_t buffer_tail;

static FDCAN_HandleTypeDef* hfdcan;
static ADC_HandleTypeDef* hadc;

// CAN variables.
static FDCAN_TxHeaderTypeDef tx_header;
static uint8_t tx_buffer[1];


// Command variables.
static bool vents_open;
static uint32_t periods_since_watering;

static volatile bool vent_command_success;
static volatile bool water_command_success;

typedef enum {
	CAN_TRANSACTION_IDLE = 0,
	CAN_TRANSACTION_STARTED = 1,
	CAN_TRANSMITTING = 2,
	CAN_ABORTED_TRANSMISSION = 3
} CAN_transaction_state;

typedef struct CAN_transaction_data {
	CAN_transaction_state state;
	uint8_t tries;
	uint32_t tx_location;
	uint32_t timeout_start;
} CAN_transaction_data_t;

static volatile CAN_transaction_data_t CAN_vent_transaction;
static volatile CAN_transaction_data_t CAN_water_transaction;

static inline bool are_CAN_transactions_active(void) {
	return ((CAN_vent_transaction.state != CAN_TRANSACTION_IDLE) ||
			(CAN_water_transaction.state != CAN_TRANSACTION_IDLE));
}

// CAN callbacks.
// We succesfully transmitted data to the second node (ACKed).
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
	if (CAN_vent_transaction.state == CAN_TRANSMITTING) {
		CAN_vent_transaction.state = CAN_TRANSACTION_IDLE;
		vent_command_success = true;
	}
	if (CAN_water_transaction.state == CAN_TRANSMITTING) {
		CAN_water_transaction.state = CAN_TRANSACTION_IDLE;
		water_command_success = true;
	}
}

// Transmission was not successful.
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
	if (CAN_vent_transaction.state == CAN_TRANSMITTING) {
		CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
		CAN_vent_transaction.tries++;
		vent_command_success = false;
	}
	if (CAN_water_transaction.state == CAN_TRANSMITTING) {
		CAN_water_transaction.state = CAN_TRANSACTION_STARTED;
		CAN_water_transaction.tries++;
		water_command_success = false;
	}
}

// Bus-off.
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

static void add_data_to_buffer(void) {
	tmp_meas_data[0] |= (vents_open * CTRL_VENT_ON_FLAG) |
					((periods_since_watering == 0) * CTRL_WATER_PUMPED_FLAG);
	memcpy(&meas_data[DATA_PACKET_LENGTH * buffer_head], tmp_meas_data, DATA_PACKET_LENGTH);
	buffer_head = (buffer_head + 1) % (MEASUREMENT_BUFFER_SIZE + 1);
	if (buffer_head == buffer_tail) {
		buffer_tail = (buffer_tail + 1) % (MEASUREMENT_BUFFER_SIZE + 1);
	}
}

static void enter_stop1(void)
{
	HAL_FDCAN_Stop(hfdcan);
	HAL_SuspendTick();
    __disable_irq();

    if (prepare_stop_mode_UART() != UART_STOP_READY ||
    		is_service_needed()) {
    	HAL_ResumeTick();
        __enable_irq();
        HAL_FDCAN_Start(hfdcan);
        return;
    }

    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    SystemClock_Config();
    HAL_ResumeTick();

    __enable_irq();
    HAL_FDCAN_Start(hfdcan);
}

void initialize_measurer(FDCAN_HandleTypeDef* hfdcan1, ADC_HandleTypeDef* hadc1) {
	hfdcan = hfdcan1;
	hadc = hadc1;
	HAL_FDCAN_Start(hfdcan);
	HAL_FDCAN_ActivateNotification(
	    hfdcan1,
	    FDCAN_IT_TX_COMPLETE |
	    FDCAN_IT_TX_ABORT_COMPLETE |
	    FDCAN_IT_BUS_OFF,
		FDCAN_TX_BUFFER0 |
		FDCAN_TX_BUFFER1 |
		FDCAN_TX_BUFFER2
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

	// Configure the pin we will use as an interrupt
	// for communication with controller node.
	configure_GPIO(1, 12, LED_config);
	// PB12 will be high when the MCU needs to send commands to the controller.
	GPIO_output_low(1, 12);

	initialize_sensors();
	initialize_UART();
	initialize_RTC();
	initialize_timer(MEASUREMENT_PERIOD_SEC);

	// Use default control variables.
	pack_u32(&ctrl_data[0], DEFAULT_RH);
	pack_s32(&ctrl_data[4], DEFAULT_TEMP);
	pack_u16(&ctrl_data[8], DEFAULT_MOISTURE);

	UART_idle = true;
	measurement_started = false;
	new_data_available = false;
	send_commands = false;
}

static void CAN_advance_transaction(volatile CAN_transaction_data_t* transaction,
		uint32_t transmission_identifier, uint32_t FDCAN_DLC) {
	switch (transaction->state) {
		case CAN_TRANSACTION_STARTED:
			// We need to start the CAN transmission.
			if (transaction->tries == MAX_CAN_TX_TRIES) {
				transaction->state = CAN_TRANSACTION_IDLE;
			}
			else {
				// Start the transmission.
				tx_header.Identifier = transmission_identifier;
				tx_header.IdType = FDCAN_STANDARD_ID;
				tx_header.TxFrameType = FDCAN_DATA_FRAME;
				tx_header.DataLength = FDCAN_DLC;
				tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				tx_header.BitRateSwitch = FDCAN_BRS_OFF;
				tx_header.FDFormat = FDCAN_FD_CAN;
				tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				tx_header.MessageMarker = 0;
				if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_buffer) == HAL_OK) {
					transaction->tx_location =
							HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
					transaction->state = CAN_TRANSMITTING;
					transaction->timeout_start = HAL_GetTick();
				}
				else {
					transaction->tries++;
				}
			}
			break;
		case CAN_TRANSMITTING:
			if (HAL_GetTick() - transaction->timeout_start > CAN_TX_TIMEOUT_MS) {
				// The message timed out.
				HAL_FDCAN_AbortTxRequest(hfdcan, transaction->tx_location);
				transaction->state = CAN_ABORTED_TRANSMISSION;
				transaction->timeout_start = HAL_GetTick();
			}
			break;
		case CAN_ABORTED_TRANSMISSION:
			if (HAL_GetTick() - transaction->timeout_start > CAN_ABORT_TIMEOUT_MS) {
				transaction->state = CAN_TRANSACTION_STARTED;
				transaction->tries++;
			}
			break;
	}
}

static void UART_handle_command() {
	UART_write_request write_result;
	switch (uart_cmd_buffer[0]) {
		case UART_GET_LAST_MEASUREMENT_ID:
			write_result = write_from_ring_buffer(meas_data,
					((buffer_head - 1) * DATA_PACKET_LENGTH) % (DATA_PACKET_LENGTH * (MEASUREMENT_BUFFER_SIZE + 1)),
					buffer_head * DATA_PACKET_LENGTH,
					DATA_PACKET_LENGTH * (MEASUREMENT_BUFFER_SIZE + 1));

			if (write_result == UART_TRANSMISSION_STARTED) {
				UART_idle = false;
			}
			break;
		case UART_GET_ALL_MEASUREMENTS_ID:
			write_result = write_from_ring_buffer(meas_data,
					buffer_tail * DATA_PACKET_LENGTH,
					buffer_head * DATA_PACKET_LENGTH,
					DATA_PACKET_LENGTH * (MEASUREMENT_BUFFER_SIZE + 1));

			if (write_result == UART_TRANSMISSION_STARTED) {
				UART_idle = false;
			}
			break;
		case UART_GET_CTRL_SETTINGS_ID:
			write_result = write_from_ring_buffer(ctrl_data,
					0,
					CONTROL_SETTINGS_LENGTH,
					DATA_PACKET_LENGTH + CONTROL_SETTINGS_BUFFER_LENGTH);

			if (write_result == UART_TRANSMISSION_STARTED) {
				UART_idle = false;
			}
			break;
		case UART_CHANGE_CTRL_SETTINGS_ID:
			if (len < (1 + CONTROL_SETTINGS_LENGTH)) {
				// Send NOT_OK.
				write_result = write_from_ring_buffer(uart_response_buffer,
					1, 0, 2);

				if (write_result == UART_TRANSMISSION_STARTED) {
					UART_idle = false;
				}
			}
			else {
				memcpy(ctrl_data, &uart_cmd_buffer[1], CONTROL_SETTINGS_LENGTH);
				// Send OK.
				write_result = write_from_ring_buffer(uart_response_buffer,
						0, 1, 2);

				if (write_result == UART_TRANSMISSION_STARTED) {
					UART_idle = false;
				}
			}

			break;
	}
}

static inline float hum_ratio(int32_t temp, uint32_t rh, uint32_t pressure) {
	int32_t exponent_arg = 1727 * temp / (temp + 23730);
	float sat_pressure = 610.78f * expf(((float) exponent_arg) / 100.0f);
	float vapor_pressure = (((float) rh) / 102400.0f) * sat_pressure;
	return 0.622f * vapor_pressure / (((float)pressure / 256.0f) - vapor_pressure);
}

static uint32_t wanted_rh;
static int32_t wanted_temp;
static uint16_t wanted_moisture;
static uint16_t SHT31_temp;
static int32_t inside_temp;
static uint16_t SHT31_rh;
static uint32_t inside_rh;
static uint32_t outside_rh;
static int32_t outside_temp;
static uint32_t outside_pressure;
static uint16_t adc_voltage;


static void update_control() {
	unpack_u32(&ctrl_data[0], &wanted_rh);
	unpack_s32(&ctrl_data[4], &wanted_temp);
	unpack_u16(&ctrl_data[8], &wanted_moisture);
	// First, we convert the SHT31 temperature and humidity readings
	// to the format used by the BME280 sensor.
	unpack_u16(&tmp_meas_data[1], &SHT31_temp);
	inside_temp = convert_temp_SHT31_to_BME280(SHT31_temp);
	unpack_u16(&tmp_meas_data[3], &SHT31_rh);
	inside_rh = convert_rh_SHT31_to_BME280(SHT31_rh);
	unpack_u32(&tmp_meas_data[11], &outside_rh);
	unpack_u32(&tmp_meas_data[15], &outside_pressure);
	unpack_s32(&tmp_meas_data[7], &outside_temp);
	unpack_u16(&tmp_meas_data[19], &adc_voltage);
	// Vent control: temperature is most important.
	if (inside_temp > wanted_temp - TEMPERATURE_MARGIN / 2 &&
		inside_temp < wanted_temp + TEMPERATURE_MARGIN / 2 &&
		inside_rh > wanted_rh - RH_MARGIN / 2 &&
		inside_rh < wanted_rh + RH_MARGIN / 2) {
		if (vents_open) {
			// If everything is fine, then we should do nothing.
			CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
			CAN_vent_transaction.tries = 0;
		}
	}
	else if (inside_temp > wanted_temp + TEMPERATURE_MARGIN) {
		// It's too hot inside.
		if (!vents_open && TEMPERATURE_DIFF < inside_temp - outside_temp) {
			// If the vents are closed, we can open them to let colder air in.
			CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
			CAN_vent_transaction.tries = 0;
		}
		else if (vents_open && outside_temp - inside_temp > TEMPERATURE_DIFF) {
			// Close the vents if the air outside is hotter than we want.
			CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
			CAN_vent_transaction.tries = 0;
		}
	}
	else if (inside_temp < wanted_temp - TEMPERATURE_MARGIN) {
		// It's too cold inside.
		if (!vents_open && outside_temp - inside_temp > TEMPERATURE_DIFF) {
			// If the vents are closed, we can open them to let hotter air in.
			CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
			CAN_vent_transaction.tries = 0;
		}
		else if (vents_open && TEMPERATURE_DIFF < inside_temp - outside_temp) {
			// Close the vents if the air outside is colder than we want.
			CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
			CAN_vent_transaction.tries = 0;
		}
	}
	else {
		// The temperature is in the correct range.
		// We can try to balance the humidity, if we can.
		float inside_hum_ratio = hum_ratio(inside_temp, inside_rh, outside_pressure);
		float outside_hum_ratio = hum_ratio(outside_temp, outside_rh, outside_pressure);
		if (inside_rh > wanted_rh + RH_MARGIN) {
			if (!vents_open && (outside_hum_ratio + HUM_DIFF < inside_hum_ratio)) {
				// If the vents are closed, we can open them to let less humid air in.
				CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
				CAN_vent_transaction.tries = 0;
			}
			else if (vents_open && (outside_hum_ratio > inside_hum_ratio + HUM_DIFF)) {
				// Close the vents if the air outside is more humid than we want.
					CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
					CAN_vent_transaction.tries = 0;
			}
		}
		else if (inside_rh < wanted_rh - RH_MARGIN) {
			if (!vents_open && (outside_hum_ratio > inside_hum_ratio + HUM_DIFF)) {
				// If the vents are closed, we can open them to let more humid air in.
				CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
				CAN_vent_transaction.tries = 0;
			}
			else if (vents_open && (outside_hum_ratio + HUM_DIFF < inside_hum_ratio)) {
				// Close the vents if the air outside is less humid than we want.
				CAN_vent_transaction.state = CAN_TRANSACTION_STARTED;
				CAN_vent_transaction.tries = 0;
			}
		}
	}

	// If the voltage is higher than the threshold, we water the plant.
	if (adc_voltage > wanted_moisture && periods_since_watering > MINIMUM_WATERING_WAIT_PERIODS) {
		CAN_water_transaction.state = CAN_TRANSACTION_STARTED;
		CAN_water_transaction.tries = 0;
	}
}

void main_loop_measurer(void) {
	if (!measurement_started && UART_idle && !send_commands && !are_CAN_transactions_active() && !new_data_available) {
		enter_stop1();
	}

	if (is_service_needed()) {
		// We need to perform the periodic measurements.
		if (!measurement_started) {
			start_measurements(hadc);
			measurement_started = true;
		}
		if (advance_measurements()) {
			collect_measurements(tmp_meas_data);
			measurement_started = false;

			update_control();

			if (are_CAN_transactions_active()) {
				// We need to send the commands to the second node.
				send_commands = true;
				GPIO_output_high(1, 12);
				// Give the other node time to wake up.
				HAL_Delay(2);
			}
			else {
				new_data_available = true;
				periods_since_watering++;
			}
			service_done();
		}
	}

	len = UART_MAX_CMD_LENGTH;
	if (UART_idle) {

		if (new_data_available) {
			add_data_to_buffer();
			new_data_available = false;
		}

		UART_read_result read_result = check_read(uart_cmd_buffer, &len);

		if ((read_result == UART_READ_READY) && (len > 0)) {
			UART_handle_command();
		}
	}
	else {
		if (check_write() != UART_TRANSMISSION_IN_PROGRESS) {
			UART_idle = true;
		}
	}
	if (are_CAN_transactions_active()) {
		// We only serve one transaction at a time, if there is
		// more than one wanting to be started in software.
		if (CAN_vent_transaction.state != CAN_TRANSACTION_IDLE) {
			tx_buffer[0] = !vents_open; // We want to change the vent state.
			CAN_advance_transaction(&CAN_vent_transaction,
					CAN_VENTS_ID, CAN_VENTS_DLC);
		}
		else {
			CAN_advance_transaction(&CAN_water_transaction,
					CAN_WATER_ID, CAN_WATER_DLC);
		}

	}
	else if (send_commands) {
		send_commands = false;
		if (vent_command_success) {
			vents_open = !vents_open;
			vent_command_success = false;
		}
		if (water_command_success) {
			periods_since_watering = 0;
			water_command_success = false;
		}
		else {
			periods_since_watering++;
		}
		new_data_available = true;
		GPIO_output_low(1, 12);
	}
}
