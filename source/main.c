#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <SPI_MB.h>
#include <serial_monitor.h>
#include <Battery.h>
#include <shutdown_circuit.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_ERR);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 10000

typedef enum
{
	STATE_RESET,   // systemreset
	STATE_IDLE,    // idle state
	STATE_PRECHARGE, // precharge state
	STATE_RUNNING, // battery management process
	STATE_ERROR,   // error state
} SystemState_t;

int main(void)
{
	

	// variables
	SystemState_t state = STATE_RESET;
	static bool previous_ecu_state = BATTERY_OFF;
	bool current_ecu_state = BATTERY_OFF;
	uint32_t event_flags = 0;

	// Initialize
	can_init();

	can_ivt_init();

	spi_adbms1818_hw_init();

	serial_monitor_init();

	battery_init();

	sdc_init();

	while (1)
	{		
		//serial monitor daten senden
		serial_monitor((uint8_t *)&battery_values, sizeof(battery_values));

		// wait for event
		event_flags = k_event_wait(&error_to_main, EVT_ERROR_BIT,false, K_NO_WAIT);

		if (event_flags & EVT_ERROR_BIT) {
		state = STATE_ERROR;
		LOG_INF("got Error-Event! switching to Error-State.\n");
		}

		switch (state)
		{
		case STATE_RESET:

			break;

		case STATE_IDLE:
			LOG_INF("Waiting for ECU OK");

			current_ecu_state = can_get_ecu_state();

			if (previous_ecu_state == BATTERY_OFF && current_ecu_state == BATTERY_ON)
			{
				state = STATE_PRECHARGE;
				LOG_INF("ECU rising edge, entering precharge state");
			}

			break;
		
		case STATE_PRECHARGE:
			// precharge logic
			if (battery_precharge_logic() == 0)
			{
				state = STATE_RUNNING;
			}
			else
			{
				state = STATE_ERROR;
				LOG_ERR("Precharge failed, entering error state");
			}
			break;

		case STATE_RUNNING:
			// rename old charging function
			battery_charging();
			LOG_INF("Battery management process running");

			current_ecu_state = can_get_ecu_state();

			if (previous_ecu_state == BATTERY_ON && current_ecu_state == BATTERY_OFF)
			{
				state = STATE_IDLE;
				LOG_INF("ECU falling edge, entering idle state");
			}

			break;

		case STATE_ERROR:
			// set all relays to 0
			sdc_shutdown();
			if (battery_get_error_code() == 0)
			{
				state = STATE_IDLE;
				LOG_INF("Errors resolved, entering idle state");
			}
			break;
		}

	}
	return 0;
}

