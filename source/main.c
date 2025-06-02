/**
 * @file main.c
 * @brief Main application for the Battery Management System (BMS).
 * @author renolluc / grossfa2
 * @date 02.06.2025
 */

/** @brief Zephyr-Framework includes */
//#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/** @brief Project includes */
#include <can_bus.h>
#include <spi_mb.h>
#include <serial_monitor.h>
#include <battery.h>
#include <shutdown_circuit.h>

/**
 * @brief Sets the name and logging levels for this module.
 *
 * Possible log levels:
 * - LOG_LEVEL_NONE
 * - LOG_LEVEL_ERR
 * - LOG_LEVEL_WRN
 * - LOG_LEVEL_INF
 * - LOG_LEVEL_DBG
 */
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/** @brief LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/** @brief define sleep time; 1000 msec = 1 sec */
#define SLEEP_TIME_MS 250
/** @brief define precharge timeout; 1000 msec = 1 sec */
#define PRECHARGE_TIMEOUT_MS 1000 // 1

/** @brief typedef for the Statemachine */
typedef enum
{
	STATE_TEST,   // systemreset
	STATE_IDLE,    // idle state
	STATE_PRECHARGE, // precharge state
	STATE_RUNNING, // battery management process
	STATE_ERROR,   // error state
} SystemState_t;

int main(void)
{
	int ret;

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	// variables
	SystemState_t state = STATE_IDLE;
	static bool previous_ecu_state = 0;
	static bool current_ecu_state = 0;
	uint32_t event_flags = 0;
	static uint32_t precharge_start_time = 0;

	// Initialize
	can_init();

	can_ivt_init();

	spi_adbms1818_hw_init();

	serial_monitor_init();

	battery_init();

	sdc_init();

	
	while (1)
	{		
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);


		//serial monitor daten senden
		serial_monitor((uint8_t *)&battery_values, sizeof(battery_values));

		// wait for event
		event_flags = k_event_wait(&error_to_main, EVT_ERROR_BIT,false, K_NO_WAIT);

		if (event_flags & EVT_ERROR_BIT) {
		state = STATE_ERROR;
		LOG_INF("got Error-Event! switching to Error-State.\n");
		}

		current_ecu_state = can_get_ecu_state();

		switch (state)
		{
		case STATE_TEST:
			//spi_wake_up();
			//spi_loopback();
			//serial_monitor((uint8_t*)(&battery_values), sizeof(battery_values));

			LOG_INF("state test lululala");
			
			break;

		case STATE_IDLE:
			LOG_INF("Waiting for ECU OK");

			// rising ECU edge detection
			if (previous_ecu_state != BATTERY_ON && current_ecu_state == BATTERY_ON)
			{
				state = STATE_PRECHARGE;
				LOG_INF("ECU rising edge, entering precharge state");
			}
			previous_ecu_state = current_ecu_state;
			break;

		case STATE_PRECHARGE:

			// set precharge start time
			if (precharge_start_time == 0)
			{
				precharge_start_time = k_uptime_get();
				LOG_INF("Precharge started");
			}

			// precharge logic done?
			if (battery_precharge_logic() == 0)
			{
				state = STATE_RUNNING;
				precharge_start_time = 0;
				LOG_INF("Precharge abgeschlossen, entering RUNNING state");
			}
			// check if precharge timeout has occurred
			else if ((k_uptime_get() - precharge_start_time) > PRECHARGE_TIMEOUT_MS)
			{
				state = STATE_ERROR;
				precharge_start_time = 0;
				sdc_shutdown();
				LOG_ERR("Precharge Timeout -> entering ERROR state");
			}
			// falling ECU edge detection
			else if (previous_ecu_state == BATTERY_ON && current_ecu_state != BATTERY_ON)
			{
				state = STATE_IDLE;
				precharge_start_time = 0;
				sdc_shutdown();
				LOG_INF("ECU Falling Edge, entering IDLE");
			}

			previous_ecu_state = current_ecu_state;
			break;

		case STATE_RUNNING:
			// rename old charging function
			battery_charging();
			LOG_INF("Battery management process running");

			// falling ECU edge detection
			if (previous_ecu_state == BATTERY_ON && current_ecu_state != BATTERY_ON)
			{
				state = STATE_IDLE;
				sdc_shutdown();
				LOG_INF("ECU falling edge, entering idle state");
			}
			previous_ecu_state = current_ecu_state;
			break;

		case STATE_ERROR:
			// set all relays to 0
			sdc_shutdown();
			LOG_INF("Error state");
			if (battery_get_error_code() == 0)
			{
				state = STATE_IDLE;
				k_event_clear(&error_to_main, EVT_ERROR_BIT);
				LOG_INF("Errors resolved, entering idle state");
			}
			break;
		default:
			LOG_ERR("Invalid State");
			return 0;
		}

	}
	return 0;
}

