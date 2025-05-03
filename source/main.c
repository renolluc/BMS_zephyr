#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <SPI_MB.h>
#include <serial_monitor.h>
#include <Battery.h>
#include <Shutdown_circuit.h>
#include <error_handler.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_ERR);
/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

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

/*error queue*/
K_MSGQ_DEFINE(err_evt_queue, sizeof(ErrorEvent), 4, 4);

int main(void)
{
	// variables
	ErrorEvent err_evt;
	SystemState_t state = STATE_RESET;

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

		if (k_msgq_get(&err_evt_queue, &err_evt, K_NO_WAIT) == 0)
		{
			// get in error state
			state = STATE_ERROR;
			//
			LOG_ERR("ERROR: %d", err_evt.code);
		}

		switch (state)
		{
		case STATE_RESET:
			if (sdc_reset() == BATTERY_OK)
			{
				state = STATE_IDLE;
			}
			break;

		case STATE_IDLE:
			//machen wir gar nichts

			// warten auf steigende Flanke ECU
				// wenn ECU OK dann set State to PROCESS

			break;
		
		case STATE_PRECHARGE:
			// precharge logic
			if (battery_precharge_logic() == 0)
			{
				state = STATE_RUNNING;
			}
			break;

		case STATE_RUNNING:
			// alte charging funktion noch umbenennen

			// warten auf ECU fallende Flanke
				// wenn ECU NOK dann set State to IDLE
			break;

		case STATE_ERROR:
			// alle relais null setzen

			// warten bis BMS ok
				//set State idle
			break;
		}

	}
	return 0;
}
