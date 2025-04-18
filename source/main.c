#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <SPI_MB.h>


/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

int main(void)
{
	
	int ret; 
	bool led_state = true;	

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

    /*
	//Initialisierung des CAN Bus
	ret	= can_init();
	if (ret < 0) {
		printk("Can Bus Init failed\n");
		return 0;
	}
    */

	spi_adbms1818_hw_init();
	
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printk("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
