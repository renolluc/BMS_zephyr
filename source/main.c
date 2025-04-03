#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <SPI_MB.h>

LOG_MODULE_REGISTER(mainLog);

/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define THREAD_STACKSIZE 512
#define PRIORITY_SPI 7

void spiThread()
{
	while(1){
	//spi_test_physical_loopback();
	spi_wakeup_adbms1818();

	k_msleep(1000);
}
}

K_THREAD_DEFINE(spi_thread_id, THREAD_STACKSIZE, spiThread, NULL, NULL, NULL,
	PRIORITY_SPI, 0, 0);


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
