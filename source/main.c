#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <SPI_MB.h>

/* LED configuration */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   10000

#include <stdlib.h>

void generate_test_frame(uint8_t *data, size_t len) {
    if (len < 476) return;

    // Clear full buffer
    memset(data, 0, len);

    // Fixed header values
    data[0] = 0xE8; data[1] = 0x03;                     // totalVoltage = 100.0 V
    data[2] = 0x10; data[3] = 0xA4;                     // highest = 4.2 V
    data[4] = 0x88; data[5] = 0x79;                     // lowest = 3.1 V
    data[6] = 0x3D; data[7] = 0x8E;                     // mean = 3.65 V
    data[8] = 0x40; data[9] = 0x1F;                     // temp1 dummy
    data[10] = 0x40; data[11] = 0x1F;                   // temp2 dummy
    data[12] = 0x40; data[13] = 0x1F;                   // temp3 dummy
    data[14] = 0x1F; data[15] = 0x02;                   // status/error
    data[16] = 0x10; data[17] = 0x00;                   // current low
    data[18] = 0x00; data[19] = 0x00;                   // current high
    data[20] = 0x20; data[21] = 0x00;                   // counter low
    data[22] = 0x00; data[23] = 0x00;                   // counter high
    data[24] = 0x64; data[25] = 0x00;                   // 100 ms
    data[26] = 0x55; data[27] = 0x00;                   // adbms temp

    // Fill 8x18 = 144 voltages starting at byte 28
    for (int i = 0; i < 8 * 18; i++) {
        float v = 3.0f + ((float)rand() / RAND_MAX) * 1.2f;  // 3.0 - 4.2 V
        uint16_t mv = (uint16_t)(v * 10000);                // e.g. 4.123 V → 41230
        data[28 + i * 2] = mv & 0xFF;
        data[28 + i * 2 + 1] = (mv >> 8) & 0xFF;
    }
}


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

	// Initialize CAN Bus
	BMS_CAN_INIT();

	spi_adbms1818_hw_init();

	serial_monitor_init();
	uint8_t test_data[476];

	while (1) {
 		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printk("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
		
		printk("Starting SerialMonitor test...\n");
		generate_test_frame(test_data, sizeof(test_data));
		SerialMonitor(test_data, sizeof(test_data));
		printk("SerialMonitor test complete.\n");	

	}

	return 0;
}
