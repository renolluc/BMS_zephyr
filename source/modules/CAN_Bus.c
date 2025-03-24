/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <stdio.h>

 #include <zephyr/kernel.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/can.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/sys/byteorder.h>

 #include <CAN_Bus.h>

#define LED_MSG_ID 0x10
#define SET_LED 1
#define RESET_LED 0
#define COUNTER_MSG_ID 0x12345

uint8_t toggle = 1;
struct can_frame change_led_frame = {
    .flags = 0,
    .id = LED_MSG_ID,
    .dlc = 1
};
 
 const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

 CAN_MSGQ_DEFINE(counter_msgq, 2);

 void tx_irq_callback(const struct device *dev, int error, void *arg)
 {
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		printf("Callback! error-code: %d\nSender: %s\n",
		       error, sender);
	}
}

int BMS_CAN_INIT()
{
	int ret;
	uint8_t test; // Declare the variable 'test' with an appropriate type

    #ifdef CONFIG_LOOPBACK_MODE
	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printf("Error setting CAN mode [%d]", ret);
		return 0;
	}
    #endif

 ret = can_start(can_dev);
	if (ret != 0) {
		printf("Error starting CAN controller [%d]", ret);
		return 0;
	}
    printf("CAN Bus initialized\n");

	test = 0x01;
    uint8_t can_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;
		/* This sending call is none blocking. */
	if	(can_send(can_dev, &change_led_frame, K_FOREVER, tx_irq_callback, "test message") == 0){
        printf("CAN message sent\n");
    };

    const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = COUNTER_MSG_ID,
		.mask = CAN_EXT_ID_MASK
	};

	struct can_frame frame;
    int filter_id;

	filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
    printf("Filter ID: %d\n", filter_id);
    
    if (k_msgq_get(&counter_msgq, &frame, K_FOREVER) == 0){
        printf("Counter received: %u\n",
        sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
    };

	return 0;
}