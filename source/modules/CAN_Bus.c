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
#define COUNTER_MSG_ID 0x12345

//Thread defines
#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
struct k_thread rx_thread_data;

 
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

void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = COUNTER_MSG_ID,
		.mask = CAN_EXT_ID_MASK
	};
	struct can_frame frame;
	int filter_id;

	//filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
	//printk("Counter filter id: %d\n", filter_id);

	while (1) {
		if (k_msgq_get(&counter_msgq, &frame, K_FOREVER) == 0) {;
			printf("Message received: %u\n",
			       sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
		}
		if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) && (frame.flags & CAN_FRAME_RTR) != 0U) {
			continue;
		}

		if (frame.dlc != 2U) {
			printf("Wrong data length: %u\n", frame.dlc);
			continue;
		}
		printf("watchdog");
	}
}

int send_can_message(const uint8_t *data)
{
	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = COUNTER_MSG_ID,
		.mask = CAN_EXT_ID_MASK
	};

	struct can_frame frame = {
		.flags = 0,
		.id = LED_MSG_ID,
		.dlc = 1
	};
	uint8_t toggle = 1;
	frame.data[0] = toggle++ & 0x01 ? 1 : 0;
		/* This sending call is none blocking. */
		if	(can_send(can_dev, &frame, K_FOREVER, tx_irq_callback, "test message") == 0){
			printf("CAN message sent\n");
		};
		return 0;
}

int BMS_CAN_INIT()
{
	int ret;

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

	k_tid_t rx_tid;

	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
							 K_THREAD_STACK_SIZEOF(rx_thread_stack),
							 rx_thread, NULL, NULL, NULL,
							 RX_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!rx_tid)
	{
		printf("ERROR spawning rx thread\n");
	}
	printf("RX thread spawned\n");

	return 0;
}