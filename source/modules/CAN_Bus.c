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

#define CAN_MSG_ID 0x12345
#define SLEEP_TIME_MS 1000

//define loopback mode for testing
#define CONFIG_LOOPBACK_MODE

//Thread defines
#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
struct k_thread rx_thread_data;

//get CAN device
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

//define message queue
CAN_MSGQ_DEFINE(can_msgq, 2);

//Callback function for sending messages
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0)
	{
		printf("Callback! error-code: %d\nSender: %s\n",
			   error, sender);
	}
}

//Thread for receiving CAN messages
void rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = CAN_MSG_ID,
		.mask = CAN_EXT_ID_MASK
	};
	struct can_frame frame;
	int filter_id;

	//add filter to message queue and initialize queue
	filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
	printk("filter id: %d\n", filter_id);

	//while loop to receive messages
	while (1) {
	    if (k_msgq_get(&can_msgq, &frame, K_FOREVER) == 0) {
			//printf("Message received: %02X\n", frame.data[0]);
			printk("received CAN message: ID=0x%X, DLC=%d, Data=", frame.id, frame.dlc);
			for (int i = 0; i < frame.dlc; i++) {
				printk("%02X ", frame.data[i]);
			}
			printk("\n");
		} 
		else {
			printk("No message in queue\n");
		}
		//printf("watchdog\n");
		//k_msleep(SLEEP_TIME_MS); 
	}
}

//Function to send CAN messages
int send_can_message(const uint8_t *data)
{
	struct can_frame frame = {
		.flags = CAN_FRAME_IDE,
		.id = CAN_MSG_ID,
		.dlc = 8};

	memset(frame.data, 0, sizeof(frame.data)); // Clear all bytes in the frame data
	memcpy(frame.data, data, frame.dlc);      // Copy only the specified number of bytes

	
    // Debug: Print the data being sent
    printf("Sending CAN message: ID=0x%X, DLC=%d, Data=", frame.id, frame.dlc);
    for (int i = 0; i < 8; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");

	/* This sending call is none blocking. */
	if (can_send(can_dev, &frame, K_NO_WAIT, tx_irq_callback, "AMS-CB") == 0)
	{
		printf("CAN message sent\n");
	};
	return 0;
}

//Function to initialize CAN Bus
int BMS_CAN_INIT()
{
	int ret;

//loopback mode for routing messages directly to msgq
#ifdef CONFIG_LOOPBACK_MODE
	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	if (ret != 0)
	{
		printf("Error setting CAN mode [%d]", ret);
		return 0;
	}
#endif

	ret = can_start(can_dev);
	if (ret != 0)
	{
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

	// not needed anymore for thread implementation but leaving it for a while
	// k_thread_start(rx_thread);

	return 0;
}

