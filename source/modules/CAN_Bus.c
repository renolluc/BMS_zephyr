/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file CAN_Bus.c
 * @brief CAN Bus communication module for Zephyr RTOS.
 *
 * This file implements functions to initialize and use the CAN interface,
 * send and receive messages, and process responses from IVT and ECU modules.
 * It includes a receiver thread, transmit callback, and utility functions
 * specific to an automotive battery management system.
 */

#include <CAN_Bus.h>
#include <zephyr/logging/log.h>

#define CAN_MSG_ID 0x123
#define SLEEP_TIME_MS 1000

// Define loopback mode for testing
//#define CONFIG_LOOPBACK_MODE

// Thread defines
#define RX_THREAD_STACK_SIZE 1024
#define RX_THREAD_PRIORITY 2
K_THREAD_STACK_DEFINE(can_rx_thread_stack, RX_THREAD_STACK_SIZE);
struct k_thread can_rx_thread_data;

// Get CAN device
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// struct for test feedback
struct k_sem test_ack_sem;

// Define message queue
CAN_MSGQ_DEFINE(can_msgq, 10);

// Callback function for sending messages
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
    char *sender = (char *)arg;

    ARG_UNUSED(dev);

    if (error != 0)
    {
        printk("Callback! error-code: %d\nSender: %s\n", error, sender);
    }
}

// Thread for receiving CAN messages
void can_rx_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    const struct can_filter filter = {
        .id = 0x000,
        .mask = 0x000};
    struct can_frame frame;
    int filter_id;

    // Add filter to message queue and initialize queue
    filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
    printk("filter id: %d\n", filter_id);

    // While loop to receive messages
    while (1)
    {
        if (k_msgq_get(&can_msgq, &frame, K_MSEC(5000)) == 0)
        {
            printk("Received CAN message: ID=0x%X, DLC=%d, Data=", frame.id, frame.dlc);
            for (int i = 0; i < frame.dlc; i++)
            {
                printk("%02X ", frame.data[i]);
            }
            printk("\n");
        }
        else
        {
            printk("No message in queue\n");
        }
        // Process received message
        if (frame.id == ADDR_ECU_RX)
        {
            //set_relays(frame.data[0]);
            if (frame.data[0] & BATTERY_ON)
            {
                //Turn the accumulator on
            }
            else if (frame.data[0 & BATTERY_OFF])
            {
                // Turn the accumulator off
            }
        }
        else if (frame.id == IVT_MSG_RESPONSE)
        {
            return;
        }
        // current measurement from the IVT
        else if (frame.id == IVT_MSG_RESULT_I)
        {   
            // refresh ivt timer
            sdc_refresh_ivt_timer();

            if (frame.data[0] == IVT_NCURRENT)
            {
                battery_values.actualCurrent = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
            }
        }
        // voltage measurement from the IVT
        else if (frame.id == IVT_MSG_RESULT_U1 || frame.id == IVT_MSG_RESULT_U2 || frame.id == IVT_MSG_RESULT_U3)
        {   
            // refresh ivt timer
            sdc_refresh_ivt_timer();

            if (frame.data[0] == IVT_NU1)
            {
                battery_values.actualVoltage = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
            }
            /* else if (frame.data[0] == IVT_NU2)
            {
                //battery_values.actualVoltages[1] = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
            }
            else if (frame.data[0] == IVT_NU3)
            {
                //battery_values.actualVoltages[2] = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
            } */
        }
        else if (frame.id == IVT_MSG_RESULT_T)
        {
            return;
        }
        else if (frame.id == IVT_MSG_RESULT_AS)
        {
            if (frame.data[0] == IVT_NQ)
            {
                battery_values.CurrentCounter = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
            }
        } 
        else if (frame.id == TEST_RXTHREAD_ID)
        {
            printk("Test message received. Giving semaphore.\n");
            k_sem_give(&test_ack_sem);  // Let the unit test know it was received
        }
    }
}

// Function to send CAN messages
int can_send_8bytes(uint32_t address, uint8_t *TxBuffer)
{
    struct can_frame frame = {
        .id = address,
        .dlc = 8,
    };

    memcpy(frame.data, TxBuffer, frame.dlc);

    // Debug: Print the data being sent
    printk("Sending CAN message: ID=0x%X, DLC=%d, Data=", frame.id, frame.dlc);
    for (int i = 0; i < frame.dlc; i++) {
        printk("%02X ", frame.data[i]);
    }
    printk("\n");

    // Send the CAN message (non-blocking)
    int ret = can_send(can_dev, &frame, K_NO_WAIT, tx_irq_callback, "AMS-CB");
    if (ret != 0) {
        printk("Error sending CAN message: %d\n", ret);
        return ret;
    }

    printk("CAN message sent successfully\n");
    return 0;
}

int can_send_ivt_nbytes(uint32_t address, uint8_t *TxBuffer, uint8_t length)
{
    struct can_frame frame = {
        .id = address,
        .dlc = length,
    };

    memcpy(frame.data, TxBuffer, frame.dlc);

    // Debug: Print the data being sent
    printk("Sending CAN message: ID=0x%X, DLC=%d, Data=", frame.id, frame.dlc);
    for (int i = 0; i < frame.dlc; i++) {
        printk("%02X ", frame.data[i]);
    }
    printk("\n");

    // Send the CAN message (non-blocking)
    int ret = can_send(can_dev, &frame, K_NO_WAIT, tx_irq_callback, "AMS-CB");
    if (ret != 0) {
        printk("Error sending CAN message: %d\n", ret);
        return ret;
    }

    printk("CAN message sent successfully\n");
    return 0;
}

int can_send_ecu(uint16_t GPIO_Input)
{
    uint8_t can_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_data[0] |= get_battery_status_code(GPIO_Input);
    can_data[1] |= get_battery_error_code();
    uint16_t total_volt = battery_values.totalVoltage;
    can_data[2] = total_volt & 0xFF;
    can_data[3] = total_volt >> 8;
    uint16_t actualCurrent = battery_values.actualCurrent;
    can_data[4] = (uint8_t)(actualCurrent / 1000);
    can_data[5] = volt2celsius(battery_values.highestCellTemp);
    if (battery_values.CurrentCounter > AKKU_CAPACITANCE) {
        can_data[6] = 0;
    } else {
        can_data[6] = 100 - (uint8_t)(battery_values.CurrentCounter / AKKU_CAPACITANCE);
    }
    return can_send_8bytes(ADDR_ECU_TX, can_data);
}


int can_ivt_init()
{
    int status = 0;
    uint8_t RxData[8];

    // Set sensor mode to STOP
    uint8_t can_data0[] = {SET_MODE, 0x00, 0x01, 0x00, 0x00};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_data0, 5);
    k_msleep(10);

    // Set current measurement to CYCLIC 100 Hz
    uint8_t can_data1[] = {(MUX_SETCONFIG | IVT_NCURRENT), CYCLIC, (CYCLETIME >> 8) & 0xFF, CYCLETIME & 0xFF};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_data1, 4);
    k_msleep(10);

    // Enable Voltage Measurement
    uint8_t can_data2[] = {(MUX_SETCONFIG | IVT_NU1), CYCLIC, (CYCLETIME >> 8) & 0xFF, CYCLETIME & 0xFF};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_data2, 4);
    k_msleep(10);
    uint8_t can_data3[] = {(MUX_SETCONFIG | IVT_NU2), CYCLIC, (CYCLETIME >> 8) & 0xFF, CYCLETIME & 0xFF};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_data3, 4);
    k_msleep(10);
    uint8_t can_data4[] = {(MUX_SETCONFIG | IVT_NU3), CYCLIC, (CYCLETIME >> 8) & 0xFF, CYCLETIME & 0xFF};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_data4, 4);
    k_msleep(10);

    // Set sensor mode to RUN
    k_msleep(100);
    uint8_t can_datan[] = {SET_MODE, 0x01, 0x01, 0x00, 0x00};
    status |= can_send_ivt_nbytes(IVT_MSG_COMMAND, can_datan, 5);
    k_msleep(10);

    return status;
}

// Function to initialize CAN Bus
int can_init()
{
    int ret;

    // Loopback mode for routing messages directly to msgq
#ifdef CONFIG_LOOPBACK_MODE
    ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
    if (ret != 0) {
        printk("Error setting CAN mode [%d]\n", ret);
        return ret;
    }
#endif

    // Ensure CAN device is ready
    if (!device_is_ready(can_dev))
    {
        printk("CAN device is not ready\n");
        return -ENODEV;
    }
    printk("CAN device is ready\n");
    
    ret = can_start(can_dev);
    if (ret != 0) {
        printk("Error starting CAN controller [%d]\n", ret);
        return ret;
    }
    printk("CAN Bus initialized\n");

    k_tid_t rx_tid = k_thread_create(&can_rx_thread_data, can_rx_thread_stack,
                                     K_THREAD_STACK_SIZEOF(can_rx_thread_stack),
                                     can_rx_thread, NULL, NULL, NULL,
                                     RX_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!rx_tid) {
        printk("ERROR spawning RX thread\n");
        return -1;
    }
    printk("RX thread spawned\n");

    // Initialize semaphore for test acknowledgment
    k_sem_init(&test_ack_sem, 0, 1);

    return 0;
}

