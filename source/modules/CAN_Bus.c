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
LOG_MODULE_REGISTER(can, LOG_LEVEL_WRN);

#define CAN_MSG_ID 0x123
#define SLEEP_TIME_MS 1000

// Define loopback mode for testing
#ifdef CONFIG_ZTEST
#define CONFIG_LOOPBACK_MODE
#endif

// Thread defines
#define RX_THREAD_STACK_SIZE 1024
#define RX_THREAD_PRIORITY -6
K_THREAD_STACK_DEFINE(can_rx_thread_stack, RX_THREAD_STACK_SIZE);
struct k_thread can_rx_thread_data;

// Get CAN device
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// struct for test feedback
struct k_sem test_ack_sem;

// Define message queue
CAN_MSGQ_DEFINE(can_msgq, 10);

// private variables set by the ECU
static int ecu_ok_flag = 0;

// Callback function for sending messages
void tx_irq_callback(const struct device *dev, int error, void *arg)
{
    char *sender = (char *)arg;

    ARG_UNUSED(dev);

    if (error != 0)
    {
        LOG_ERR("Callback! error-code: %d\nSender: %s\n", error, sender);
    }
}

// Thread for receiving CAN messages
void can_thread(void *arg1, void *arg2, void *arg3)
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
    LOG_DBG("filter id: %d\n", filter_id);

    // While loop to receive messages
    while (1)
    {
        // send data to ECU
        can_send_ecu();

        if (k_msgq_get(&can_msgq, &frame, K_MSEC(100)) == 0)
        {
            LOG_INF("Received CAN message: ID=0x%X, DLC=%d", frame.id, frame.dlc);
            LOG_HEXDUMP_DBG(frame.data, frame.dlc, "Data");

            // Process received message
            if (frame.id == ADDR_ECU_RX)
            {
                // set_relays(frame.data[0]);
                if (frame.data[0] == BATTERY_ON)
                {
                    // Set ecu_ok_flag high
                    ecu_ok_flag = 1;
                }
                else if (frame.data[0] == BATTERY_OFF)
                {
                    // Set ecu_ok_flag low
                    ecu_ok_flag = 0;
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
                battery_refresh_ivt_timer();

                if (frame.data[0] == IVT_NCURRENT)
                {
                    battery_values.actualCurrent = frame.data[5] | frame.data[4] << 8 | frame.data[3] << 16 | frame.data[2] << 24;
                }
            }
            // voltage measurement from the IVT
            else if (frame.id == IVT_MSG_RESULT_U1 || frame.id == IVT_MSG_RESULT_U2 || frame.id == IVT_MSG_RESULT_U3)
            {
                // refresh ivt timer
                battery_refresh_ivt_timer();

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
                LOG_DBG("Test message received. Giving semaphore.\n");
                k_sem_give(&test_ack_sem); // Let the unit test know it was received
            }
        }
        else
        {
            LOG_DBG("No message in queue\n");
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
    LOG_INF("Sending CAN message: ID=0x%X, DLC=%d", frame.id, frame.dlc);
    LOG_HEXDUMP_DBG(frame.data, frame.dlc, "Data");

    // Send the CAN message (non-blocking)
    int ret = can_send(can_dev, &frame, K_NO_WAIT, tx_irq_callback, "AMS-CB");
    if (ret != 0)
    {
        LOG_ERR("Error sending CAN message: %d\n", ret);
        return ret;
    }

    LOG_DBG("CAN message sent successfully\n");
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
    LOG_INF("Sending CAN message: ID=0x%X, DLC=%d", frame.id, frame.dlc);
    LOG_HEXDUMP_DBG(frame.data, frame.dlc, "Data");

    // Send the CAN message (non-blocking)
    int ret = can_send(can_dev, &frame, K_NO_WAIT, tx_irq_callback, "AMS-CB");
    if (ret != 0)
    {
        LOG_ERR("Error sending CAN message: %d\n", ret);
        return ret;
    }

    LOG_DBG("CAN message sent successfully\n");
    return 0;
}

int can_send_ecu(void)
{
    uint8_t can_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_data[0] |= battery_get_status_code();
    can_data[1] |= battery_get_error_code();
    uint16_t total_volt = battery_values.totalVoltage;
    can_data[2] = total_volt & 0xFF;
    can_data[3] = total_volt >> 8;
    uint16_t actualCurrent = battery_values.actualCurrent;
    can_data[4] = (uint8_t)(actualCurrent / 1000);
    can_data[5] = battery_volt2celsius(battery_values.highestCellTemp);
    if (battery_values.CurrentCounter > AKKU_CAPACITANCE)
    {
        can_data[6] = 0;
    }
    else
    {
        can_data[6] = 100 - (uint8_t)(battery_values.CurrentCounter / AKKU_CAPACITANCE);
    }
    return can_send_8bytes(ADDR_ECU_TX, can_data);
}

int can_ivt_init()
{
    int status = 0;

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

int can_get_ecu_state()
{
    return ecu_ok_flag;
}

// Function to initialize CAN Bus
int can_init()
{
    int ret;

    // Loopback mode for routing messages directly to msgq
#ifdef CONFIG_LOOPBACK_MODE
    ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
    if (ret != 0)
    {
        LOG_ERR("Error setting CAN mode [%d]\n", ret);
        return ret;
    }
#endif

    // Ensure CAN device is ready
    if (!device_is_ready(can_dev))
    {
        LOG_ERR("CAN device is not ready\n");
        return -ENODEV;
    }
    LOG_INF("CAN device is ready\n");

    ret = can_start(can_dev);
    if (ret != 0)
    {
        LOG_ERR("Error starting CAN controller [%d]\n", ret);
        return ret;
    }
    LOG_INF("CAN Bus initialized\n");

    k_tid_t rx_tid = k_thread_create(&can_rx_thread_data, can_rx_thread_stack,
                                     K_THREAD_STACK_SIZEOF(can_rx_thread_stack),
                                     can_thread, NULL, NULL, NULL,
                                     RX_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!rx_tid)
    {
        LOG_ERR("ERROR spawning RX thread\n");
        return -1;
    }
    LOG_INF("RX thread spawned\n");

    // Initialize semaphore for test acknowledgment
    k_sem_init(&test_ack_sem, 0, 1);

    return 0;
}