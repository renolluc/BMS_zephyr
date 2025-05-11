/**
 * @file serial_monitor.c
 * @brief UART asynchronous transmission interface using Zephyr RTOS
 *
 * This module provides UART communication capabilities for telemetry data,
 * including sending framed messages and generating synthetic data for testing.
 *
 * @author renolluc / grossfa2
 * @date 27.04.2025
 */

 #include "serial_monitor.h"

 /** @brief Logger module definition for serial monitor */
 LOG_MODULE_REGISTER(serial_monitor, LOG_LEVEL_INF);
 
 /** @brief UART device instance */
 static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));

 /** @brief Flag indicating if UART transmission is complete */
 static volatile bool uart_tx_done = true;

 /**
 * @brief UART callback function for handling UART events
 *
 * @param dev Pointer to UART device
 * @param evt UART event structure
 * @param user_data Unused
 */
 static void serial_callback(const struct device *dev, struct uart_event *evt, void *user_data)
 {
     ARG_UNUSED(user_data);
 
     switch (evt->type) {
     case UART_TX_DONE:
         /* Transmission completed */
         uart_tx_done = true;
         break;
     case UART_TX_ABORTED:
         /* Transmission aborted, handle accordingly */
         uart_tx_done = true;
         LOG_ERR("UART TX aborted\n");
         break;
     default:
         break;
     }
 }
 
 /**
 * @brief Initialize the serial monitor UART interface
 *
 * This function checks if the UART device is ready and sets the UART callback handler.
 */
 void serial_monitor_init(void)
 {
     if (!device_is_ready(uart_dev)) {
         LOG_ERR("UART device not ready\n");
         return;
     }
     LOG_INF("UART device %s is ready\n", uart_dev->name);

     uart_callback_set(uart_dev, serial_callback, NULL);
 }
 
 /**
 * @brief Transmit data over UART with start/stop framing
 *
 * @param data Pointer to data payload
 * @param size Length of data payload in bytes
 */
 /*void serial_monitor(const uint8_t *data, uint16_t size)
 {
    static const uint8_t start[] = {0xFF, 0xA3};
    static const uint8_t stop[]  = {0xFF, 0xB3};

    if ((size + 4) > 520) {
        LOG_ERR("Data size too large");
        return;
    }

    // Send start frame
    uart_poll_out(uart_dev, start[0]);
    uart_poll_out(uart_dev, start[1]);

    // Send payload
    for (int i = 0; i < size; i++) {
        uart_poll_out(uart_dev, data[i]);
    }

    // Send stop frame
    uart_poll_out(uart_dev, stop[0]);
    uart_poll_out(uart_dev, stop[1]);

    LOG_INF("UART poll TX done: %d bytes sent", size + 4);
}*/
void serial_monitor(const uint8_t *data, uint16_t size)
{
    static const uint8_t start[] = {0xFF, 0xA3};
    static const uint8_t stop[]  = {0xFF, 0xB3};

    // Create TX buffer (start + payload + stop)
    uint8_t buffer[520];
    if (size + 4 > sizeof(buffer)) {
        LOG_ERR("Payload too large");
        return;
    }

    buffer[0] = start[0];
    buffer[1] = start[1];
    memcpy(&buffer[2], data, size);
    buffer[2 + size] = stop[0];
    buffer[3 + size] = stop[1];

    size_t total = size + 4;
    size_t sent = 0;

    while (sent < total) {
        int bytes = uart_fifo_fill(uart_dev, &buffer[sent], total - sent);
        if (bytes > 0) {
            sent += bytes;
        } else {
            k_sleep(K_USEC(50)); // Wait briefly for FIFO space
        }
    }

    LOG_INF("Sent %zu bytes via FIFO fill", total);
}

/**
 * @brief Generate a synthetic test frame simulating battery telemetry
 *
 * @param data Pointer to buffer to be filled
 * @param len Length of the buffer; must be at least 476 bytes
 */
void serial_generate_test_frame(uint8_t *data, size_t len) {
    if (len < 476) return;

    // Clear full buffer
    memset(data, 0, len);

    // Fixed header values
    data[0] = 0xE8; data[1] = 0x03;                     // totalVoltage = 100.0 V
    data[2] = 0x10; data[3] = 0xA4;                     // highest = 4.2 V
    data[4] = 0x88; data[5] = 0x79;                     // lowest = 3.1 V
    data[6] = 0x3D; data[7] = 0x8E;                     // mean = 3.65 V
    data[8] = 0x40; data[9] = 0x1F;                     // highest cell temp
    data[10] = 0x40; data[11] = 0x1F;                   // lowest cell temp
    data[12] = 0x40; data[13] = 0x1F;                   // mean cell temp
    data[14] = 0x1F; data[15] = 0x02;                   // status and error
    data[16] = 0x10; data[17] = 0x00;                   // current low
    data[18] = 0x00; data[19] = 0x00;                   // current high
    data[20] = 0x00; data[21] = 0x00;                   // voltage low
    data[22] = 0x00; data[23] = 0x00;                   // voltage high
    data[24] = 0x20; data[25] = 0x00;                   // counter low
    data[26] = 0x00; data[27] = 0x00;                   // counter high
    data[28] = 0x64; data[29] = 0x00;                   // 100 ms
    data[30] = 0x55; data[31] = 0x00;                   // adbms temp

    // Fill 8x18 = 144 voltages starting at byte 28
    for (int i = 0; i < 8 * 18; i++) {
        float v = 3.0f + ((float)rand() / RAND_MAX) * 1.2f;  // 3.0 - 4.2 V
        uint16_t mv = (uint16_t)(v * 10000);                // e.g. 4.123 V → 41230
        data[28 + i * 2] = mv & 0xFF;
        data[28 + i * 2 + 1] = (mv >> 8) & 0xFF;
    }
    // Fill 8x3 = 24 temperatures right after voltage data
    size_t temp_offset = 28 + (8 * 18 * 2);
    for (int i = 0; i < 8 * 3; i++) {
        float temp = 20.0f + ((float)rand() / RAND_MAX) * 40.0f;  // 20°C to 60°C
        uint16_t ct = (uint16_t)(temp * 100);                    // 0.01 °C units
        data[temp_offset + i * 2] = ct & 0xFF;
        data[temp_offset + i * 2 + 1] = (ct >> 8) & 0xFF;
    }
}