/**
 * @file   serial_monitor.h
 * @brief  API-definition for the Serial Monitor module in the BMS-System.
 * @author renolluc / grossfa2
 * @date   20.04.2025
 */

#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

/* Zephyr-Framework includes */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
/* Project includes */

/** @brief UART device label for Zephyr device tree */
#define SERIAL_DEVICE DT_LABEL(DT_NODELABEL(usart2))

 /**
 * @brief Initialize the serial monitor UART interface
 *
 * This function checks if the UART device is ready and sets the UART callback handler.
 */
void serial_monitor_init(void);

/**
 * @brief Transmit data over UART with start/stop framing
 *
 * @param data Pointer to data payload
 * @param size Length of data payload in bytes
 */
void serial_monitor(const uint8_t *data, uint16_t size);

/**
 * @brief Generate a synthetic test frame simulating battery telemetry
 *
 * @param data Pointer to buffer to be filled
 * @param len Length of the buffer; must be at least 476 bytes
 */
void serial_generate_test_frame(uint8_t *data, size_t len);

#endif /* SERIAL_MONITOR_H */
