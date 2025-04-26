/* serial_monitor.h - Header for UART asynchronous transmit using Zephyr RTOS */

#ifndef SERIAL_MONITOR_H
#define SERIAL_MONITOR_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#define SERIAL_DEVICE DT_LABEL(DT_NODELABEL(usart2))

void serial_monitor_init(void);
void serial_monitor(const uint8_t *data, uint16_t size);
void serial_generate_test_frame(uint8_t *data, size_t len);

#endif /* SERIAL_MONITOR_H */
