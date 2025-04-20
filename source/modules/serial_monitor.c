/* serial_monitor.c - UART asynchronous transmit using Zephyr RTOS
 *
 * Author: schweja3 (adapted to Zephyr)
 */

 #include <zephyr/kernel.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/uart.h>
 #include <string.h>
 #include <zephyr/logging/log.h>

 /* LOG Module initialization */
 LOG_MODULE_REGISTER(serial_monitor, LOG_LEVEL_INF);
 
 /* UART device definition (ensure usart2 is enabled in DTS) */
 static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));
 
 static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
 {
     ARG_UNUSED(user_data);
 
     switch (evt->type) {
     case UART_TX_DONE:
         /* Transmission completed */
         break;
     case UART_TX_ABORTED:
         /* Transmission aborted, handle accordingly */
         break;
     case UART_RX_BUF_REQUEST:
     case UART_RX_BUF_RELEASED:
     default:
         break;
     }
 }
 
 void serial_monitor_init(void)
 {
     if (!device_is_ready(uart_dev)) {
         LOG_INF("UART device not ready\n");
         return;
     }
     LOG_INF("UART device %s is ready\n", uart_dev->name);

     uart_callback_set(uart_dev, uart_callback, NULL);
 }
 
 void SerialMonitor(const uint8_t *data, uint16_t size)
 {
     static uint8_t DMA_buffer[520];
     static const uint8_t start[] = {0xFF, 0xA3};
     static const uint8_t stop[] = {0xFF, 0xB3};
 
     if ((size + 4) > sizeof(DMA_buffer)) {
         printk("Data size too large\n");
         return;
     }
 
     DMA_buffer[0] = start[0];
     DMA_buffer[1] = start[1];
 
     memcpy(&DMA_buffer[2], data, size);
 
     DMA_buffer[size + 2] = stop[0];
     DMA_buffer[size + 3] = stop[1];
 
     uart_tx(uart_dev, DMA_buffer, size + 4, SYS_FOREVER_MS);
 }
 