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
 LOG_MODULE_REGISTER(serial_monitor, LOG_LEVEL_DBG);
 
 /* UART device definition (ensure usart2 is enabled in DTS) */
 static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));

 /* UART transmit done flag */
 static volatile bool uart_tx_done = true;

 static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
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
     case UART_RX_BUF_REQUEST:
     case UART_RX_BUF_RELEASED:
     default:
         break;
     }
 }
 
 void serial_monitor_init(void)
 {
     if (!device_is_ready(uart_dev)) {
         LOG_ERR("UART device not ready\n");
         return;
     }
     LOG_INF("UART device %s is ready\n", uart_dev->name);

     uart_callback_set(uart_dev, uart_callback, NULL);
 }
 
 void SerialMonitor(const uint8_t *data, uint16_t size)
 {
    static const uint8_t start[] = {0xFF, 0xA3};
    static const uint8_t stop[]  = {0xFF, 0xB3};

    if ((size + 4) > 520) {
        LOG_ERR("Data size too large");
        return;
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
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
}
 