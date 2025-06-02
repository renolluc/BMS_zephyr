#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <can_bus.h>

#define TEST_CAN_MSG_ID 0x123
#define TEST_RXTHREAD_ID 0x7A0
#define TEST_CAN_DATA_LEN 8

void *can_init_wrapper(void) {
    int ret = can_init();
    if (ret != 0) {
        printk("CAN init failed: %d\n", ret);
    }
    return NULL;  // Always return a void* (NULL is fine if no state needed)
}

ZTEST(can_bus_tests, test_can_send_8bytes)
{
    uint8_t test_data[TEST_CAN_DATA_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    int ret = can_send_8bytes(TEST_CAN_MSG_ID, test_data);

    zassert_equal(ret, 0, "can_send_8bytes failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_can_send_ivt_nbytes)
{
    uint8_t test_data[5] = {0x10, 0x20, 0x30, 0x40, 0x50};
    int ret = can_send_ivt_nbytes(TEST_CAN_MSG_ID, test_data, 5);

    zassert_equal(ret, 0, "can_send_ivt_nbytes failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_can_ivt_init)
{
    int ret = can_ivt_init();

    zassert_equal(ret, 0, "can_ivt_init failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_can_rx_thread)
{


    struct can_frame test_frame = {
        .id = TEST_RXTHREAD_ID,
        .dlc = 8,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22}
    };

    printk("Sending test CAN message\n");
    int ret = can_send_ivt_nbytes(TEST_RXTHREAD_ID, test_frame.data, test_frame.dlc);
    zassert_equal(ret, 0, "Failed to send test CAN message: %d", ret);

    printk("Waiting for message in can_thread\n");
    ret = k_sem_take(&test_ack_sem, K_MSEC(1000));
    zassert_equal(ret, 0, "Test message was not handled in time");
}

ZTEST(can_bus_tests, test_volt2celsius_boundaries)
{
    zassert_equal(can_volt2celsius(24000), 0, ">2.3V should return 0°C");
    zassert_equal(can_volt2celsius(1000), 100, "<0.2V should return 100°C");
    zassert_equal(can_volt2celsius(50000), 0, "Extreme high value should still return 0°C");
}

ZTEST(can_bus_tests, test_volt2celsius_polynomial)
{
    uint8_t temp = can_volt2celsius(10000);
    zassert_true(temp > 0 && temp < 100, "Temperature from polynomial out of range");
}

ZTEST_SUITE(can_bus_tests, NULL,  can_init_wrapper, NULL, NULL, NULL);
