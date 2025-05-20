#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>

#define TEST_CAN_MSG_ID 0x123
#define TEST_RXTHREAD_ID 0x7A0
#define TEST_CAN_DATA_LEN 8

ZTEST(can_bus_tests, test_can_init)
{
    int ret = can_init();

    // 0 for initialization success, -16 for CAN Mode already set
    // this does not do what you think it does: `0 | -16` evaluates to -16 (| is binary or)
    zassert_equal(ret, 0 | -16, "can_init failed with error code %d", ret);
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

/* ZTEST(can_bus_tests, test_can_send_ecu)
{
    uint16_t gpio_input = 0x01; // Example GPIO input
    int ret = can_send_ecu(gpio_input);

    zassert_equal(ret, 0, "can_send_ecu failed with error code %d", ret);
} */

ZTEST(can_bus_tests, test_can_ivt_init)
{
    int ret = can_ivt_init();

    zassert_equal(ret, 0, "can_ivt_init failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_can_rx_thread)
{
    const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    zassert_true(device_is_ready(can_dev), "CAN device is not ready");

    struct can_frame test_frame = {
        .id = TEST_RXTHREAD_ID,
        .dlc = 8,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22}
    };

    printk("Sending test CAN message\n");
    int ret = can_send(can_dev, &test_frame, K_NO_WAIT, NULL, NULL);
    zassert_equal(ret, 0, "Failed to send test CAN message: %d", ret);

    printk("Waiting for message in can_thread\n");
    ret = k_sem_take(&test_ack_sem, K_MSEC(1000));
    zassert_equal(ret, 0, "Test message was not handled in time");
}

ZTEST_SUITE(can_bus_tests, NULL, can_init, NULL, NULL, NULL);
