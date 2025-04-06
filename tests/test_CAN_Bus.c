#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>

#define TEST_CAN_MSG_ID 0x12345
#define TEST_CAN_DATA_LEN 8

ZTEST(can_bus_tests, test_send_CAN)
{
    uint8_t test_data[TEST_CAN_DATA_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    int ret = send_CAN(TEST_CAN_MSG_ID, test_data);

    zassert_equal(ret, 0, "send_CAN failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_send_CAN_IVT_nbytes)
{
    uint8_t test_data[5] = {0x10, 0x20, 0x30, 0x40, 0x50};
    int ret = send_CAN_IVT_nbytes(TEST_CAN_MSG_ID, test_data, 5);

    zassert_equal(ret, 0, "send_CAN_IVT_nbytes failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_send_data2ECU)
{
    uint16_t gpio_input = 0x01; // Example GPIO input
    int ret = send_data2ECU(gpio_input);

    zassert_equal(ret, 0, "send_data2ECU failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_ISA_IVT_Init)
{
    int ret = ISA_IVT_Init();

    zassert_equal(ret, 0, "ISA_IVT_Init failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_BMS_CAN_INIT)
{
    int ret = BMS_CAN_INIT();

    zassert_equal(ret, 0, "BMS_CAN_INIT failed with error code %d", ret);
}

ZTEST(can_bus_tests, test_rx_thread)
{
    const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    zassert_true(device_is_ready(can_dev), "CAN device is not ready");

    struct can_frame test_frame = {
        .id = TEST_CAN_MSG_ID,
        .dlc = 6,
        .flags = CAN_FRAME_IDE,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
    };

    // Send a test CAN message
    int ret = can_send(can_dev, &test_frame, K_NO_WAIT, NULL, NULL);
    zassert_equal(ret, 0, "Failed to send test CAN message: %d", ret);

    // Wait for the message to be received by the rx_thread
    struct can_frame received_frame;
    ret = k_msgq_get(&can_msgq, &received_frame, K_MSEC(500)); // Increased timeout
    zassert_equal(ret, 0, "Failed to receive CAN message in rx_thread");

    // Verify the received message
    zassert_equal(received_frame.id, test_frame.id, "Message ID mismatch");
    zassert_equal(received_frame.dlc, test_frame.dlc, "Message DLC mismatch");
    for (int i = 0; i < test_frame.dlc; i++) {
        zassert_equal(received_frame.data[i], test_frame.data[i], "Data mismatch at byte %d", i);
    }
}

ZTEST_SUITE(can_bus_tests, NULL, NULL, NULL, NULL, NULL);
