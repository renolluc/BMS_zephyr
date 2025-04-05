#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <CAN_Bus.h>

#define TEST_CAN_MSG_ID 0x123
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

ZTEST_SUITE(can_bus_tests, NULL, NULL, NULL, NULL, NULL);
