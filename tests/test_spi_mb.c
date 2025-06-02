#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <spi_mb.h>

void spi_create_command(uint16_t cmd_in, uint8_t *cmd_out);

ZTEST(spi_tests, test_pec_example0)
{
    uint8_t data[] = {0x00, 0x01};
    uint16_t crc = spi_generate_pec(data, 2);
    zassert_equal(crc, 0x3D6E, "Expected 0x3D6E, got 0x%X", crc);
}


ZTEST(spi_tests, test_pec_example1)
{
    uint8_t data[] = {0xAB, 0xCD};
    uint16_t crc = spi_generate_pec(data, 2);
    zassert_equal(crc, 0x08EC2, "PEC mismatch! Expected: 0x08EC2, Got: 0x%X", crc);
}

ZTEST(spi_tests, test_pec_example2)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = spi_generate_pec(data, 4);
    zassert_equal(crc, 0xD354, "PEC mismatch! Expected: 0xD354, Got: 0x%X", crc);
}

ZTEST(spi_tests, test_create_command)
{
    uint16_t cmd_in = 0x1234;
    uint8_t cmd_out[4];

    spi_create_command(cmd_in, cmd_out);

    zassert_equal(cmd_out[0], 0x12, "MSB incorrect");
    zassert_equal(cmd_out[1], 0x34, "LSB incorrect");

    uint8_t crc_data[2] = {0x12, 0x34};
    uint16_t expected_crc = spi_generate_pec(crc_data, 2);
    uint16_t actual_crc = (cmd_out[2] << 8) | cmd_out[3];

    zassert_equal(expected_crc, actual_crc, "PEC mismatch in spi_create_command");
}


 __weak void spi_write_registergroup(uint16_t command, uint8_t *data)
{
    printk("Mock spi_write_registergroup called with command 0x%04X\n", command);
    return;
}

__weak void spi_send_command(uint16_t command)
{
    printk("Mock spi_send_command called with command 0x%04X\n", command);
    return;
}

__weak int spi_wakeup_adbms1818(void)
{
    printk("Mock spi_wakeup_adbms1818 called\n");
    return 0;
}

ZTEST(spi_tests, test_discharge_mapping_logic)
{
    uint32_t cells_to_balance[NUM_OF_CLIENTS] = {0x12345};
    int ret = spi_set_discharge_cell_x(cells_to_balance);
    zassert_equal(ret, 0, "spi_set_discharge_cell_x failed");
}

ZTEST_SUITE(spi_tests, NULL, NULL, NULL, NULL, NULL);