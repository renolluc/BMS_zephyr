#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <SPI_MB.h>
#include <zephyr/sys/crc.h>



ZTEST(pec_tests, test_pec_example0)
{
    uint8_t data[] = {0x00, 0x01};
    uint16_t crc = spi_generate_pec(data, 2);
    zassert_equal(crc, 0x3D6E, "Expected 0x3D6E, got 0x%X", crc);
    //uint16_t crc = generatePEC(data, 2);
    //zassert_equal(crc, 0x3D6E, "PEC mismatch! Expected: 0x3D6E, Got: 0x%X", crc);
}


ZTEST(pec_tests, test_pec_example1)
{
    uint8_t data[] = {0xAB, 0xCD};
    uint16_t crc = spi_generate_pec(data, 2);
    //uint16_t crc = generatePEC(data, 2);
    zassert_equal(crc, 0x08EC2, "PEC mismatch! Expected: 0x08EC2, Got: 0x%X", crc);
}

ZTEST(pec_tests, test_pec_example2)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = spi_generate_pec(data, 4);
    //uint16_t crc = generatePEC(data, 4);
    zassert_equal(crc, 0xD354, "PEC mismatch! Expected: 0xD354, Got: 0x%X", crc);
}

ZTEST_SUITE(pec_tests, NULL, NULL, NULL, NULL, NULL);