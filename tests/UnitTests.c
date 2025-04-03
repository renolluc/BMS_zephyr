#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include "SPI_MB.h"



ZTEST_SUITE(crc15_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(crc15_tests, test_pec_example1)
{
    uint8_t data[] = {0xAB, 0xCD};
    uint16_t crc = spi_generate_pec(data, 2);
    zassert_equal(crc, 0x1234, "PEC mismatch! Expected: 0x1234, Got: 0x%X", crc);
}

ZTEST(crc15_tests, test_pec_example2)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = spi_generate_pec(data, 4);
    zassert_equal(crc, 0x5678, "PEC mismatch! Expected: 0x5678, Got: 0x%X", crc);
}
