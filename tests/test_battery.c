#include <zephyr/ztest.h>
#include "battery.h"
#include <stdint.h>

// declarations for internal functions
void battery_reset_error_flag(uint8_t mask);
void battery_set_reset_status_flag(uint8_t set, uint8_t mask);
BatterySystemTypeDef* battery_calc_values(const uint16_t *volt_data, const uint16_t *temp_data);
// Mocks
uint16_t mock_voltages[NUM_OF_CLIENTS * 18];
uint16_t mock_temperatures[NUM_OF_CLIENTS * 8];


extern BatterySystemTypeDef battery_values;

ZTEST(battery, test_reset_and_get_error_flag)
{
    battery_values.error = 0xFF; // Set all error flags
    battery_reset_error_flag(battery_values.error);
    zassert_equal(battery_get_error_code(), 0, "Error flags should be cleared");
}

ZTEST(battery, test_set_error_flag)
{
    battery_reset_error_flag(battery_values.error); // Clear first
    battery_set_error_flag(0x02);
    battery_set_error_flag(0x01);
    zassert_equal(battery_get_error_code(), 0x03, "Error flag setting failed");
}

ZTEST(battery, test_status_flag_set_clear)
{
    battery_values.status = 0;
    battery_set_reset_status_flag(1, 0x04);
    zassert_true(battery_values.status & 0x04, "Status flag not set");

    battery_set_reset_status_flag(0, 0x04);
    zassert_false(battery_values.status & 0x04, "Status flag not cleared");
}

ZTEST(battery, test_calc_values_basic)
{
    for (int i = 0; i < NUM_OF_CLIENTS * 18; i++) mock_voltages[i] = 4;
    for (int i = 0; i < NUM_OF_CLIENTS * 8; i++) mock_temperatures[i] = 70;

    BatterySystemTypeDef *val = battery_calc_values(mock_voltages, mock_temperatures);
    zassert_equal(val->meanCellVoltage, 4, "Mean voltage incorrect");
    zassert_equal(val->totalVoltage, (uint16_t)((4 * (NUM_OF_CLIENTS * 18 - 2)) / 1000), "Total voltage incorrect");
    zassert_equal(val->meanCellTemp, 70, "Mean temperature incorrect");
}

ZTEST(battery, test_volt2celsius_boundaries)
{
    zassert_equal(battery_volt2celsius(24000), 0, ">2.3V should return 0°C");
    zassert_equal(battery_volt2celsius(1000), 100, "<0.2V should return 100°C");
    zassert_equal(battery_volt2celsius(50000), 0, "Extreme high value should still return 0°C");
}

ZTEST(battery, test_volt2celsius_polynomial)
{
    uint8_t temp = battery_volt2celsius(10000);
    zassert_true(temp > 0 && temp < 100, "Temperature from polynomial out of range");
}

ZTEST_SUITE(battery, NULL, NULL, NULL, NULL, NULL);
