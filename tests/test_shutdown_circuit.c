#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "shutdown_circuit.h"
#include "Battery.h"
#include "Status_error_flags.h"

/* Test suite for the shutdown circuit module */
ZTEST_SUITE(shutdown_circuit_tests, NULL, NULL, NULL, NULL, NULL);

//Function for mocking the GPIOs
static int mock_sdc_in_value;

/* 1. Provide a mock function under a different name */
static int mock_sdc_in_value;
int mock_gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
    return mock_sdc_in_value;
}

/* 2. Redirect all calls to gpio_pin_get_dt → our mock */
#define gpio_pin_get_dt mock_gpio_pin_get_dt

/* 1. Test: Initialization */
ZTEST(shutdown_circuit_tests, test_sdc_init)
{
    int ret = sdc_init();
    /* Expect 0 on success (GPIOs configured) */
    zassert_equal(ret, 0, "sdc_init failed with error code %d", ret);
}

/* 2. Test: Shutdown function */
ZTEST(shutdown_circuit_tests, test_sdc_shutdown)
{
    /* Clear any existing error flags */
    battery_values.error = 0;
    int ret = sdc_shutdown();
    zassert_equal(ret, 0, "sdc_shutdown returned %d", ret);

}

/* 3. Test: Feedback check (no falling edge) */
ZTEST(shutdown_circuit_tests, test_sdc_check_feedback_no_error)
{
    /* First call: prev_state initialized true; without mocking GPIO,
       we only verify that no error code is returned */
    int ret = sdc_check_feedback();
    zassert_equal(ret, 0, "sdc_check_feedback returned %d", ret);
}

/* 4. Test: State check under normal conditions */
ZTEST(shutdown_circuit_tests, test_sdc_check_state_ok)
{
    /* Ensure battery error is cleared, feedback is OK */
    battery_values.error = 0;
    int ret = sdc_check_state();
    zassert_equal(ret, 0, "sdc_check_state failed with error code %d", ret);
}

/* 5. Test: check if sdc_feedback detects a falling edge when triggered*/
ZTEST(shutdown_circuit_tests, test_sdc_check_feedback)
{
    int ret;

    /* Clear any existing battery error flags */
    battery_reset_error_flag(battery_values.error);

    /* 1) Prime prev_state: simulate feedback high */
    mock_sdc_in_value = 1;
    ret = sdc_check_feedback();
    zassert_equal(ret, 0, "Initial sdc_check_feedback() returned %d", ret);

    // For this feature you habe to mock the GPIO input
    /* 2) Simulate falling edge: feedback goes from 1 → 0 */
    // mock_sdc_in_value = 0;
    // ret = sdc_check_feedback();
    // zassert_equal(ret, -1, "sdc_check_feedback() on falling edge returned %d", ret);

    // /* After a falling edge, ERROR_SDC should be set */
    // zassert_true(battery_get_error_code() & ERROR_SDC,
    //              "ERROR_SDC flag not set after falling-edge feedback"); 
}
