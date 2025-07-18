/**
 * @file shutdown_circuit.c
 * @brief Implementation of the shutdown circuit module in the BMS system.
 * @author renolluc / grossfa2
 * @date 20.04.2025
 *
 * This module initializes and controls the GPIOs for the
 * shutdown relays, checks their status and reads feedback signals.
 * In addition, corresponding status flags are set in the event of errors.
 */

#include "shutdown_circuit.h"

/**
 * @brief Sets the name and logging levels for this module.
 *
 * Possible log levels:
 * - LOG_LEVEL_NONE
 * - LOG_LEVEL_ERR
 * - LOG_LEVEL_WRN
 * - LOG_LEVEL_INF
 * - LOG_LEVEL_DBG
 */
LOG_MODULE_REGISTER(shutdown_circuit, LOG_LEVEL_WRN);


/**
 * @brief Initializes the SDC GPIO and internal timers.
 *
 * @retval 0 on success, negative error code otherwise.
 */
int sdc_init(void)
{

#define CHECK_READY(spec)                     \
    do                                        \
    {                                         \
        if (!device_is_ready((spec).port))    \
        {                                     \
            LOG_ERR("GPIO port %s not ready", \
                    ((spec).port)->name);     \
            return -ENODEV;                   \
        }                                     \
    } while (0)

    CHECK_READY(sdc_in_spec);
    CHECK_READY(sdc_out_spec);
    CHECK_READY(drive_air_pos_spec);
    CHECK_READY(drive_air_neg_spec);
    CHECK_READY(drive_precharge_spec);

    sdc_shutdown();
    LOG_INF("SDC initialized, timeout %d ms", IVT_TIMEOUT_MS);
    return 0;
}

/**
 * @brief Checks Battery Errors and controls the SDC signal.
 *
 * - If there are no critical errors (mask 0x47), SDC is set high,
 * and the error counter is cleared.
 *
 * - In the event of errors, the counter is incremented; from 3 consecutive
 * errors, SDC is set low and ERROR_SDC is triggered.
 *
 * Must be called periodically
 *
 * @retval 0 If SDC-Out high or error not yet latched or
 * @retval <0 If SDC-Error latched (after ≥3 errors)
 */
int sdc_check_state(void)
{
    /* Checks for Battery Errors (Mask 0x47) */
    if (((battery_values.error & SDC_BATTERY_STATUS_ERROR_MASK) == 0))
    {
        /* OK-Tree: SDC high & reset errorcounter */
        //spi_adbms1818_hw_init();
        LOG_INF("SDC: OK, setting sdc high");
        gpio_pin_set_dt(&sdc_out_spec, 1);
        battery_reset_error_flag(ERROR_SDC);
    }
    else
    {
        LOG_WRN("SDC not OK");
        return -1;
    }
    return 0;
}

/**
 * @brief Monitor the SDC feedback line and de-energize relays on falling edge.
 *
 * This function reads the shutdown-circuit feedback input (active high) via Devicetree,
 * detects a falling edge, and then drives the AIR_NEGATIVE, AIR_POSITIVE and
 * PRECHARGE relays off (active-low drive) using Zephyr’s GPIO API.
 *
 * Must be called periodically
 *
 * @retval 0 If no falling edge detected, or
 * @retval -1 If a falling edge was detected, relays de-energized, and ERROR_SDC set.
 */
int sdc_check_feedback(void)
{
    static bool prev_state = false;
    bool curr_sdc_in_state;


    /* Read the SDC feedback line (active high) */
    curr_sdc_in_state = gpio_pin_get_dt(&sdc_in_spec);
    
    LOG_INF("prev_state: %d", prev_state);
    LOG_INF("curr_sdc_in_state: %d", curr_sdc_in_state);
    /* Falling edge: feedback went from 1 → 0 */
    if (!curr_sdc_in_state && prev_state)
    {
        /* De-energize AIR and precharge relays (drive outputs low) */
        LOG_ERR("Falling edge detected, de-energizing relays");
        sdc_shutdown();
        battery_set_error_flag(ERROR_SDC);
        prev_state = curr_sdc_in_state;
        return -1;
    }

    prev_state = curr_sdc_in_state;
    return 0;
}

/**
 * @brief De-energize the relays
 *
 * This function is called when the Relays should be de-energized.
 * It turns off the AIR_POSITIVE, AIR_NEGATIVE, and PRECHARGE relays
 *
 * @return 0 on success.
 */
int sdc_shutdown(void)
{
    gpio_pin_set_dt(&drive_air_pos_spec, 0);
    gpio_pin_set_dt(&drive_air_neg_spec, 0);
    gpio_pin_set_dt(&drive_precharge_spec, 0);
    LOG_ERR("entered shutdown");
    return 0;
}
