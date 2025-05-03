/*
 * Shutdown_circuit.c
 * 
 * Description: This module handles SPI communication for the BMS system.
 * 
 * Author: renolluc / grossfa2
 * Date: 22.03.2025
 * 
 */

#include "Shutdown_circuit.h"

LOG_MODULE_REGISTER(shutdown_circuit, LOG_LEVEL_ERR);

static const struct device *gpioa_dev;
static uint8_t  sdc_error_counter = 2;

/**
 * @brief Initializes the SDC output GPIO and internal timers.
 *
 * @return 0 on success, negative errno otherwise.
 */
int sdc_init(void)
{
    int ret;
 
    /* bind each port by its label */
    gpioa_dev  = DEVICE_DT_GET(GPIOA_DEVICE);
    

    #define CHECK_READY(spec)                                          \
    do {                                                               \
        if (!device_is_ready((spec).port)) {                           \
            LOG_ERR("GPIO port %s not ready",                          \
                    ((spec).port)->name);                              \
            return -ENODEV;                                            \
        }                                                              \
    } while (0)

    CHECK_READY(sdc_in_spec);
    CHECK_READY(sdc_out_spec);
    CHECK_READY(drive_air_pos_spec);
    CHECK_READY(drive_air_neg_spec);
    CHECK_READY(drive_precharge_spec);

    ret = gpio_pin_configure_dt(&sdc_in_spec, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&sdc_out_spec, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&drive_air_pos_spec, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&drive_air_neg_spec, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&drive_precharge_spec, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* Start-Deadline initial setzen */
    ivt_deadline_ms     = k_uptime_get() + IVT_TIMEOUT_MS;
    sdc_error_counter   = 0U;

    LOG_INF("SDC initialized, timeout %d ms", IVT_TIMEOUT_MS);
    printk("SDC initialized, timeout %d ms\n", IVT_TIMEOUT_MS);
    return 0;
}

/**
 * @brief Checks the IVT timeout condition and controls the SDC signal.
 *
 * - If more than IVT_TIMEOUT_MS has elapsed since the last IVT reset, 
 * ERROR_IVT is set and the deadline is readjusted.  
 * 
 * - If there are no critical errors (mask 0x47), SDC is set high,
 * the IVT timer is reset and the error counter is cleared.  
 * 
 * - In the event of errors, the counter is incremented; from 3 consecutive 
 * errors, SDC is set low and ERROR_SDC is triggered.  
 *
 * @return BATTERY_OK If SDC-Out high or error not yet latched or 
 * BATTERY_ERROR If SDC-Error latched (after ≥3 errors)
 */
Battery_StatusTypeDef sdc_check_state(void)
{
    int64_t now = k_uptime_get();
    bool curr_ecu_state;

    /*ECU Battery Ok*/
    //curr_ecu_state = can_get_ecu_state();
    //
    //
    //
    if (!curr_ecu_state)
    {
        battery_set_error_flag(ERROR_BATTERY);
        LOG_ERR("ECU not ready");
    }

    /* IVT-Timeout prüfen */
    if (now >= ivt_deadline_ms) {
        sdc_refresh_ivt_timer();
        battery_set_error_flag(ERROR_IVT);
        LOG_ERR("IVT timeout, flag ERROR_IVT set");
    }

    /* Prüfen, ob kritische Fehler vorliegen (Mask 0x47) */
    if ((battery_values.error & 0x47) == 0) {
        /* OK-Pfad: SDC high, Deadline & Fehler-Counter zurücksetzen */
        //gpio_pin_set_dt(&sdc_out_spec, 1);
        sdc_error_counter = 0U;
    }else{

        /* Fehler-Pfad: Counter inkrementieren, ggf. latchen */
        sdc_error_counter++;
        if (sdc_error_counter >= 3U) {
            //gpio_pin_set_dt(&sdc_out_spec, 0);
            battery_set_error_flag(ERROR_BATTERY);
            LOG_ERR("SDC latched low after %d errors", sdc_error_counter);
            return BATTERY_ERROR;
        }
}

    /* Noch nicht genug Fehler gesammelt, SDC high belassen */
    return BATTERY_OK;
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
 * @param unused Not used (kept for signature compatibility).
 */
int sdc_check_feedback(void)
{
    static bool prev_state = true;  /* assume pulled-up idle = high */
    bool curr_sdc_in_state;
    int  ret;

    //nochmals anschauen 
    //wieso braucht state 2 abfragen macht keinene sinn vergleich mit dem alten code 
    /* Read the feedback pin */
    curr_sdc_in_state = gpio_pin_get_dt(&sdc_in_spec);
    if (curr_sdc_in_state < 0) {
        gpio_pin_set_dt(&drive_air_pos_spec, 0);
        gpio_pin_set_dt(&drive_air_neg_spec, 0);
        gpio_pin_set_dt(&drive_precharge_spec,0);
        LOG_ERR("Failed to read SDC feedback pin (%d)", curr_sdc_in_state);
        return ;
    }
    /////////////////////////////////////////////////////////////////////////
    /* Falling edge: feedback went from 1 → 0 */
    if (!curr_sdc_in_state && prev_state) {
        /* De-energize AIR and precharge relays (drive outputs low) */
        gpio_pin_set_dt(&drive_air_pos_spec, 0);
        gpio_pin_set_dt(&drive_air_neg_spec, 0);
        gpio_pin_set_dt(&drive_precharge_spec,0);
        LOG_ERR("SDC feedback lost; relays turned off");
        return -1;
    }

    prev_state = curr_sdc_in_state;
    return 0;
}

/**
 * @brief Reset and update the Shutdown-Circuit output.
 *
 * This function replicates the original HAL-based SDC reset behavior using
 * Zephyr APIs. It performs the following steps:
 *   1. Reset the internal error counter to 2.  
 *   2. Reinitialize the ADBMS1818 over SPI.  
 *   3. Run the battery self-check.  
 *   4. Check for an IVT timeout using k_uptime_get() against ivt_deadline_ms,
 *      setting ERROR_IVT if expired.  
 *   5. If SPI init and battery check both succeeded and no critical errors
 *      (mask 0x47) are present, drive the SDC output pin high, restart the
 *      IVT deadline, and return BATTERY_OK. Otherwise, drive the SDC pin low
 *      and return BATTERY_ERROR.
 *
 * @return BATTERY_OK    if SDC remains active (high)
 * @return BATTERY_ERROR if SDC is latched low
 */
Battery_StatusTypeDef sdc_reset(void)
{
    int spi_ret;
    Battery_StatusTypeDef batt_ret;
    int64_t now = k_uptime_get();
    bool success;

    /* 1) Reset error counter */
    sdc_error_counter = 2;

    /* 2) SPI ADBMS1818 hardware init */
    spi_ret = spi_adbms1818_hw_init();
    if (spi_ret < 0) {
        LOG_ERR("SPI ADBMS init failed (%d)", spi_ret);
    }

    /* 3) Battery self-check */
    batt_ret = battery_check_state();
    if (batt_ret != BATTERY_OK) {
        LOG_ERR("Battery check failed (%d)", batt_ret);
    }

    /* 4) IVT timeout check */
    if (now >= ivt_deadline_ms) {
        battery_set_error_flag(ERROR_IVT);
    }

    /* Determine overall success */
    success = (spi_ret == 0) && (batt_ret == BATTERY_OK) && ((battery_values.error & 0x47) == 0);

    if (success) {
        /* SDC OK: drive high, restart IVT deadline */
        gpio_pin_set_dt(&sdc_out_spec, 1);
        return BATTERY_OK;
    } else {
        /* SDC error: drive low */
        gpio_pin_set_dt(&sdc_out_spec, 0);
        battery_set_error_flag(ERROR_SDC);
        return BATTERY_ERROR;
    }
}

/**
 * @brief Reset the IVT timer.
 *
 * This function resets the IVT timer to the current time plus the IVT_TIMEOUT_MS
 * 
 */
void sdc_refresh_ivt_timer(void)
{
    int64_t now = k_uptime_get();

    /* Reset the IVT deadline */
    ivt_deadline_ms = now + IVT_TIMEOUT_MS;
}
