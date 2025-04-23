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

#define GPIOA_DEVICE DT_NODELABEL(gpioa)

static const struct device *gpioa_dev;
static uint8_t error_counter = 2;


#define IVT_TIMEOUT_MS   400
static struct k_timer ivt_timer;

static int64_t ivt_deadline_ms;
static uint8_t  sdc_error_counter;

/**
 * @brief Initialisiert das SDC-Output-GPIO und interne Zeitgeber.
 *
 * Diese Funktion wird via SYS_INIT() automatisch vor main() aufgerufen.
 *
 * @param dev Nicht verwendet (SYS_INIT-Parameter).
 * @return 0 bei Erfolg, negativer errno sonst.
 */
static int sdc_init(void)
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

    ret = gpio_pin_configure_dt(&sdc_in_spec, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&sdc_out_spec, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* Start-Deadline initial setzen */
    ivt_deadline_ms     = k_uptime_get() + IVT_TIMEOUT_MS;
    sdc_error_counter   = 0U;

    LOG_INF("SDC initialized, timeout %d ms", IVT_TIMEOUT_MS);
    printk("SDC initialized, timeout %d ms\n", IVT_TIMEOUT_MS);
    return 0;
}

/**
 * @brief Überprüft die IVT-Timeout-Bedingung und steuert das SDC-Signal.
 *
 * - Wenn seit letzter IVT-Reset länger als IVT_TIMEOUT_MS vergangen ist,  
 *   wird ERROR_IVT gesetzt und die Deadline neu justiert.  
 * - Liegen keine kritischen Fehler (Mask 0x47) vor, wird SDC high gesetzt,
 *   der IVT-Timer zurückgesetzt und der Fehler-Counter gelöscht.  
 * - Bei Fehlern wird der Counter inkrementiert; ab 3 aufeinanderfolgenden  
 *   Fehlern wird SDC low gesetzt und ERROR_SDC ausgelöst.  
 *
 * @return BATTERY_OK   Wenn SDC-Out high oder Fehler noch nicht latched  
 * @return BATTERY_ERROR Wenn SDC-Error latched (nach ≥3 Fehlern)
 */
Battery_StatusTypeDef refresh_sdc(void)
{
    int64_t now = k_uptime_get();

    /* IVT-Timeout prüfen */
    if (now >= ivt_deadline_ms) {
        ivt_deadline_ms = now + IVT_TIMEOUT_MS;
        battery_set_error_flag(ERROR_IVT);
        LOG_DBG("IVT timeout, flag ERROR_IVT set");
    }

    /* Prüfen, ob kritische Fehler vorliegen (Mask 0x47) */
    if ((battery_values.error & 0x47) == 0) {
        /* OK-Pfad: SDC high, Deadline & Fehler-Counter zurücksetzen */
        gpio_pin_set(gpioa_dev, SDC_Out_Pin, 1);
        ivt_deadline_ms   = now + IVT_TIMEOUT_MS;
        sdc_error_counter = 0U;
        return BATTERY_OK;
    }else{

        /* Fehler-Pfad: Counter inkrementieren, ggf. latchen */
        sdc_error_counter++;
        if (sdc_error_counter >= 3U) {
            gpio_pin_set(gpioa_dev, SDC_Out_Pin, 0);
            battery_set_error_flag(ERROR_SDC);
            LOG_INF("SDC latched low after %d errors", sdc_error_counter);
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
void check_sdc_feedback(void)
{
    static bool prev_state = true;  /* assume pulled-up idle = high */
    bool curr_state;
    int  ret;

    /* Read the feedback pin */
    curr_state = gpio_pin_get_dt(&sdc_in_spec);
    if (curr_state < 0) {
        LOG_ERR("Failed to read SDC feedback pin (%d)", curr_state);
        return;
    }

    /* Falling edge: feedback went from 1 → 0 */
    if (!curr_state && prev_state) {
        /* De-energize AIR and precharge relays (drive outputs low) */
        gpio_pin_set_dt(&drive_air_pos_spec, 0);
        gpio_pin_set_dt(&drive_air_neg_spec, 0);
        gpio_pin_set_dt(&drive_precharge_spec,0);
        LOG_INF("SDC feedback lost; relays turned off");
    }

    prev_state = curr_state;
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
    error_counter = 2;

    /* 2) SPI ADBMS1818 hardware init */
    spi_ret = spi_adbms1818_hw_init();
    if (spi_ret < 0) {
        LOG_ERR("SPI ADBMS init failed (%d)", spi_ret);
    }

    /* 3) Battery self-check */
    batt_ret = check_battery();
    if (batt_ret != BATTERY_OK) {
        LOG_ERR("Battery check failed (%d)", batt_ret);
    }

    /* 4) IVT timeout check */
    if (now >= ivt_deadline_ms) {
        battery_set_error_flag(ERROR_IVT);
    }

    /* Determine overall success */
    success = (spi_ret == 0) &&(batt_ret == BATTERY_OK) && ((battery_values.error & 0x47) == 0);

    if (success) {
        /* SDC OK: drive high, restart IVT deadline */
        ivt_deadline_ms = now + IVT_TIMEOUT;
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
 * @brief Drive the AIR and precharge relays based on a CAN-data bitmask.
 *
 * This function updates the GPIO outputs for the AIR_POSITIVE, AIR_NEGATIVE,
 * and PRECHARGE relays to match the bits in \p can_data. Only when the mask
 * changes since the last call are the pins actually toggled.
 *
 * @param can_data   Bitmask containing one or more of  
 *                   - AIR_POSITIVE  
 *                   - AIR_NEGATIVE  
 *                   - PRECHARGE_RELAY  
 */
void sdc_set_relays(uint8_t can_data)
{
    static uint8_t last_value = 0;

    /* Only update on change */
    if (last_value == can_data) {
        return;
    }

    /* AIR positive relay */
    if(can_data & AIR_POSITIVE) {
        gpio_pin_set_dt(&drive_air_pos_spec, 1);
    } else {
        gpio_pin_set_dt(&drive_air_pos_spec, 0);
    }

    /* AIR negative relay */
    if(can_data & AIR_NEGATIVE) {
        gpio_pin_set_dt(&drive_air_neg_spec, 1);
    } else {
        gpio_pin_set_dt(&drive_air_neg_spec, 0);
    }

    /* Precharge relay */
    if(can_data & PRECHARGE_RELAY) {
        gpio_pin_set_dt(&drive_precharge_spec, 1);
    } else {
        gpio_pin_set_dt(&drive_precharge_spec, 0);
    }
    
    last_value = can_data;
}