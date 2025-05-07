/**
 * @file   shutdown_circuit.h
 * @brief  API-definition for the shutdown-circuit-module in the BMS-System.
 * @author renolluc / grossfa2
 * @date   20.04.2025
 */

 #ifndef INC_SHUTDOWN_CIRCUIT_H_
 #define INC_SHUTDOWN_CIRCUIT_H_

/* Zephyr-Framework Includes */
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
/* Project Includes */
#include "Status_error_flags.h"
#include "Battery.h"

/** @brief Device-Label for GPIO-Port A */
#define GPIOA_DEVICE DT_NODELABEL(gpioa)

/** @brief Mask for Battery-Errorbits (0x47) */
#define SDC_BATTERY_STATUS_ERROR_MASK 0x47

/* External GPIO specifications */
static const struct gpio_dt_spec sdc_in_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(sdc_in))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(sdc_in), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(sdc_in), 0),
};

static const struct gpio_dt_spec sdc_out_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(sdc_out))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(sdc_out), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(sdc_out), 0),
};

static const struct gpio_dt_spec drive_air_pos_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(drive_air_pos))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(drive_air_pos), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(drive_air_pos), 0),
};

static const struct gpio_dt_spec drive_air_neg_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(drive_air_neg))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(drive_air_neg), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(drive_air_neg), 0),
};

static const struct gpio_dt_spec drive_precharge_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(drive_precharge))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(drive_precharge), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(drive_precharge), 0),
};

/**
 * @brief      Checks if the sdc is in a valid state.
 * @retval 0   Running without errors
 * @retval <0  Error when there is an error in he battery system
 */
int sdc_check_state(void);

/**
 * @brief      Checks the feedback line of the Shutdown-Circuit.
 * @retval 0   Feedback line is high
 * @retval <0  Error when feedback line is low
 */

int sdc_check_feedback(void);

/**
 * @brief      Initializes the Shutdown-Circuit and the GPIOs.
 * @retval 0   Initialization successful
 * @retval <0  Error during initialization
 */

int sdc_init(void);

/**
 * @brief      De-energizes all relays
 * @retval 0   Relais erfolgreich abgeschaltet
 */
int sdc_shutdown(void);

#endif /* INC_SHUTDOWN_CIRCUIT_H_ */