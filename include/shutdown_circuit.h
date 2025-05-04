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
static const struct gpio_dt_spec sdc_in_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcin), gpios);
static const struct gpio_dt_spec sdc_out_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcout), gpios);
static const struct gpio_dt_spec drive_air_pos_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveairpositive), gpios);
static const struct gpio_dt_spec drive_air_neg_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveairnegative), gpios);
static const struct gpio_dt_spec drive_precharge_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveprecharge), gpios);

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