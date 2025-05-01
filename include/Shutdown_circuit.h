/*
 * Shutdown_circuit.h
 *
 * Description: This module handles the Shutdown_circuit for the BMS system.
 *
 * Author: renolluc / grossfa2
 * Date: 20.04.2025
 *
 */

#ifndef INC_SHUTDOWN_CIRCUIT_H_
#define INC_SHUTDOWN_CIRCUIT_H_

#include "Battery_types.h"
#include "Status_error_flags.h"
#include "CAN_Bus.h"

#define IVT_TIMEOUT_MS 400
#define BATTERY_TIMEOUT_MS 400

#define GPIOA_DEVICE DT_NODELABEL(gpioa)

static const struct gpio_dt_spec sdc_in_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcin), gpios);
static const struct gpio_dt_spec sdc_out_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcout), gpios);
static const struct gpio_dt_spec drive_air_pos_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveairpositive), gpios);
static const struct gpio_dt_spec drive_air_neg_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveairnegative), gpios);
static const struct gpio_dt_spec drive_precharge_spec = GPIO_DT_SPEC_GET(DT_ALIAS(driveprecharge), gpios);
    
static uint64_t ivt_deadline_ms;


extern Battery_StatusTypeDef sdc_check_state(void);
extern int sdc_check_feedback(void);
extern int sdc_init(void);
#endif /* INC_SHUTDOWN_CIRCUIT_H_ */