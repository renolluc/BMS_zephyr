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

#define SDC_IN_Pin 0
#define SDC_Out_Pin 8

#define IVT_TIMEOUT_MS 400
#define BATTERY_TIMEOUT_MS 400

#define GPIOA_DEVICE DT_NODELABEL(gpioa)

static const struct gpio_dt_spec sdc_in_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcin), gpios);
static const struct gpio_dt_spec sdc_out_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sdcout), gpios);
    
static uint64_t ivt_deadline_ms;
static uint64_t battery_deadline_ms;

extern Battery_StatusTypeDef refresh_sdc();
extern void sdc_set_relays(uint8_t CAN_Data);

#endif /* INC_SHUTDOWN_CIRCUIT_H_ */