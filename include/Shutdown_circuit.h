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
#include <zephyr/devicetree/gpio.h>

#define IVT_TIMEOUT_MS 400
#define BATTERY_TIMEOUT_MS 400

#define GPIOA_DEVICE DT_NODELABEL(gpioa)

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
   
static uint64_t ivt_deadline_ms;


extern Battery_StatusTypeDef refresh_sdc();


#endif /* INC_SHUTDOWN_CIRCUIT_H_ */