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

#define SDC_IN_Pin GPIO_PIN_0
#define SDC_IN_GPIO_Port GPIOA
#define SDC_Out_Pin GPIO_PIN_8
#define SDC_Out_GPIO_Port GPIOA
//#define Drive_AIR_positive_Pin GPIO_PIN_4
//#define Drive_AIR_positive_GPIO_Port GPIOB
//#define Drive_AIR_negative_Pin GPIO_PIN_5
//#define Drive_AIR_negative_GPIO_Port GPIOB

#define CHARGE_ENABLE_PA9 9
#define CHARGER_CONNECTED_PA10 10
#define PRECHARGE_ENAABLE_PB7 7
#define DRIVE_PRECHARGE_PB6 6

static const struct gpio_dt_spec sdc_in_spec = 
    GPIO_DT_SPEC_GET(DT_ALIAS(sdcin), gpios);
static const struct gpio_dt_spec sdc_out_spec = 
    GPIO_DT_SPEC_GET(DT_ALIAS(sdcout), gpios);

extern Battery_StatusTypeDef refresh_sdc();
extern void sdc_set_relays(uint8_t CAN_Data);

#endif /* INC_SHUTDOWN_CIRCUIT_H_ */