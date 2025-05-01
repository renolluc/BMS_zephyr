/*
 * Battery.h
 *
 * Description: This module handles the Batteries for the BMS system.
 *
 * Author: renolluc / grossfa2
 * Date: 20.04.2025
 *
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "SPI_MB.h"
#include "CAN_Bus.h"
#include "Shutdown_circuit.h" 
#include "Battery_types.h"
#include <zephyr/drivers/gpio.h>
#include "Status_error_flags.h"
#include "zephyr/logging/log.h"
#include "error_handler.h"

/**
  * @brief  Battery Status
  */

#define MAX_VOLT 42000
#define MIN_VOLT 30000
#define MIN_TEMP 16725		// min temperature at 20°
#define MAX_TEMP 6115		// max temperature at 60°
#define AKKU_CAPACITANCE 45000		// in As ≙ 12.5 Ah


// struct for the battery
typedef struct {
    uint16_t totalVoltage;          // Total voltage of the accumulator system
    uint16_t highestCellVoltage;    // Highest voltage among individual cells
    uint16_t lowestCellVoltage;     // Lowest voltage among individual cells
    uint16_t meanCellVoltage;       // Mean voltage of all cells

    uint16_t highestCellTemp;
    uint16_t lowestCellTemp;
    uint16_t meanCellTemp;

    uint8_t status;     // Current status of the accumulator
    uint8_t error;		// current error flags of the accumulator

    int32_t actualCurrent;          // Actual current from IVT
    int32_t actualVoltage;          // Actual voltage from IVT 
    int32_t CurrentCounter;         // Current counter from IVT

    uint16_t time_per_measurement;
    uint16_t adbms_itemp;

    uint32_t balance_cells[NUM_OF_CLIENTS];

	uint16_t volt_buffer[18*NUM_OF_CLIENTS];
	uint16_t temp_buffer[10*NUM_OF_CLIENTS];
} BatterySystemTypeDef;

// extern variables
extern BatterySystemTypeDef battery_values;


static const struct gpio_dt_spec vfb_air_pos_spec = GPIO_DT_SPEC_GET(DT_ALIAS(vfbairpositive), gpios);
static const struct gpio_dt_spec vfb_air_neg_spec = GPIO_DT_SPEC_GET(DT_ALIAS(vfbairnegative), gpios);
static const struct gpio_dt_spec vfb_pc_relay_spec = GPIO_DT_SPEC_GET(DT_ALIAS(vfbpcrelay), gpios);
static const struct gpio_dt_spec charger_con_spec = GPIO_DT_SPEC_GET(DT_ALIAS(chargerconnect), gpios);

extern int battery_status_gpio_init(void);
extern Battery_StatusTypeDef battery_check_state(void);
void battery_monitor_thread(void);
extern void battery_stop_balancing(void);

#endif /* INC_BATTERY_H_ */

