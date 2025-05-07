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

/* Zephyr-Framework includes */
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
/* Project includes */
#include "SPI_MB.h"
#include "CAN_Bus.h"
#include "shutdown_circuit.h" 
#include "Status_error_flags.h"


/**
  * @brief  Battery Status
  */

#define MAX_VOLT 42000
#define MIN_VOLT 30000
#define MIN_TEMP 16725		// min temperature at 20°
#define MAX_TEMP 6115		// max temperature at 60°
#define AKKU_CAPACITANCE 45000		// in As ≙ 12.5 Ah
#define IVT_TIMEOUT_MS 400 //

 //define error event
 #define EVT_ERROR_BIT (1 << 0)
 extern struct k_event error_to_main;

// struct for the battery
typedef struct {
    uint16_t totalVoltage;          // Total voltage of the accumulator system
    uint16_t highestCellVoltage;    // Highest voltage among individual cells
    uint16_t lowestCellVoltage;     // Lowest voltage among individual cells
    uint16_t meanCellVoltage;       // Mean voltage of all cells

    uint16_t highestCellTemp;       // Highest temperature among individual cells
    uint16_t lowestCellTemp;      // Lowest temperature among individual cells
    uint16_t meanCellTemp;      // Mean temperature of all cells

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


static const struct gpio_dt_spec vfb_air_pos_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_air_pos))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_air_pos), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_air_pos), 0),
};

static const struct gpio_dt_spec vfb_air_neg_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_air_neg))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_air_neg), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_air_neg), 0),
};

static const struct gpio_dt_spec vfb_pc_relay_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_pc_relay))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_pc_relay), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_pc_relay), 0),
};

static const struct gpio_dt_spec charger_con_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(charger_con))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(charger_con), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(charger_con), 0),
};

static const struct gpio_dt_spec charge_en_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(charge_en))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(charge_en), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(charge_en), 0),
};


extern uint8_t battery_get_status_code(void);
extern uint8_t battery_get_error_code(void);
extern uint8_t battery_volt2celsius(uint16_t volt_100uV);
extern void battery_set_error_flag(uint8_t mask);
extern void battery_reset_error_flags(void);
extern int battery_init(void);
extern int battery_check_state(void);
extern void battery_stop_balancing(void);
extern void battery_charging(void);
extern void battery_refresh_ivt_timer(void);
extern int battery_precharge_logic(void);


#endif /* INC_BATTERY_H_ */

