/**
 * @file   Battery.h
 * @brief  API-definition for the battery-module in the BMS-System.
 * @author renolluc / grossfa2
 * @date   20.04.2025
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

/* Zephyr-Framework includes */
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
/* Project includes */
#include "SPI_MB.h"
#include "shutdown_circuit.h" 
#include "Status_error_flags.h"


/** @brief Maximum allowable cell voltage in mV (42000 = 4.2V) */
#define MAX_VOLT 42000
#define MAX_VOLT_TEST 19000
/** @brief Minimum allowable cell voltage in mV (30000 = 3.0V) */
#define MIN_VOLT 30000
#define MIN_VOLT_TEST 9000
/** @brief Minimum raw temperature ADC value (≈ 20 °C) */
#define MIN_TEMP 16725
/** @brief Maximum raw temperature ADC value (≈ 60 °C) */
#define MAX_TEMP 6115
/** @brief Accumulator capacity in As (45000 = 12.5 Ah) */
#define AKKU_CAPACITANCE 45000
/** @brief Timeout value for IVT response in milliseconds */
#define IVT_TIMEOUT_MS 400


/** @brief Event bitmask for reporting battery errors */
#define EVT_ERROR_BIT (1 << 0)


/** @brief Event signaling errors to the main control loop */
extern struct k_event error_to_main;


/**
 * @brief Battery system state representation
 *
 * Contains all relevant data points for status monitoring,
 * diagnostic calculation and communication with other modules.
 */
typedef struct {
  uint16_t totalVoltage;          // Total voltage of the accumulator system
  uint16_t highestCellVoltage;    // Highest voltage among individual cells
  uint16_t lowestCellVoltage;     // Lowest voltage among individual cells
  uint16_t meanCellVoltage;       // Mean voltage of all cells

  uint16_t highestCellTemp;       // Highest temperature among individual cells
  uint16_t lowestCellTemp;        // Lowest temperature among individual cells
  uint16_t meanCellTemp;          // Mean temperature of all cells

  uint8_t status;                 // Current status of the accumulator
  uint8_t error;		              // current error flags of the accumulator

  int32_t actualCurrent;          // Actual current from IVT
  int32_t actualVoltage;          // Actual voltage from IVT 
  int32_t CurrentCounter;         // Current counter (Charge Counter) from IVT (SOC estimation)

  uint16_t time_per_measurement;  // sets the time between two measurements
  uint16_t adbms_itemp;           // Internal temperature of the ADBMS chip

  uint32_t balance_cells[NUM_OF_CLIENTS]; // Cell balancing mask for each client

	uint16_t volt_buffer[18*NUM_OF_CLIENTS]; // Buffer for raw voltage readings
	uint16_t temp_buffer[10*NUM_OF_CLIENTS]; // Buffer for raw temperature readings
} BatterySystemTypeDef;

/* --- GPIO Bindings (from Devicetree) --- */

/** @brief GPIO input: AIR+ feedback signal */
static const struct gpio_dt_spec vfb_air_pos_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_air_pos))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_air_pos), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_air_pos), 0),
};


/** @brief GPIO input: AIR− feedback signal */
static const struct gpio_dt_spec vfb_air_neg_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_air_neg))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_air_neg), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_air_neg), 0),
};

/** @brief GPIO input: Precharge relay feedback signal */
static const struct gpio_dt_spec vfb_pc_relay_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(vfb_pc_relay))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(vfb_pc_relay), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(vfb_pc_relay), 0),
};

/** @brief GPIO input: Charger connection signal */
static const struct gpio_dt_spec charger_con_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(charger_con))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(charger_con), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(charger_con), 0),
};

/** @brief GPIO output: Charger enabled signal */
static const struct gpio_dt_spec charge_en_spec = {
  .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(charge_en))),
  .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(charge_en), 0),
  .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(charge_en), 0),
};

/* --- Battery API --- */


/** @brief Global instance containing the latest battery values */
extern BatterySystemTypeDef battery_values;

/**
 * @brief Initializes GPIOs related to battery feedback and charger sensing.
 * @return 0 on success, negative errno code otherwise.
 */
extern int battery_init(void);

/**
 * @brief Retrieves the current status bitfield from the battery system.
 * @return Battery status bitfield (8-bit)
 */
extern uint8_t battery_get_status_code(void);

/**
 * @brief Retrieves the current error bitfield from the battery system.
 * @return Battery error code (8-bit)
 */
extern uint8_t battery_get_error_code(void);

/**
 * @brief Converts raw voltage (in 100 µV units) to temperature in °C.
 * @param volt_100uV Voltage in units of 100 µV
 * @return Temperature in degrees Celsius
 */
extern uint8_t battery_volt2celsius(uint16_t volt_100uV);

/**
 * @brief Sets error flags in the battery error field.
 * @param Bitmask (8-bit) representing error flags to set
 */
extern void battery_set_error_flag(uint8_t mask);

/**
 * @brief Resets all battery error flags to zero.
 */
extern void battery_reset_error_flag(uint8_t mask);

/**
 * @brief Checks for system errors and updates the battery error state accordingly.
 * @return 0 if OK, <0 if errors are detected
 */
extern int battery_check_state(void);

/**
 * @brief Stops all active cell balancing operations.
 */
extern void battery_stop_balancing(void);

/**
 * @brief Handles the charging logic including balance management.
 */
extern void battery_charging(void);

/**
 * @brief Refreshes the IVT (current sensor) watchdog timer.
 */
extern void battery_refresh_ivt_timer(void);

/**
 * @brief Executes precharge logic by managing AIR +/- and precharge relays.
 * @return 0 on successful precharge, <0 if feedback is incorrect
 */
extern int battery_precharge_logic(void);


#endif /* INC_BATTERY_H_ */