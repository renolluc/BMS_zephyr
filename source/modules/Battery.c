/*
 * Battery.c
 *
 * Description: This module handles the Batteries for the BMS system.
 *
 * Author: renolluc / grossfa2
 * Date: 20.04.2025
 *
 */

 #include "Battery.h"

  
 #define GPIOA_DEVICE DT_NODELABEL(gpioa)
 #define GPIOB_DEVICE DT_NODELABEL(gpiob)

 static const struct device *gpioa_dev;
 static const struct device *gpiob_dev;

 static uint64_t ivt_deadline_ms;
 struct k_event error_to_main;
 K_EVENT_DEFINE(error_to_main);

 BatterySystemTypeDef battery_values;
 LOG_MODULE_REGISTER(battery, LOG_LEVEL_ERR);

// Thread defines
#define BATTERY_MONITOR_STACK_SIZE 1024
#define BATTERY_MONITOR_THREAD_PRIORITY 8
K_THREAD_STACK_DEFINE(battery_monitor_thread_stack, BATTERY_MONITOR_STACK_SIZE);
struct k_thread battery_monitor_thread_data;


/**
 * @brief Clears all stored error flags in the battery system.
 *
 * This function resets the internal error bitfield to zero,
 * indicating that no error conditions are currently active.
 */
void battery_reset_error_flags(void)
{
    battery_values.error = 0;
}
 
/**
 * @brief Retrieves the current battery error code.
 *
 * This function returns the internal error bitfield, where each bit
 * represents a specific error condition in the battery management system.
 *
 * @return The 8‑bit error code (bitfield) from battery_values.error.
 */
uint8_t battery_get_error_code(void)
{
    return battery_values.error;
}

 /**
 * @brief Sets specified error flags in the battery system.
 *
 * This function OR‑combines the provided mask into the internal error bitfield,
 * enabling one or more error condition flags while preserving existing ones.
 *
 * @param mask Bitmask representing the error flags to set.
 */
void battery_set_error_flag(uint8_t mask)
{
    battery_values.error |= mask;
}

/**
 * @brief Sets or clears specified status flags in the battery system.
 *
 * This function modifies the internal status bitfield by either setting
 * or clearing the bits defined in `mask`. If `set` is non-zero, the bits
 * in `mask` are OR‑combined into `battery_values.status`; if `set` is zero,
 * those bits are cleared.
 *
 * @param set  Non-zero to set the flags, zero to clear them.
 * @param mask Bitmask indicating which status flags to modify.
 */
void battery_set_reset_status_flag(uint8_t set, uint8_t mask)
{
    if (set) {
        battery_values.status |= mask;
    } else {
        battery_values.status &= ~mask;
    }
}

/**
 * @brief Update and return the aggregated battery status flags.
 *
 * Reads the error state, ADBMS internal temperature, and the three GPIO status inputs
 * (AIR positive, AIR negative, Precharge) via Zephyr's GPIO API, and sets or clears
 * the corresponding bits in battery_values.status.
 *
 * @return The current battery status bitfield.
 */
uint8_t battery_get_status_code(void)
{
    bool ok;
    int val;

    /* STATUS_BATTERY_OK: no error bits set in lower 5 bits */
    ok = ((battery_values.error & 0x1F) == 0);
    battery_set_reset_status_flag(ok, STATUS_BATTERY_OK);

    /* STATUS_MB_TEMP_OK: internal temperature <= 85°C */
    ok = (battery_values.adbms_itemp <= 85);
    battery_set_reset_status_flag(ok, STATUS_MB_TEMP_OK);

    /* STATUS_AIR_POSITIVE: read v_fb_air_positive pin */
    val = gpio_pin_get_dt(&vfb_air_pos_spec);
    battery_set_reset_status_flag(val > 0, STATUS_AIR_POSITIVE);

    /* STATUS_AIR_NEGATIVE: read v_fb_air_negative pin */
    val = gpio_pin_get_dt(&vfb_air_neg_spec);
    battery_set_reset_status_flag(val > 0, STATUS_AIR_NEGATIVE);

    /* STATUS_PRECHARGE: read v_fb_pc_relay pin */
    val = gpio_pin_get_dt(&vfb_pc_relay_spec);
    battery_set_reset_status_flag(val > 0, STATUS_PRECHARGE);

    return battery_values.status;
}
 
 /**
 * @brief Computes aggregate battery metrics from raw ADC readings.
 *
 * This function processes arrays of raw cell voltages and temperatures to compute:
 *   - Total pack voltage (in 0.1 V units)
 *   - Mean cell voltage
 *   - Minimum and maximum cell voltages
 *   - Mean cell temperature
 *   - Minimum and maximum cell temperatures
 *
 * Two known-unpopulated cells (indices 142 and 143) are skipped in the voltage loop,
 * and two faulty temperature sensors (indices 1 and 26) are skipped in the temperature loop.
 *
 * @param volt_data  Pointer to an array of raw voltage readings. Must be at least
 *                   NUM_OF_CLIENTS * 18 elements long.
 * @param temp_data  Pointer to an array of raw temperature readings. Must be at least
 *                   NUM_OF_CLIENTS * 8 elements long.
 * @return Pointer to the global BatterySystemTypeDef instance (`battery_values`),
 *         updated with the computed metrics.
 */
BatterySystemTypeDef* battery_calc_values(const uint16_t *volt_data, const uint16_t *temp_data)
{
const size_t volts_per_client = NUM_OF_CLIENTS * 18;
const size_t temps_per_client = NUM_OF_CLIENTS * 8;
const uint16_t unpopulated_cells[] = {142, 143};
const size_t   n_unpopulated   = ARRAY_SIZE(unpopulated_cells);

uint32_t total = 0;
uint16_t min_v = UINT16_MAX;
uint16_t max_v = 0;

/* Voltage: accumulate and find min/max, skipping known-unpopulated cell indices */
for (size_t i = 0; i < volts_per_client; i++) {
bool skip = false;
for (size_t k = 0; k < n_unpopulated; k++) {
if (i == unpopulated_cells[k]) {
skip = true;
break;
}
}
if (skip) {
continue;
}
uint16_t v = volt_data[i];
total += v;
if (v < min_v) {
min_v = v;
}
if (v > max_v) {
max_v = v;
}
}

/* Compute mean and total voltage (convert raw sum to 0.1 V units) */
battery_values.meanCellVoltage = (uint16_t)(total / (volts_per_client - n_unpopulated));
battery_values.totalVoltage    = (uint16_t)(total / 1000U);
battery_values.lowestCellVoltage  = min_v;
battery_values.highestCellVoltage = max_v;

/* Temperature: repeat for temperature readings, skipping known-bad sensor indices */
total = 0;
min_v = UINT16_MAX;
max_v = 0;
const uint16_t bad_sensors[] = {1, 26};
const size_t n_bad = ARRAY_SIZE(bad_sensors);

for (size_t i = 0; i < temps_per_client; i++) {
bool skip = false;
for (size_t k = 0; k < n_bad; k++) {
if (i == bad_sensors[k]) {
skip = true;
break;
}
}
if (skip) {
continue;
}
uint16_t t = temp_data[i];
total += t;
if (t < min_v) {
min_v = t;
}
if (t > max_v) {
max_v = t;
}
}

battery_values.meanCellTemp    = (uint16_t)(total / (temps_per_client - n_bad));
battery_values.highestCellTemp = max_v;
battery_values.lowestCellTemp  = min_v;

return &battery_values;
}

/**
 * @brief Convert a raw voltage reading to temperature in degrees Celsius.
 *
 * This function applies a 10th‑order calibration polynomial to the input voltage
 * (expressed in units of 100 µV) to compute the corresponding temperature.
 *
 * Boundary conditions:
 *   - If the reading is above 23000 (i.e. > 2.3 V), returns 0 °C.
 *   - If the reading is below  2000 (i.e. < 0.2 V), returns 100 °C.
 *
 * @param volt_100uV Voltage reading in units of 100 µV.
 * @return Temperature in degrees Celsius as an 8‑bit unsigned integer.
 */
uint8_t battery_volt2celsius(uint16_t volt_100uV)
{
    /* Handle out‑of‑range inputs */
    if (volt_100uV > 23000U) {
        return 0U;
    } else if (volt_100uV < 2000U) {
        return 100U;
    }

    /* Calibration polynomial coefficients (a0..a10), single‑precision */
    static const float coefficients[11] = {
        1.65728946e+02f,
       -5.76649020e-02f,
        1.80075051e-05f,
       -3.95278974e-09f,
        5.86752736e-13f,
       -5.93033515e-17f,
        4.07565006e-21f,
       -1.87118391e-25f,
        5.48516319e-30f,
       -9.27411410e-35f,
        6.87565181e-40f
    };

    /* Evaluate polynomial via Horner's method */
    float result = coefficients[10];
    for (int i = 9; i >= 0; --i) {
        result = result * (float)volt_100uV + coefficients[i];
    }

    /* Cast to uint8_t for return (°C) */
    return (uint8_t)result;
}
 
/**
 * @brief Perform a single battery measurement cycle and update status.
 *
 * This function reads all cell voltages and temperatures, updates error flags
 * on SPI communication failures, computes aggregate metrics, checks for limit
 * violations, and finally refreshes the external SD‑Card interface.
 *
 * On Zephyr RTOS, all delays and error codes follow POSIX conventions; no HAL types
 * are used. The measurement functions return negative errno on failure, or 0 on success.
 *
 * @return Battery_StatusTypeDef  Current overall system status after SD‑Card refresh.
 */
Battery_StatusTypeDef battery_check_state(void)
{
    int err = 0;

    /* Read voltages into buffer */
    err |= spi_read_voltages(battery_values.volt_buffer);
    if (err < 0) {
        battery_set_error_flag(ERROR_SPI | ERROR_BATTERY);
    }

    /* Read temperatures into buffer */
    err |= spi_read_temp(battery_values.temp_buffer);
    if (err < 0) {
        battery_set_error_flag(ERROR_SPI | ERROR_BATTERY);
    }

    /* Compute aggregate values */
    battery_calc_values(battery_values.volt_buffer,
                        battery_values.temp_buffer);

    /* Check voltage limits */
    if (battery_values.highestCellVoltage > MAX_VOLT ||
        battery_values.lowestCellVoltage  < MIN_VOLT) {
        battery_set_error_flag(ERROR_VOLT | ERROR_BATTERY);
        err |= -1;
    }

    /* Check temperature limits */
    if (battery_values.highestCellTemp < MAX_TEMP ||
        battery_values.lowestCellTemp  > MIN_TEMP) {
        battery_set_error_flag(ERROR_TEMP | ERROR_BATTERY);
        err |= -1;
    }

    /* IVT-Timeout prüfen */
    int64_t now = k_uptime_get();

    if (now >= ivt_deadline_ms) {
        battery_refresh_ivt_timer();
        battery_set_error_flag(ERROR_IVT);
        LOG_ERR("IVT timeout, flag ERROR_IVT set");
        err |= -1;
    }

    return err;
}
  
 /**
 * @brief Ceases all cell balancing operations.
 *
 * This function clears the per‑cell balance mask for every ADBMS client,
 * then issues the updated discharge‑control command to stop all balancing.
 *
 */
void battery_stop_balancing(void)
{
    /* Clear all balance‑cell flags */
    for (uint8_t i = 0; i < NUM_OF_CLIENTS; i++) {
        battery_values.balance_cells[i] = 0;
    }

    /* Issue the updated discharge‑control command to halt balancing */
    int err = spi_set_discharge_cell_x(battery_values.balance_cells);
    if (err < 0) {
        /* Log an error if the SPI transfer failed */
        LOG_ERR("Failed to stop cell balancing (err %d)", err);
    }
}

/**
 * @brief Perform cell balancing based on voltage differentials and internal temperature.
 *
 * This function reads the internal die temperature via SPI. If the temperature exceeds
 * the safety threshold (83 °C), balancing is immediately stopped. Otherwise, if the highest
 * cell voltage is at or above 4.10 V (41000 in raw units), it computes a per‑cell discharge
 * mask: any cell whose voltage exceeds the lowest cell voltage by more than 100 units is
 * flagged for balancing. The resulting mask array is sent to the ADBMS devices to activate
 * cell‑discharge circuits. If the highest cell voltage is below the threshold, balancing is
 * also stopped.
 *
 * @note  
 * - Uses `spi_read_adbms_temp()` and `spi_set_discharge_cell_x()`, which return negative errno codes  
 *   on failure.  
 */
void balancing(void)
{
    int err;
    uint16_t die_temp;

    /* Read internal die temperature (°C × 10 or raw units depending on implementation) */
    die_temp = spi_read_adbms_temp();
    battery_values.adbms_itemp = die_temp;

    /* Safety check: stop balancing if temperature exceeds threshold */
    if (die_temp >= 83U) {
        battery_stop_balancing();
        return;
    }

    /* Only start balancing when pack voltage is high enough */
    if (battery_values.highestCellVoltage < 41000U) {
        battery_stop_balancing();
        return;
    }

    /* Compute discharge mask for each client based on voltage differential */
    for (uint16_t client = 0; client < NUM_OF_CLIENTS; ++client) {
        battery_values.balance_cells[client] = 0U;
        for (uint8_t cell = 0; cell < 18U; ++cell) {
            uint16_t v = battery_values.volt_buffer[client * 18U + cell];
            if ((v - battery_values.lowestCellVoltage) > 100U) {
                battery_values.balance_cells[client] |= (1U << cell);
            }
        }
    }

    /* Issue the balancing command via SPI */
    err = spi_set_discharge_cell_x(battery_values.balance_cells);
    if (err < 0) {
        LOG_ERR("Failed to set discharge mask (err %d)", err);
    }
}
 
 /**
 * @brief Handle precharge relay logic based on Precharge_EN input transitions.
 *
 * This function is called when the Precharge should be enabled
 * It drives the AIR and precharge relays to prepare for charging.
 * 
 */
int battery_precharge_logic(void)
{
    gpio_pin_set_dt(&drive_air_neg_spec, 1);
    gpio_pin_set_dt(&drive_precharge_spec, 1);

    if(battery_values.totalVoltage && battery_values.actualVoltage) {

        gpio_pin_set_dt(&drive_air_pos_spec, 1);
        gpio_pin_set_dt(&drive_air_neg_spec, 1);
        gpio_pin_set_dt(&drive_precharge_spec, 1);
        k_msleep(50);
        gpio_pin_set_dt(&drive_precharge_spec, 0);

        if((gpio_pin_get_dt(&vfb_pc_relay_spec)== 0) && 
           (gpio_pin_get_dt(&vfb_air_pos_spec)== 1) &&
           (gpio_pin_get_dt(&vfb_air_neg_spec)== 1) ) {
                return 0;
            }
        else {
            return -1;
        }
    }else{
        LOG_INF("Precharge not finished yet");
        return 0;
    }
}
 
/**
 * @brief Update charging state and invoke balancing or shutdown.
 *
 * This function must be called periodically (e.g., in the main loop or a timer thread).
 * It first runs the precharge logic, then reads the charger‐connected sense pin. 
 * 
 * Active‐low logic:
 *  - 0 => charger connected
 *  - 1 => charger disconnected
 *
 * If the charger has just connected, it sets the STATUS_CHARGING flag; if already
 * charging, it continues balancing. On disconnect, it clears STATUS_CHARGING,
 * resets the internal temperature, and stops balancing.
 */
void battery_charging(void)
{
    int  gpio_val;
    bool charger_connected;

    /* Run the precharge first */
    battery_precharge_logic();

    /* Read the charger‐connected sense pin (active low) */
    gpio_val = gpio_pin_get_dt(&charger_con_spec);
    if (gpio_val < 0) {
        LOG_ERR("Failed to read Charger_Con pin (err %d)", gpio_val);
        return;
    }
    charger_connected = (gpio_val == 0);

    /* Charging state logic */
    if (charger_connected) {
        /* Charger plugged in */
        if (!(battery_values.status & STATUS_CHARGING)) {
            battery_set_reset_status_flag(1, STATUS_CHARGING);
        } else {
            /* Continue balancing while charging */
            balancing();
        }
    } else {
        /* Charger has been unplugged */
        if (battery_values.status & STATUS_CHARGING) {
            battery_set_reset_status_flag(0, STATUS_CHARGING);
            battery_values.adbms_itemp = 0;
            battery_stop_balancing();
        }
    }
}

/**
 * @brief Set the measurement interval for the battery monitoring cycle.
 *
 * This function updates the global `battery_values.time_per_measurement` field
 * to specify how long (in milliseconds) has elapsed between successive
 * measurements of voltages and temperatures.
 *
 * @param time_ms Measurement interval in milliseconds.
 */
void battery_set_time_per_measurement(uint16_t time_ms)
{
    battery_values.time_per_measurement = time_ms;
}

/**
 * @brief Reset the IVT timer.
 *
 * This function resets the IVT timer to the current time plus the IVT_TIMEOUT_MS
 * 
 */
void battery_refresh_ivt_timer(void)
{
    int64_t now = k_uptime_get();

    /* Reset the IVT deadline */
    ivt_deadline_ms = now + IVT_TIMEOUT_MS;
}

void battery_monitor_thread()
{
    LOG_INF("Battery Monitor Thread started\n");
    int state = 0;
    int err_counter = 0;

    while (1)
    {
        // reset flags every cycle
        battery_reset_error_flags();

        state |= battery_check_state();
        state |= sdc_check_state();
        state |= sdc_check_feedback();

        if (state < 0)
        {
            err_counter++;
        }
        else if (err_counter >= 3)
        {
            k_event_post(&error_to_main, EVT_ERROR_BIT);
        }
        else if (state == 0)
        {
            // no error
            err_counter = 0;
        }

        k_msleep(1000);
    }
}

/**
 * @brief Initialize the GPIO inputs for battery status pins.
 *
 * Must be called once during system init before reading status.
 *
 * @return 0 on success, negative errno otherwise.
 */
int battery_init(void)
{
    int ret;

    /* bind each port by its label */
    gpioa_dev = DEVICE_DT_GET(GPIOA_DEVICE);
    gpiob_dev = DEVICE_DT_GET(GPIOB_DEVICE);

#define CHECK_READY(spec)                     \
    do                                        \
    {                                         \
        if (!device_is_ready((spec).port))    \
        {                                     \
            LOG_ERR("GPIO port %s not ready", \
                    ((spec).port)->name);     \
            return -ENODEV;                   \
        }                                     \
    } while (0)

    CHECK_READY(vfb_air_pos_spec);
    CHECK_READY(vfb_air_neg_spec);
    CHECK_READY(vfb_pc_relay_spec);
    CHECK_READY(charger_con_spec);

    /* configure each pin as input */
    ret = gpio_pin_configure_dt(&vfb_air_pos_spec, GPIO_INPUT);
    if (ret)
        return ret;

    ret = gpio_pin_configure_dt(&vfb_air_neg_spec, GPIO_INPUT);
    if (ret)
        return ret;

    ret = gpio_pin_configure_dt(&vfb_pc_relay_spec, GPIO_INPUT);
    if (ret)
        return ret;

    ret = gpio_pin_configure_dt(&charger_con_spec, GPIO_INPUT);
    if (ret)
        return ret;

    LOG_INF("BMS GPIOs initialized");
    printk("Battery GPIOs initialized\n");

    // Open Thread
    k_tid_t battery_monitor_thread_id = k_thread_create(&battery_monitor_thread_data, battery_monitor_thread_stack,
                                                        K_THREAD_STACK_SIZEOF(battery_monitor_thread_stack),
                                                        battery_monitor_thread, NULL, NULL, NULL,
                                                        BATTERY_MONITOR_THREAD_PRIORITY, 0, K_NO_WAIT);

    if (!battery_monitor_thread_id)
    {
        LOG_ERR("ERROR spawning Battery Monitoring thread\n");
        return -1;
    }
    else
    {
        LOG_INF("Battery Monitoring thread spawned\n");
        LOG_INF("Battery Succesfully Initialized\n");
        return 0;
    }
}