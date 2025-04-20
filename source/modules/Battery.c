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
 #include "Status_error_flags.h"
 
 #define GPIOA_DEVICE DT_NODELABEL(gpioa)
 #define GPIOB_DEVICE DT_NODELABEL(gpiob)

 static const struct device *gpioa_dev;
 static const struct device *gpiob_dev;
 

 /**
 * @brief Initialize the GPIO inputs for battery status pins.
 *
 * Must be called once during system init before reading status.
 *
 * @return 0 on success, negative errno otherwise.
 */
 int battery_status_gpio_init(void)
 {
     int ret;
 
     /* bind each port by its label */
     gpioa_dev  = DEVICE_DT_GET(GPIOA_DEVICE);
     gpiob_dev = DEVICE_DT_GET(GPIOB_DEVICE);
     

 
     if (!gpioa_dev || !gpiob_dev) {
         printk("ERROR: GPIO ports not found\n");
         return -ENODEV;
     }
 
     /* configure each pin as input */
     ret = gpio_pin_configure(gpioa_dev, V_FB_AIR_positive_Pin, GPIO_INPUT);
     if (ret) return ret;
 
     ret = gpio_pin_configure(gpioa_dev, V_FB_AIR_negative_Pin, GPIO_INPUT);
     if (ret) return ret;
 
     ret = gpio_pin_configure(gpioa_dev, V_FB_PC_Relay_Pin, GPIO_INPUT);
     if (ret) return ret;

     ret = gpio_pin_configure(gpiob_dev, Drive_AIR_positive_Pin, GPIO_OUTPUT);
     if (ret) return ret;
 
     ret = gpio_pin_configure(gpiob_dev, Drive_AIR_negative_Pin, GPIO_OUTPUT);
     if (ret) return ret;
 
     ret = gpio_pin_configure(gpiob_dev, Drive_Precharge_Relay_Pin, GPIO_OUTPUT);
     if (ret) return ret;
 
     return 0;
 }




 
 BatterySystemTypeDef battery_values;
 
 void battery_reset_error_flags(){
     battery_values.error = 0;
 }
 
 uint8_t battery_get_error_code(){
     return battery_values.error;
 }
 
 void battery_set_error_flag(uint8_t mask){
     battery_values.error |= mask;
 }
 
 void battery_set_reset_status_flag(uint8_t set, uint8_t mask){
     if(set){
         battery_values.status |= mask;
     }else{
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
    val = gpio_pin_get(gpioa_dev, V_FB_AIR_positive_Pin);
    battery_set_reset_status_flag(val > 0, STATUS_AIR_POSITIVE);

    /* STATUS_AIR_NEGATIVE: read v_fb_air_negative pin */
    val = gpio_pin_get(gpioa_dev, V_FB_AIR_negative_Pin);
    battery_set_reset_status_flag(val > 0, STATUS_AIR_NEGATIVE);

    /* STATUS_PRECHARGE: read v_fb_pc_relay pin */
    val = gpio_pin_get(gpioa_dev, V_FB_PC_Relay_Pin);
    battery_set_reset_status_flag(val > 0, STATUS_PRECHARGE);

    return battery_values.status;
}

  
 BatterySystemTypeDef* battery_calc_values(uint16_t *volt_data, uint16_t *temp_data){
     // get total, mean, min, max
     uint32_t total = 0;
     uint16_t min = 50000;
     uint16_t max = 0;
     for(uint16_t i = 0; i<(18*NUM_OF_CLIENTS); i++){
         if((i != 142) && (i != 143)){		// 2 zellen nicht bestückt
             total += volt_data[i];
             if(volt_data[i] < min){
                 min = volt_data[i];
             }
             if(volt_data[i] > max){
                 max = volt_data[i];
             }
         }
     }
     battery_values.meanCellVoltage = (uint16_t)(total / (18*NUM_OF_CLIENTS-2));		// 2 zellen nicht bestückt
     battery_values.totalVoltage = (uint16_t)(total /= 1000); 		// total voltage in 0.1V/bit
     battery_values.lowestCellVoltage = min;
     battery_values.highestCellVoltage = max;
 
     total = 0;
     min = 50000;
     max = 0;
     for(uint16_t i = 0; i<(8*NUM_OF_CLIENTS); i++){
         if((i != 1) && (i != 26)){		// 2 sensoren defekt
             total += temp_data[i];
             if(temp_data[i] < min){
                 min = temp_data[i];
             }
             if(temp_data[i] > max){
                 max = temp_data[i];
             }
         }
     }
     battery_values.meanCellTemp = (uint16_t)(total / (8*NUM_OF_CLIENTS-2));		// 2 sensoren defekt
     battery_values.highestCellTemp = min;
     battery_values.lowestCellTemp = max;
     return &battery_values;
 }
 
 uint8_t volt2celsius(uint16_t volt_100uV){		// convert volt to celsius with polynom
     if(volt_100uV > 23000){
         return 0;
     }else if(volt_100uV < 2000){
         return 100;
     }
     // Coefficients of the polynomial: a0, a1, ..., a10
     double coefficients[11] = {1.65728946e+02, -5.76649020e-02, 1.80075051e-05, -3.95278974e-09, 5.86752736e-13, -5.93033515e-17, 4.07565006e-21, -1.87118391e-25, 5.48516319e-30, -9.27411410e-35, 6.87565181e-40};
 
     // Calculate the polynomial value
     double result = coefficients[10];
     for (int8_t i = 9; i >= 0; i--) {
         result = result * volt_100uV + coefficients[i];
     }
     return (uint16_t)(result);		// in °C
 }
 
 
 
 Battery_StatusTypeDef check_battery(){
     HAL_StatusTypeDef status = Read_Voltages(battery_values.volt_buffer);
     status |= Read_Temp(battery_values.temp_buffer);
 
     if(status){
         set_battery_error_flag(ERROR_SPI|ERROR_BATTERY);
     }else{
         //User_LED_GPIO_Port->ODR ^= User_LED_Pin; // Toggle user LED if communication works
         calc_Battery_values(battery_values.volt_buffer, battery_values.temp_buffer);
         // check limits
         if((battery_values.highestCellVoltage > MAX_VOLT) || (battery_values.lowestCellVoltage < MIN_VOLT)){
             set_battery_error_flag(ERROR_VOLT|ERROR_BATTERY);
         }
         if((battery_values.highestCellTemp < MAX_TEMP) || (battery_values.lowestCellTemp > MIN_TEMP)){
             set_battery_error_flag(ERROR_TEMP|ERROR_BATTERY);
         }
     }
     return refresh_SDC();
 }
 

 
 void stop_balancing(){
     for(uint8_t i=0; i<NUM_OF_CLIENTS; i++){
         battery_values.balance_cells[i] = 0;
     }
     set_DCCx(battery_values.balance_cells);		// stop balancing
 }
 
 void balancing(){				// retrun if charger should be active
     uint16_t itemp = read_ADBMS_Temp();
     battery_values.adbms_itemp = itemp;
     if(itemp>=83){
         stop_balancing();
     }else{
         // do some balancing
         if(battery_values.highestCellVoltage >= 41000){		// start balancing at 4.10 V
             for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
                 battery_values.balance_cells[i] = 0;
                 for(uint8_t j=0; j<18; j++){
                     // get difference per cell to lowest cell
                     if((battery_values.volt_buffer[i*18 + j]-battery_values.lowestCellVoltage) > 100){
                         battery_values.balance_cells[i] |= 1<<j;
                     }
                 }
             }
             set_DCCx(battery_values.balance_cells);		// actuall balancing command
 
         }else{
             stop_balancing();
         }
     }
 }
 
 void battery_precharge_logic(){
     static uint32_t GPIOB_old = 0;
     static uint32_t cnt_100ms = 0;
     uint32_t GPIOB_Input = Precharge_EN_GPIO_Port->IDR;
     if ((GPIOB_Input & Precharge_EN_Pin)>(GPIOB_old & Precharge_EN_Pin)){ 			// rising edge pc_en
         set_relays(AIR_NEGATIVE | PRECHARGE_RELAY);
         cnt_100ms = 0;
     }else if(GPIOB_Input & Precharge_EN_Pin){										// pc_en high
         if(cnt_100ms > 55){
             set_relays(AIR_NEGATIVE | AIR_POSITIVE);
         }else if(cnt_100ms > 50){
             set_relays(AIR_NEGATIVE | AIR_POSITIVE | PRECHARGE_RELAY);
             cnt_100ms++;		// increment every 100ms
         }else{
             cnt_100ms++;		// increment every 100ms
         }
     }else if((GPIOB_Input & Precharge_EN_Pin)<(GPIOB_old & Precharge_EN_Pin)){		// falling edge pc_en
         set_relays(0);
     }
     GPIOB_old = GPIOB_Input;
 }
 
 void battery_charging(uint32_t input_data){
     // precharge logic
     precharge_logic();
 
     // charging logic
        if(!(input_data & Charger_Con_Pin)){		// charger connected
         if((battery_values.status&STATUS_CHARGING) == 0){
             battery_set_reset_status_flag(1, STATUS_CHARGING);
         }else{
             balancing();
         }
     }else{
         if((battery_values.status&STATUS_CHARGING) == STATUS_CHARGING){		// charger disconnected
             battery_set_reset_status_flag(0, STATUS_CHARGING);
             battery_values.adbms_itemp = 0;
             stop_balancing();
         }
     }
 }
 
 void battery_set_time_per_measurement(uint16_t time_ms){
     battery_values.time_per_measurement = time_ms;
 }