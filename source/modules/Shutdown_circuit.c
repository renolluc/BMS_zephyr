/*
 * Shutdown_circuit.c
 * 
 * Description: This module handles SPI communication for the BMS system.
 * 
 * Author: renolluc / grossfa2
 * Date: 22.03.2025
 * 
 */

#include "Shutdown_circuit.h"
#include "Battery.h"
#include "Status_error_flags.h"
#include "CAN_Bus.h"

static uint8_t error_counter = 2;

Battery_StatusTypeDef refresh_SDC(){
    if(TIM16->CNT >= IVT_TIMEOUT){
        TIM16->CNT = IVT_TIMEOUT;
        set_battery_error_flag(ERROR_IVT);
    }
    if ((battery_values.error&0x47) == 0){
        // SDC OK
        SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
        // reset tim7 timeout counter
        TIM7->CNT = 0;
        error_counter = 0;
    }else{
        // SDC error
        error_counter++;
        if(error_counter >= 3){
            SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
            set_battery_error_flag(ERROR_SDC);
            //set_relays(0);								// open AIR relais
            return BATTERY_ERROR;
        }
    }
    return BATTERY_OK;
}

void check_SDC_Feedback(uint32_t input_data){
    static uint32_t old_data = 0;
    if ((input_data & SDC_IN_Pin) < (old_data & SDC_IN_Pin)){					// open relais when global SDC falling
        Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin<<16;		// low
        Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin<<16;		// low
        Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin<<16;	// low
    }
    old_data = input_data;
}

Battery_StatusTypeDef SDC_reset(){
    error_counter = 2;
    HAL_StatusTypeDef status_hw;
    status_hw = ADBMS_HW_Init();
    status_hw |= check_battery();
    if(TIM16->CNT >= IVT_TIMEOUT){
        TIM16->CNT = IVT_TIMEOUT;
        status_hw |= HAL_ERROR;
    }
    // SDC on / off
    if(status_hw == HAL_OK){
        TIM7->CNT = 0;		// reset tim7 timeout counter
        SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
        return BATTERY_OK;
    }else{
        SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
        return BATTERY_ERROR;
    }
}

 // IN CAN FUNKTION UMDISPONIEREN
 void set_relays(uint8_t CAN_Data){
    static uint64_t last_value = 0;
    if(last_value != CAN_Data){
        if(CAN_Data & AIR_POSITIVE){
            Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin;	// high
        }else{
            Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin<<16;	// low
        }
        if(CAN_Data & AIR_NEGATIVE){
            Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin;	// high
        }else{
            Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin<<16;	// low
        }
        if(CAN_Data & PRECHARGE_RELAY){
            Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin;	// high
        }else{
            Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin<<16;	// low
        }
    }
    last_value = CAN_Data;
}