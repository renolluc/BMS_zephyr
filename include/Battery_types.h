#ifndef BATTERY_TYPES_H
#define BATTERY_TYPES_H

/** @brief Status-Codes für das BMS */
typedef enum {
    BATTERY_OK        = 0x00,
    BATTERY_ERROR     = 0x01,
    BATTERY_TEMP_ERROR= 0x03,
    BATTERY_VOLT_ERROR= 0x05,
    BATTERY_VT_ERROR  = 0x07,
} Battery_StatusTypeDef;

#endif /* BATTERY_TYPES_H */
