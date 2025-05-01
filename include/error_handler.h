/*
 * error_handler.h
 *
 *  Created on: 20.04.2025
 *      Author: renolluc / grossfa2
 ******************************************************************************
 * @file           : error_handler.h
 * @brief          : Header for error handling in the BMS system.
 */

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

typedef enum
{
    ERR_SRC_CAN,  // error in Can thread
    ERR_SRC_BATT, // error in Batterythread
} ErrorSource_t;

typedef struct
{
    ErrorSource_t source;
    uint32_t code; // z.B. Hersteller-Errorcode oder Thread-intern
} ErrorEvent;

extern struct k_msgq err_evt_queue;
#endif /* INC_ERROR_HANDLER_H_ */