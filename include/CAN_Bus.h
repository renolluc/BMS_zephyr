/** 
 * @file   CAN_Bus.h
 * @brief  API-definition for the Can module in the BMS-System.
 * @author renolluc / grossfa2
 * @date   20.04.2025
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

/* Zephyr-Framework includes */
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
/* Project includes */
#include <Battery.h>

/** @brief CAN device label for Zephyr device tree */
#define CAN_DEVICE DT_LABEL(DT_NODELABEL(can1))

/** @brief Semaphore for RX thread test acknowledgment */
extern struct k_sem test_ack_sem;

/** @brief Message ID used by RX thread test communication */
#define TEST_RXTHREAD_ID 0x7A0

/** @brief Message ID used for ECU communication */
#define ADDR_ECU_RX 0x410 // Address used ECU => CB
#define ADDR_ECU_TX 0x310 // Address used CB => ECU

/** @brief Message ID used for IVT communication */
// adresses
#define IVT_MSG_COMMAND   0x411		// CB => ISA
#define IVT_MSG_RESPONSE  0x511		// ISA => CB
#define IVT_MSG_RESULT_I  0x521		// ISA => CB
#define IVT_MSG_RESULT_U1 0x522		// ISA => CB
#define IVT_MSG_RESULT_U2 0x523		// ISA => CB
#define IVT_MSG_RESULT_U3 0x524		// ISA => CB
#define IVT_MSG_RESULT_T  0x525		// ISA => CB
#define IVT_MSG_RESULT_W  0x526		// ISA => CB
#define IVT_MSG_RESULT_AS 0x527		// ISA => CB
#define IVT_MSG_RESULT_WH 0x528		// ISA => CB

// Muxbyte
#define MUX_RESULTS 	0x00
#define MUX_SETCANID 	0x10
#define MUX_SETCONFIG 	0x20
#define MUX_SETCOMMAND 	0x30
#define MUX_GETLOGDATA 	0x40
#define MUX_GETCANID 	0x50
#define MUX_GETCONFIG 	0x60
#define MUX_GETCOMMAND 	0x70
#define MUX_RESPLOGDATA 0x80
#define MUX_RESPCANID	0x90
#define MUX_RESPCONFIG 	0xA0
#define MUX_RESPSETGET 	0xB0
#define MUX_NOTALLOWED	0xFF

#define SET_MODE 0x34

#define ISA_SERIAL_NUMBER 10139584

// channel n
#define IVT_NCURRENT 0
#define IVT_NU1 1
#define IVT_NU2 2
#define IVT_NU3 3
#define IVT_NT 4
#define IVT_NP 5
#define IVT_NQ 6
#define IVT_NE 7

// operating mode
#define DISABLED 0
#define TRIGGERED 1
#define CYCLIC 2

// settings
#define CYCLETIME 100  // in ms


// CAN send data frame
// 0 status flags
// 1 error flags
// 2 HVvoltage LOW
// 3 HVvoltage HIGH
// 4 HVcurrent
// 5 accu temp
// 6 SOC
// 7 reserved

//wird vermutlich nicht mehr gebraucht
//extern CAN_HandleTypeDef hcan1;

/**
 * @brief Initializes CAN controller and receive thread.
 *
 * @return 0 on success, error code otherwise.
 */
extern int can_init(void);

/**
 * @brief Sends an 8-byte CAN frame to the specified address.
 *
 * @param address Target CAN ID.
 * @param TxBuffer Pointer to 8-byte data buffer.
 * @return 0 on success, error code otherwise.
 */
int can_send_8bytes(uint32_t address, uint8_t *TxBuffer);

/**
 * @brief Sends a CAN frame with specified byte length.
 *
 * @param address Target CAN ID.
 * @param TxBuffer Pointer to data buffer.
 * @param length Number of bytes to send.
 * @return 0 on success, error code otherwise.
 */
int can_send_ivt_nbytes(uint32_t address, uint8_t *TxBuffer, uint8_t length);

/**
 * @brief Formats and sends sensor data to the ECU.
 *
 * @param GPIO_Input GPIO input encoding system status.
 * @return 0 on success, error code otherwise.
 */
int can_send_ecu(void);

/**
 * @brief Checks if the ECU flag is OK or NOK. OK means the accumulator can be connected.
 *
 * @return 1 if ECU is OK, 0 otherwise.
 */
extern int can_get_ecu_state();

/**
 * @brief Initializes IVT sensor with measurement configurations.
 *
 * @return 0 on success, error code otherwise.
 */
extern int can_ivt_init(void);

#endif