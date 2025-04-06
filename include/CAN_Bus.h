/*
 * CAN_Bus.h
 *
 *  Created on: 08.03.2025
 *      Author: renolluc / grossfa2
 */

#ifndef INC_CAN_BUS_H
#define INC_CAN_BUS_H

#include <stdint.h>
#include <battery.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

// Expose the CAN message queue for testing purposes
extern struct k_msgq can_msgq;

// CAN device
#define CAN_DEVICE DT_LABEL(DT_NODELABEL(can1))

#define ADDR_ECU_RX 0x410 // Address used ECU => CB
#define ADDR_ECU_TX 0x310 // Address used CB => ECU

//>> ISA defines
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

//CAN receive Flags in 8 bit format, can byte 0 LSB neue Flags nur für Akku ein- und ausschalten
#define BATTERY_ON 		 (1<<0)
#define BATTERY_OFF 	 (0<<0)
#define BATTERY_SW_RESET (1<<3) 

// CAN receive Flags in 8 bit format, can byte 0 LSB wird von obigen Flags ersetzt
/*
#define AIR_POSITIVE 		(1<<0)
#define AIR_NEGATIVE 		(1<<1)
#define PRECHARGE_RELAY 	(1<<2)
#define BATTERY_SW_RESET	(1<<3) 
*/

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

int BMS_CAN_INIT(void);
int send_CAN(uint32_t address, uint8_t *TxBuffer);
int send_CAN_IVT_nbytes(uint32_t address, uint8_t *TxBuffer, uint8_t length);
int send_data2ECU(uint16_t GPIO_Input);
int ISA_IVT_Init(void);

#endif