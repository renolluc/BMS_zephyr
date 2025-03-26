/*
 * CAN_Bus.h
 *
 *  Created on: 08.03.2025
 *      Author: renolluc / grossfa2
 */

#ifndef INC_CAN_BUS_H
#define INC_CAN_BUS_H

#include <stdint.h>

#define CAN_DEVICE DT_LABEL(DT_NODELABEL(can1))  // Zephyr CAN-Device
//can1 ist die Verküpfung im Device Tree

int BMS_CAN_INIT(void);
int send_can_message(const uint8_t *data);

#endif