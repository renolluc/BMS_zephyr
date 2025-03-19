/*
 * CAN_Bus.h
 *
 *  Created on: 08.03.2025
 *      Author: renolluc / grossfa2
 */

#include <CAN_Bus.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(custom_can, LOG_LEVEL_DBG);

static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

int can_init(void) {
    if (!device_is_ready(can_dev)) {
        LOG_INF("CAN device not ready\n");
        return -1;
    }
    LOG_INF("CAN initialized\n");
    return 0;
}

int can_send_msg(uint32_t id, uint8_t *data, uint8_t len) {
    struct can_frame frame;
    frame.id = id;
    frame.dlc = len;
    frame.flags = CAN_FRAME_IDE; // Standard Frame

    memcpy(frame.data, data, len);

    int err = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    return err;
}

int can_receive_msg(uint32_t *id, uint8_t *data) {
    struct can_frame frame;
    int err = can_recv(can_dev, &frame, K_FOREVER, NULL, NULL);

    if (err == 0) {
        *id = frame.id;
        memcpy(data, frame.data, frame.dlc);
    }
    return err;
}

