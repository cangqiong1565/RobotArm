
#ifndef MYCAN_H
#define MYCAN_H

#include "main.h"
#include "fdcan.h"
#include <stdint.h>

/* 初始化 FDCAN1 */
HAL_StatusTypeDef mycan_init(void);

/* 发送标准 CAN 数据帧，长度 0~8 字节 */
HAL_StatusTypeDef mycan_send(
    FDCAN_HandleTypeDef *hfdcan,
    uint16_t id,
    const uint8_t *txdata,
    uint8_t len
);

/* 轮询接收，timeout 单位 ms */
HAL_StatusTypeDef mycan_receive(
    FDCAN_HandleTypeDef *hfdcan,
    uint8_t *rxdata,
    uint32_t timeout,
    FDCAN_RxHeaderTypeDef *rxheader
);

#endif
