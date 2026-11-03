//
// Created by l on 2026/10/19.
//

#ifndef DCU_HOST_PROTOCOL_H
#define DCU_HOST_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    WAIT_AA,
    WAIT_55,
    RECEIVE_DATA
} HostRxState;

bool HostProtocol_Receive(const uint8_t *buf, uint32_t len);
bool HostProtocol_TryTransmitFeedback(uint32_t now_ms);
void HostProtocol_NotifyTxComplete(void);

#endif //DCU_HOST_PROTOCOL_H
