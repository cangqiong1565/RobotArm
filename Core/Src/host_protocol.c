#include "../Inc/host_protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "command_manager.h"

#include "joint_manager.h"
#include "usbd_cdc_if.h"

#define HOST_COMMAND_FRAME_SIZE 28U
#define HOST_FEEDBACK_FRAME_SIZE 48U
#define HOST_FEEDBACK_HEADER_0 0x5AU
#define HOST_FEEDBACK_HEADER_1 0xA5U

static uint8_t feedback_tx_buffers[2][HOST_FEEDBACK_FRAME_SIZE];
static volatile uint8_t feedback_tx_busy = 0U;
static uint8_t feedback_tx_index = 0U;
static uint32_t feedback_sequence = 0U;

static uint16_t CRC16_Modbus(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xA001U;
            else
                crc >>= 1;
        }
    }
    return crc;
}

bool HostProtocol_Receive(const uint8_t *buf, uint32_t len)
{
    static uint8_t frame[HOST_COMMAND_FRAME_SIZE];
    static uint32_t used = 0;
    static HostRxState state = WAIT_AA;
    bool published = false;

    if (buf == NULL)
    {
        return false;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t byte = buf[i];

        switch (state)
        {
            case WAIT_AA:
            if (byte == 0xAA)
            {
                frame[0] = byte;
                state = WAIT_55;
            }break;

            case WAIT_55:
                if (byte == 0x55)
                {
                    frame[1] = byte;
                    used = 2U;
                    state = RECEIVE_DATA;
                }
                else if (byte != 0xAA)
                {
                    state = WAIT_AA;
                }break;

            case RECEIVE_DATA:
                frame[used] = byte;
                used++;

                if (used == sizeof(frame))
                {
                    uint16_t calculated_crc = CRC16_Modbus(&frame[2], used - 4U);
                    uint16_t received_crc = (uint16_t)frame[26] | ((uint16_t)frame[27] << 8);

                    if (calculated_crc == received_crc)
                    {
                        JointCommand cmd;

                        memcpy(&cmd.sequence,       &frame[2],  4U);
                        memcpy(cmd.target,          &frame[6], 16U);
                        memcpy(&cmd.velocity_limit, &frame[22], 4U);

                        if (Joint_TargetsAreValid(cmd.target, cmd.velocity_limit))
                        {
                            Command_Publish(&cmd, HAL_GetTick());
                            published = true;
                        }
                    }

                    used = 0U;
                    state = WAIT_AA;
                }
                break;

                default:
                used = 0U;
                state = WAIT_AA;
                break;
        }
    }

    return published;
}

bool HostProtocol_TryTransmitFeedback(uint32_t now_ms)
{
    JointFeedbackSnapshot snapshot;
    uint8_t *frame;
    uint16_t crc;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (feedback_tx_busy != 0U)
    {
        __set_PRIMASK(primask);
        return false;
    }
    feedback_tx_busy = 1U;
    frame = feedback_tx_buffers[feedback_tx_index];
    feedback_tx_index ^= 1U;
    __set_PRIMASK(primask);

    Joint_GetFeedbackSnapshot(&snapshot, now_ms);
    frame[0] = HOST_FEEDBACK_HEADER_0;
    frame[1] = HOST_FEEDBACK_HEADER_1;
    feedback_sequence++;
    memcpy(&frame[2], &feedback_sequence, sizeof(uint32_t));
    memcpy(&frame[6], &now_ms, sizeof(uint32_t));
    memcpy(&frame[10], snapshot.actual_position, sizeof(snapshot.actual_position));
    memcpy(&frame[26], snapshot.received_us, sizeof(snapshot.received_us));
    memcpy(&frame[42], snapshot.valid, sizeof(snapshot.valid));
    crc = CRC16_Modbus(&frame[2], 44U);
    frame[46] = (uint8_t)(crc & 0xFFU);
    frame[47] = (uint8_t)(crc >> 8);

    if (CDC_Transmit_HS(frame, HOST_FEEDBACK_FRAME_SIZE) != USBD_OK)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        feedback_tx_busy = 0U;
        __set_PRIMASK(primask);
        return false;
    }
    return true;
}

void HostProtocol_NotifyTxComplete(void)
{
    feedback_tx_busy = 0U;
}
