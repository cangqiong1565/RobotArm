#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include <stdbool.h>

#include "main.h"

#define hcan_t FDCAN_HandleTypeDef

#define POSITION_AND_SPEED_OFFSET 0x100

typedef struct
{
    uint8_t motor_id;
    uint8_t state;

    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;

    uint8_t mos_temp;
    uint8_t rotor_temp;

} DM4310_Feedback;

void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan_usb_rx(uint8_t*buf,uint32_t len);
void fdcan_usb_process(void);
void canx_rx_parse(DM4310_Feedback *motor, const uint8_t *data);
bool DM4310_SendPositionVelocity(uint8_t id, float position, float velocity_limit);
bool DM4310_RequestOutputPosition(uint8_t motor_id);
bool DM4310_ParseOutputPositionReply(
    uint16_t rx_id, const uint8_t data[8],
    uint8_t *motor_id, float *xout_rad);

#endif /* __BSP_FDCAN_H_ */

