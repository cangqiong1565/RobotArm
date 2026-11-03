//
// Created by l on 2026/10/18.
//

#ifndef DCU_JOINT_MANAGER_H
#define DCU_JOINT_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#define JOINT_COUNT 4

#include "bsp_fdcan.h"

typedef struct
{
    uint8_t motor_id;
    uint16_t master_id;

    float zero_offset;
    int8_t direction;

    float pmax;
    float vmax;
    float tmax;

    float min_limit;
    float max_limit;

} JointConfig;

typedef struct {
    float actual_position;
    float actual_velocity;
    float actual_torque;

    float target_position;

    uint32_t last_feedback_ms;

    bool feedback_valid;
    bool calibrated;
    bool fault;

    uint8_t motor_state;

    float motor_position;
    float motor_velocity;
    float motor_torque;


    uint32_t last_motor_feedback_ms;
    uint32_t last_output_position_rx_us;
    bool motor_feedback_valid;

}JointState;

typedef struct
{
    JointConfig config;
    JointState state;

} Joint;

typedef struct
{
    float actual_position[JOINT_COUNT];
    uint8_t valid[JOINT_COUNT];

    uint32_t received_us[JOINT_COUNT];
} JointFeedbackSnapshot;

extern Joint joints[4];

Joint *Joint_FindById(uint8_t motor_id);
void Joint_Init(void);
bool Joint_SendTarget(uint8_t joint_index, float target_position, float velocity_limit);
bool Joint_UpdateFeedback(const DM4310_Feedback *feedback, uint32_t now_ms, uint16_t rx_can_id);
bool Joint_TargetsAreValid(const float target[JOINT_COUNT], float velocity_limit);
void Joint_GetFeedbackSnapshot(JointFeedbackSnapshot *snapshot, uint32_t now_ms);
void Joint_HostTimeoutHold(uint32_t now_ms);
bool Joint_UpdateOutputPosition(
    uint8_t motor_id,
    float xout_rad,
    uint32_t rx_us,
    uint16_t rx_can_id);

void Joint_BeginXoutEpoch(void);
void Joint_AbortXoutEpoch(void);
bool Joint_TakeCompletedXoutEpoch(void);

#endif //DCU_JOINT_MANAGER_H
