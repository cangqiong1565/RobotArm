#include "../Inc/joint_manager.h"
#include "string.h"
#include "bsp_fdcan.h"
#include "fdcan.h"
#include "cmsis_gcc.h"

#define FEEDBACK_VALID_WINDOW_MS 100U
#define HOST_TIMEOUT_HOLD_VELOCITY_RAD_S 0.20f
#define XOUT_ALL_MASK 0x0FU

static volatile uint8_t xout_epoch_active = 0U;
static volatile uint8_t xout_epoch_received_mask = 0U;
static volatile uint8_t xout_epoch_ready = 0U;

Joint joints[4];

void Joint_Init(void)
{
    memset(joints, 0, sizeof(joints));

    //关节1 -----------------------------------------
    joints[0].config.motor_id = 0x01;
    joints[0].config.master_id = 0x51;
    joints[0].config.zero_offset = -0.00719881f;
    joints[0].config.max_limit = 3.14159f;
    joints[0].config.min_limit = 0;
    joints[0].config.direction = 1;
    joints[0].config.pmax = 12.5f;
    joints[0].config.vmax = 30.0f;
    joints[0].config.tmax = 10.0f;

    //关节2 -----------------------------------------
    joints[1].config.motor_id = 0x02;
    joints[1].config.master_id = 0x52;
    joints[1].config.zero_offset = 0.000155f;
    joints[1].config.max_limit = 0.0f;
    joints[1].config.min_limit = -2.8274f;
    joints[1].config.direction = -1;
    joints[1].config.pmax = 12.5f;
    joints[1].config.vmax = 30.0f;
    joints[1].config.tmax = 10.0f;

    //关节3 -----------------------------------------
    joints[2].config.motor_id = 0x03;
    joints[2].config.master_id = 0x53;
    joints[2].config.zero_offset = -0.000462f;
    joints[2].config.max_limit = 3.14159f;
    joints[2].config.min_limit = 0;
    joints[2].config.direction = 1;
    joints[2].config.pmax = 12.5f;
    joints[2].config.vmax = 30.0f;
    joints[2].config.tmax = 10.0f;


    //关节4 -----------------------------------------
    joints[3].config.motor_id = 0x04;
    joints[3].config.master_id = 0x54;
    joints[3].config.zero_offset = -0.002699f;
    joints[3].config.max_limit = 5.8992f;
    joints[3].config.min_limit = 0;
    joints[3].config.direction = -1;
    joints[3].config.pmax = 12.5f;
    joints[3].config.vmax = 30.0f;
    joints[3].config.tmax = 10.0f;
}

Joint *Joint_FindById(uint8_t motor_id)
{
    for (uint8_t i = 0; i < JOINT_COUNT; i++)
    {
        if (joints[i].config.motor_id == motor_id)
        {
            return &joints[i];
        }
    }

    return NULL;
}

bool Joint_UpdateFeedback(const DM4310_Feedback *feedback, uint32_t now_ms, uint16_t rx_can_id)
{
    if (feedback == NULL)
        return false;

    Joint *joint = Joint_FindById(feedback->motor_id);

    if (joint == NULL)
        return false;

    if (rx_can_id != joint->config.master_id)
        return false;

    // 先检查配置
    if (!isfinite(joint->config.pmax) ||
        !isfinite(joint->config.vmax) ||
        !isfinite(joint->config.tmax) ||
        joint->config.pmax <= 0.0f ||
        joint->config.vmax <= 0.0f ||
        joint->config.tmax <= 0.0f)
        return false;

    if (joint->config.direction != 1 &&
        joint->config.direction != -1)
        return false;

    if (!isfinite(joint->config.zero_offset))
        return false;

    // 再进行换算
    float motor_position =
        -joint->config.pmax +
        ((float)feedback->position_raw / 65535.0f) *
        2.0f * joint->config.pmax;

    float motor_velocity =
        -joint->config.vmax +
        ((float)feedback->velocity_raw / 4095.0f) *
        2.0f * joint->config.vmax;

    float motor_torque =
        -joint->config.tmax +
        ((float)feedback->torque_raw / 4095.0f) *
        2.0f * joint->config.tmax;

    // 最后统一更新有效状态
    joint->state.motor_position = motor_position;
    joint->state.motor_velocity = motor_velocity;
    joint->state.motor_torque = motor_torque;

    joint->state.last_motor_feedback_ms = now_ms;
    joint->state.motor_feedback_valid = true;


    return true;
}

bool Joint_SendTarget(uint8_t joint_index, float target_position, float velocity_limit)
{
    if (joint_index >= JOINT_COUNT)
        return false;

    if (!isfinite(target_position) ||
        !isfinite(velocity_limit) ||
        velocity_limit <= 0.0f)
        return false;

    if (!isfinite(joints[joint_index].config.zero_offset))
        return false;

    if (joints[joint_index].config.direction != 1 &&
        joints[joint_index].config.direction != -1)
        return false;

    /* 检查限位配置和本次目标 */
    if (!isfinite(joints[joint_index].config.min_limit) ||
        !isfinite(joints[joint_index].config.max_limit) ||
        joints[joint_index].config.min_limit >=
            joints[joint_index].config.max_limit)
        return false;

    if (target_position < joints[joint_index].config.min_limit ||
        target_position > joints[joint_index].config.max_limit)
        return false;

    /* 开发阶段：暂不以标定、反馈、故障或反馈超时状态拦截位置命令。 */

    float motor_target =
        joints[joint_index].config.zero_offset +
        (float)joints[joint_index].config.direction *
        target_position;

    if (!DM4310_SendPositionVelocity(joints[joint_index].config.motor_id, motor_target, velocity_limit))
        return false;

    joints[joint_index].state.target_position = target_position;

    return true;
}

bool Joint_TargetsAreValid(const float target[JOINT_COUNT], float velocity_limit)
{
    if (target == NULL || !isfinite(velocity_limit) || velocity_limit <= 0.0f)
        return false;

    for (uint8_t i = 0U; i < JOINT_COUNT; i++)
    {
        if (!isfinite(target[i]) ||
            !isfinite(joints[i].config.min_limit) ||
            !isfinite(joints[i].config.max_limit) ||
            target[i] < joints[i].config.min_limit ||
            target[i] > joints[i].config.max_limit)
            return false;
    }
    return true;
}

void Joint_GetFeedbackSnapshot(JointFeedbackSnapshot *snapshot, uint32_t now_ms)
{
    if (snapshot == NULL)
        return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < JOINT_COUNT; i++)
    {
        snapshot->actual_position[i] = joints[i].state.actual_position;
        snapshot->received_us[i] = joints[i].state.last_output_position_rx_us;
        snapshot->valid[i] =
            (joints[i].state.feedback_valid &&
             ((uint32_t)(now_ms - joints[i].state.last_feedback_ms) <= FEEDBACK_VALID_WINDOW_MS))
            ? 1U : 0U;
    }
    __set_PRIMASK(primask);
}

void Joint_HostTimeoutHold(uint32_t now_ms)
{
    (void)now_ms;
    for (uint8_t i = 0U; i < JOINT_COUNT; i++)
    {
        (void)Joint_SendTarget(
            i,
            joints[i].state.target_position,
            HOST_TIMEOUT_HOLD_VELOCITY_RAD_S);
    }
}

bool Joint_UpdateOutputPosition(
    uint8_t motor_id,
    float xout_rad,
    uint32_t rx_us,
    uint16_t rx_can_id)
{
    Joint *joint = Joint_FindById(motor_id);

    if (joint == NULL ||
        rx_can_id != joint->config.master_id ||
        !isfinite(xout_rad) ||
        !isfinite(joint->config.zero_offset))
    {
        return false;
    }

    if (joint->config.direction != 1 &&
        joint->config.direction != -1)
    {
        return false;
    }

    joint->state.actual_position =
        (float)joint->config.direction *
        (xout_rad - joint->config.zero_offset);

    joint->state.last_output_position_rx_us = rx_us;
    joint->state.last_feedback_ms = HAL_GetTick();
    joint->state.feedback_valid = true;

    if (xout_epoch_active != 0U)
    {
        uint8_t bit = (uint8_t)(1U<<(motor_id - 1U));

        xout_epoch_received_mask |= bit;
        if (xout_epoch_received_mask == XOUT_ALL_MASK)
        {
            xout_epoch_ready = 1U;
        }
    }

    return true;
}

void Joint_BeginXoutEpoch(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    xout_epoch_received_mask = 0U;
    xout_epoch_ready = 0U;
    xout_epoch_active = 1U;

    __set_PRIMASK(primask);
}

void Joint_AbortXoutEpoch(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    xout_epoch_active = 0U;
    xout_epoch_received_mask = 0U;
    xout_epoch_ready = 0U;

    __set_PRIMASK(primask);
}


bool Joint_TakeCompletedXoutEpoch(void)
{
    bool ready;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ready = (xout_epoch_ready != 0U);

    if (ready)
    {
        xout_epoch_ready = 0U;
        xout_epoch_active = 0U;
    }

    __set_PRIMASK(primask);
    return ready;
}

