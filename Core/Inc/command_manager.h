//
// Created by l on 2026/10/19.
//

#ifndef DCU_COMMAND_MANAGER_H
#define DCU_COMMAND_MANAGER_H

#include <stdint.h>

#include "joint_manager.h"

typedef struct {
    uint32_t sequence; //命令编号
    float target[JOINT_COUNT]; //四个关节的目标位置
    float velocity_limit; //允许的速度上限
}JointCommand;

typedef struct {
    uint32_t tick_id;
    JointCommand command;
}ControlSnapshot;

void Command_Publish(const JointCommand *cmd, uint32_t received_ms);
bool Command_HasReceived(void);
void Command_LatchForControlTick(uint32_t tick_id,uint32_t now_ms);
bool Command_GetNewSnapshot(ControlSnapshot *out,uint32_t *last_handled_tick);

#endif //DCU_COMMAND_MANAGER_H
