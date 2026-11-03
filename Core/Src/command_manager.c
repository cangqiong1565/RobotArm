#include "../Inc/command_manager.h"
#include "cmsis_gcc.h"

static JointCommand command_buffer;
static volatile uint8_t command_valid = 0;
static volatile uint32_t command_received_ms = 0;

static ControlSnapshot active_snapshot;
static volatile uint8_t active_valid;

void Command_Publish(const JointCommand *cmd, uint32_t received_ms)
{
    if (cmd == NULL) return;

    uint32_t primask = __get_PRIMASK(); // 记住原来的状态
    __disable_irq();                   // 屏蔽中断

    command_buffer = *cmd;
    command_received_ms = received_ms;
    command_valid = 1U;

    __set_PRIMASK(primask);             // 恢复原来的状态
}

bool Command_HasReceived(void)
{
    return command_valid != 0U;
}

void Command_LatchForControlTick(uint32_t tick_id,uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();
    /*
     *  0：普通中断当前允许响应
     *  1：普通中断当前已被屏蔽
     */

    __disable_irq();

    if ((command_valid != 0U) && ((uint32_t)(now_ms - command_received_ms) <= 150U))
    {
        active_snapshot.tick_id = tick_id;
        active_snapshot.command = command_buffer;
        active_valid = 1U;
    }
    else
    {
        active_valid = 0U;
    }

    __set_PRIMASK(primask);

}


bool Command_GetNewSnapshot(ControlSnapshot *out,uint32_t *last_handled_tick)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if ((active_valid == 0U) || (active_snapshot.tick_id == *last_handled_tick))
    {
        __set_PRIMASK(primask);
        return false;
    }

    *out = active_snapshot;
    *last_handled_tick = out->tick_id;

    __set_PRIMASK(primask);
    return true;
}
