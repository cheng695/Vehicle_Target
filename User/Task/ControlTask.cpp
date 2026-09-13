#include "ControlTask.hpp"
#include "ChassisController.hpp"
#include "RobotContext.hpp"
#include "buzzer_manager.hpp"

/*
 * 检查底盘控制依赖设备是否在线。
 *
 * 当前只把遥控器作为硬保护条件；电机和裁判系统在线检查保留为注释，
 * 避免改变上场初始化阶段的行为。
 */
bool check_online()
{
    bool isconnected = true;

    for (int i = 0; i < 4; i++)
    {
        if (!Motor3508.isConnected(i + 1, i + 1))
        {
            isconnected = false;
        }
    }

    // if(!MotorJ4310.isConnected(1, 1))
    // {
    //     isconnected = false;
    // }

    if (!DT7.isConnected())
    {
        isconnected = false;
    }


    return isconnected;
}

/*
 * 兼容旧接口：初始化底盘控制模块。
 */
void fsm_init()
{
    InitChassisControl();
}

extern "C" {
void ControlTask(void const *argument)
{
    (void)argument;

    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    InitChassisControl();

    for (;;)
    {
        // BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        ChassisControlLoop(check_online(), operator_input);
        osDelay(1);
    }
}
}
