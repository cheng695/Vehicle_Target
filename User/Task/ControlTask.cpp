#include "ControlTask.hpp"
#include "ChassisController.hpp"
#include "GimbalController.hpp"
#include "RobotContext.hpp"
#include "buzzer_manager.hpp"
#include "main.h"

// Hight 上下限位调试值：1 表示引脚为高电平，0 表示引脚为低电平。
volatile uint8_t debug_hight_limit_a_level = 0;
volatile uint8_t debug_hight_limit_b_level = 0;

void UpdateHightLimitSwitchDebug()
{
    debug_hight_limit_a_level =
        HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin) == GPIO_PIN_SET ? 1U : 0U;
    debug_hight_limit_b_level =
        HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin) == GPIO_PIN_SET ? 1U : 0U;
}

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


extern "C" {
void ControlTask(void const *argument)
{
    (void)argument;

    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    InitChassisControl();
    InitGimbalControl();

    for (;;)
    {
        UpdateHightLimitSwitchDebug();

        // BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        ChassisControlLoop(check_online(), operator_input);
        GimbalControlLoop(check_online(), operator_input, osKernelSysTick() * 0.001f);
        
        osDelay(1);
    }
}
}
