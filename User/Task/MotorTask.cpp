#include "MotorTask.hpp"
#include "RobotContext.hpp"
#include "can_device.hpp"

float m3508_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/*
 * 注册 CAN 接收回调。
 */
void MotorInit(void)
{
    static auto& can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto& can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame& frame)
    {
        Motor3508.Parse(frame);
    });

    can2.register_rx_callback([](const HAL::CAN::Frame& frame)
    {
        MotorJ4310.Parse(frame);
    });
}

/*
 * 下发底盘电机和超电 CAN 指令。
 */
static void motor_control_logic(uint32_t tick)
{
    (void)tick;

    for (int i = 0; i < 4; i++)
    {
        m3508_current[i] = Motor3508.getVelocityRpm(i+1);
        Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
    }

    Motor3508.sendCAN();
}

extern "C" {
void CanTask(void const* argument)
{
    (void)argument;

    MotorInit();

    static uint32_t loop_count = 0;
    for (;;)
    {
        loop_count++;
        motor_control_logic(loop_count);

        osDelay(1);
    }
}
}
