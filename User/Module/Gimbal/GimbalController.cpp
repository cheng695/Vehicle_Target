#include "GimbalController.hpp"
#include "GimbalEventDetector.hpp"
#include "RobotContext.hpp"
#include "main.h"
#include "pid.hpp"

ALG::PID::PID yaw_pid(0.0f, 0.0f, 1.6f, 0.0f, 0.0f, 0.0f);
ALG::PID::PID hight_pid(3000.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);

namespace
{
constexpr GPIO_PinState HIGHT_LIMIT_ACTIVE_LEVEL = GPIO_PIN_RESET;
constexpr float HIGHT_GRAVITY_FEEDFORWARD = 500.0f; //850
constexpr float HIGHT_CURRENT_MAX = 16384.0f;
}

namespace
{
GimbalController gimbal_controller;

void StopGimbal()
{
    MotorJ4310.setIsenable(false);
    gimbal_target.target_yaw_velocity = 0.0f;
    for (float &output : gimbal_output.out_gimbal_yaw)
        output = 0.0f;
}
}

GimbalController::GimbalController() = default;

void GimbalController::Init()
{
    yaw_planner_.Init();
    hight_planner_.Init();
    yaw_pid.reset();
    hight_pid.reset();
    hight_stall_detector_.Reset();
    hight_zero_angle_deg_ = 0.0f;
    hight_position_valid_ = false;
    hight_previous_lower_limit_ = false;
    hight_previous_upper_limit_ = false;
}

void GimbalController::UpdateYaw(GimbalYawState state, const OperatorInput &input, float time_s,
                                 GimbalTarget &target, GimbalOutput &output)
{
    target = yaw_planner_.Plan(state, input, time_s);

    output.out_gimbal_yaw[0] = 0.0f;
    output.out_gimbal_yaw[1] = target.target_yaw_velocity;
    output.out_gimbal_yaw[2] = yaw_pid.getKp();
    output.out_gimbal_yaw[3] = yaw_pid.getKd();
    output.out_gimbal_yaw[4] = 0.0f;
}

void GimbalController::StopHight(GimbalTarget &target, GimbalOutput &output)
{
    gimbal_hight_fsm.ForceStop();
    hight_planner_.Init();
    hight_pid.reset();
    // 停止只中断计时，不解除已经锁存的堵转故障。
    hight_stall_detector_.Update(false, 0.0f, 0.0f, 0.0f, HAL_GetTick());
    target.target_hight_velocity = 0.0f;
    output.out_gimbal_hight = 0.0f;
}

void GimbalController::UpdateHight(GimbalTarget &target, GimbalOutput &output, float time_s)
{
    const bool upper_limit_active =
        HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin) == HIGHT_LIMIT_ACTIVE_LEVEL;
    const bool lower_limit_active =
        HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin) == HIGHT_LIMIT_ACTIVE_LEVEL;

    const float angle_deg = Motor3508_5.getAddAngleDeg(1);
    if (!(upper_limit_active && lower_limit_active))
    {
        // 仅在刚触发时校准，离开开关期间不能持续移动零点。
        if (lower_limit_active && (!hight_previous_lower_limit_ || !hight_position_valid_))
        {
            hight_zero_angle_deg_ = angle_deg;
            hight_position_valid_ = true;
        }
        else if (upper_limit_active && (!hight_previous_upper_limit_ || !hight_position_valid_))
        {
            hight_zero_angle_deg_ = angle_deg - GimbalHightPlanner::HIGHT_TRAVEL_DEG;
            hight_position_valid_ = true;
        }
    }
    hight_previous_lower_limit_ = lower_limit_active;
    hight_previous_upper_limit_ = upper_limit_active;

    if (HasHightStallFault() || gimbal_hight_fsm.Get_Now_State() == GimbalHightState::STOP)
    {
        StopHight(target, output);
        return;
    }

    const GimbalTarget hight_target = hight_planner_.Plan(
        gimbal_hight_fsm.Get_Now_State(),
        time_s,
        upper_limit_active,
        lower_limit_active,
        angle_deg - hight_zero_angle_deg_,
        hight_position_valid_);

    target.target_hight_velocity = hight_target.target_hight_velocity;

    if (upper_limit_active && lower_limit_active)
    {
        StopHight(target, output);
        return;
    }

    const float actual_velocity = Motor3508_5.getVelocityRads(1);
    float current_command = hight_pid.UpDate(target.target_hight_velocity, actual_velocity);

    // 升降机构运动时加入定值重力前馈，抵消负载自重。
    if (gimbal_hight_fsm.Get_Now_State() != GimbalHightState::STOP)
        current_command += HIGHT_GRAVITY_FEEDFORWARD;

    current_command = std::clamp(current_command, -HIGHT_CURRENT_MAX, HIGHT_CURRENT_MAX);

    // 限位保护作用于最终电流，防止 PID 仍然向限位方向施力。
    if (upper_limit_active && current_command > 0.0f)
        current_command = 0.0f;
    if (lower_limit_active && current_command < 0.0f)
        current_command = 0.0f;

    // 调试期望值期间暂时禁用堵转检测，实机运行前恢复此段。
    // if (hight_stall_detector_.Update(true, target.target_hight_velocity,
    //                                  actual_velocity, current_command, HAL_GetTick()))
    // {
    //     StopHight(target, output);
    //     return;
    // }

    output.out_gimbal_hight = current_command;
}

void InitGimbalControl()
{
    MotorJ4310.setIsenable(false);
    gimbal_fsm.Init();
    gimbal_hight_fsm.Init();
    gimbal_controller.Init();
    gimbal_target = GimbalTarget{};
    gimbal_output = GimbalOutput{};
}

void GimbalControlLoop(bool is_online, const OperatorInput &input, float time_s)
{
    // 将遥控器拨杆转换为 Yaw 状态事件。
    // 每周期解析，离线恢复且挡位未变化时也能恢复状态。
    gimbal_fsm.Dispatch(ResolveGimbalYawEvent(input));

    const bool upper_limit_active = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin) == HIGHT_LIMIT_ACTIVE_LEVEL;
    const bool lower_limit_active = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin) == HIGHT_LIMIT_ACTIVE_LEVEL;
    
    const bool hight_enabled = is_online && Motor3508_5.isConnected(1, 5) &&
                               !gimbal_controller.HasHightStallFault();
    if (hight_enabled)
    {
        gimbal_hight_fsm.Dispatch(ResolveGimbalHightEvent(input));
        gimbal_hight_fsm.Dispatch(ResolveGimbalHightLimitEvent(upper_limit_active, lower_limit_active));
    }
    else
    {
        gimbal_hight_fsm.ForceStop();
    }

    // 设备离线时强制进入停止状态。
    if (!is_online)
    {
        gimbal_fsm.ForceStop();
        gimbal_hight_fsm.ForceStop();
    }

    switch (gimbal_fsm.Get_Now_State())
    {
    case GimbalYawState::CONSTANT_SPEED:
    case GimbalYawState::VARIABLE_SPEED:
        gimbal_controller.UpdateYaw(gimbal_fsm.Get_Now_State(), input, time_s, gimbal_target, gimbal_output);
        MotorJ4310.setIsenable(true);
        break;

    case GimbalYawState::STOP:
    case GimbalYawState::STATUS_COUNT:
    default:
        StopGimbal();
        break;
    }

    // 当前调用周期计入 FSM 状态运行时间。
    gimbal_fsm.TIM_Update();


    if (hight_enabled)
        gimbal_controller.UpdateHight(gimbal_target, gimbal_output, time_s);
    else
        gimbal_controller.StopHight(gimbal_target, gimbal_output);

    DM4310_velocity_rpm[0] = MotorJ4310.getVelocityRads(1);
    M3508_5_velocity_rpm[0] = Motor3508_5.getVelocityRads(1);
    M3508_5_position[0] = Motor3508_5.getAddAngleDeg(1);
}
