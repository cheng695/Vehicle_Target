#include "GimbalController.hpp"
#include "RobotContext.hpp"
#include "GimbalTuning.hpp"
#include "main.h"
#include "pid.hpp"

// PID instances are owned by GimbalController.

namespace
{
GimbalController gimbal_controller;

}

GimbalController::GimbalController()
    : yaw_pid_(GimbalConfig::YAW_PID.kp, GimbalConfig::YAW_PID.ki,
               GimbalConfig::YAW_PID.kd, GimbalConfig::YAW_PID.output_limit,
               GimbalConfig::YAW_PID.integral_limit,
               GimbalConfig::YAW_PID.integral_separation_threshold),
      hight_pid_(GimbalConfig::HIGHT_PID.kp, GimbalConfig::HIGHT_PID.ki,
                 GimbalConfig::HIGHT_PID.kd, GimbalConfig::HIGHT_PID.output_limit,
                 GimbalConfig::HIGHT_PID.integral_limit,
                 GimbalConfig::HIGHT_PID.integral_separation_threshold)
{}

void GimbalController::Init()
{
    yaw_planner_.Init();
    hight_planner_.Init();
    yaw_pid_.reset();
    hight_pid_.reset();
    hight_stall_detector_.Reset();
    event_detector_.Reset();
    hight_zero_angle_deg_ = 0.0f;
    hight_position_valid_ = false;
    hight_previous_lower_limit_ = false;
    hight_previous_upper_limit_ = false;
}

void GimbalController::UpdateYaw(GimbalYawState state, const OperatorInput &input, float time_s,
                                 GimbalTarget &target, GimbalOutput &output)
{
    yaw_pid_.setK(GimbalTuning::YAW_PID_KP,
                  GimbalTuning::YAW_PID_KI,
                  GimbalTuning::YAW_PID_KD);
    yaw_pid_.setMax(GimbalTuning::YAW_PID_OUTPUT_LIMIT);
    yaw_pid_.setIntegralLimit(GimbalTuning::YAW_PID_INTEGRAL_LIMIT);
    yaw_pid_.setIntegralSeparation(GimbalTuning::YAW_PID_INTEGRAL_SEPARATION);
    target = yaw_planner_.Plan(state, input, time_s);

    output.out_gimbal_yaw[0] = 0.0f;
    output.out_gimbal_yaw[1] = target.target_yaw_velocity;
    output.out_gimbal_yaw[2] = GimbalTuning::YAW_PID_KP;
    output.out_gimbal_yaw[3] = GimbalTuning::YAW_PID_KD;
    output.out_gimbal_yaw[4] = 0.0f;
}

void GimbalController::StopYaw(GimbalTarget &target, GimbalOutput &output)
{
    MotorJ4310.setIsenable(false);
    target.target_yaw_velocity = 0.0f;
    for (float &value : output.out_gimbal_yaw)
        value = 0.0f;
}

void GimbalController::StopHight(GimbalTarget &target, GimbalOutput &output)
{
    hight_fsm_.ForceStop();
    hight_planner_.Init();
    hight_pid_.reset();
    // 停止只中断计时，不解除已经锁存的堵转故障。
    hight_stall_detector_.Update(false, 0.0f, 0.0f, 0.0f, HAL_GetTick());
    target.target_hight_velocity = 0.0f;
    output.out_gimbal_hight = 0.0f;
}

void GimbalController::UpdateHight(GimbalTarget &target, GimbalOutput &output, float time_s)
{
    const bool upper_limit_active =
        HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin) ==
        (GimbalConfig::HIGHT_UPPER_LIMIT_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);
    const bool lower_limit_active =
        HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin) ==
        (GimbalConfig::HIGHT_LOWER_LIMIT_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);

    const float angle_deg = Motor3508_5.getAddAngleDeg(GimbalConfig::HIGHT_MOTOR_CHANNEL);
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
            hight_zero_angle_deg_ = angle_deg - GimbalTuning::HIGHT_TRAVEL_DEG;
            hight_position_valid_ = true;
        }
    }
    hight_previous_lower_limit_ = lower_limit_active;
    hight_previous_upper_limit_ = upper_limit_active;

    if (HasHightStallFault() || hight_fsm_.GetCurrentState() == GimbalHightState::STOP)
    {
        StopHight(target, output);
        return;
    }

    const GimbalTarget hight_target = hight_planner_.Plan(
        hight_fsm_.GetCurrentState(),
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

    hight_pid_.setK(GimbalTuning::HIGHT_PID_KP,
                    GimbalTuning::HIGHT_PID_KI,
                    GimbalTuning::HIGHT_PID_KD);
    hight_pid_.setMax(GimbalTuning::HIGHT_PID_OUTPUT_LIMIT);
    hight_pid_.setIntegralLimit(GimbalTuning::HIGHT_PID_INTEGRAL_LIMIT);
    hight_pid_.setIntegralSeparation(GimbalTuning::HIGHT_PID_INTEGRAL_SEPARATION);
    const float actual_velocity = Motor3508_5.getVelocityRads(GimbalConfig::HIGHT_MOTOR_CHANNEL);
    float current_command = hight_pid_.UpDate(target.target_hight_velocity, actual_velocity);

    // 升降机构运动时加入定值重力前馈，抵消负载自重。
    if (hight_fsm_.GetCurrentState() != GimbalHightState::STOP)
        current_command += GimbalTuning::HIGHT_GRAVITY_FEEDFORWARD;

    const float hight_current_max = GimbalTuning::HIGHT_CURRENT_MAX;
    current_command = std::clamp(current_command,
                                 -hight_current_max,
                                 hight_current_max);

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
    gimbal_controller.Init();
    gimbal_target = GimbalTarget{};
    gimbal_output = GimbalOutput{};
}

void GimbalController::ControlLoop(bool is_online, const OperatorInput &input, float time_s,
                                   GimbalTarget &target, GimbalOutput &output)
{
    // 将遥控器拨杆转换为 Yaw 状态事件。
    // 每周期解析，离线恢复且挡位未变化时也能恢复状态。
    yaw_fsm_.Dispatch(event_detector_.ResolveYawEvent(input));

    const bool upper_limit_active =
        HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin) ==
        (GimbalConfig::HIGHT_UPPER_LIMIT_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);
    const bool lower_limit_active =
        HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin) ==
        (GimbalConfig::HIGHT_LOWER_LIMIT_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);
    
    const bool hight_enabled = is_online &&
                               Motor3508_5.isConnected(GimbalConfig::HIGHT_MOTOR_CHANNEL,
                                                       GimbalConfig::HIGHT_MOTOR_RING_ID) &&
                               !HasHightStallFault();
    if (hight_enabled)
    {
        hight_fsm_.Dispatch(event_detector_.ResolveHightEvent(input));
        hight_fsm_.Dispatch(event_detector_.ResolveHightLimitEvent(upper_limit_active, lower_limit_active));
    }
    else
    {
        hight_fsm_.ForceStop();
    }

    // 设备离线时强制进入停止状态。
    if (!is_online)
    {
        yaw_fsm_.ForceStop();
        hight_fsm_.ForceStop();
    }

    switch (yaw_fsm_.GetCurrentState())
    {
    case GimbalYawState::CONSTANT_SPEED:
    case GimbalYawState::VARIABLE_SPEED:
        UpdateYaw(yaw_fsm_.GetCurrentState(), input, time_s, target, output);
        MotorJ4310.setIsenable(true);
        break;

    case GimbalYawState::STOP:
    case GimbalYawState::STATUS_COUNT:
    default:
        StopYaw(target, output);
        break;
    }

    // 当前调用周期计入 FSM 状态运行时间。
    yaw_fsm_.UpdateTime();


    if (hight_enabled)
        UpdateHight(target, output, time_s);
    else
        StopHight(target, output);

    DM4310_velocity_rpm[0] = MotorJ4310.getVelocityRads(GimbalConfig::YAW_MOTOR_CHANNEL);
    M3508_5_velocity_rpm[0] = Motor3508_5.getVelocityRads(GimbalConfig::HIGHT_MOTOR_CHANNEL);
    M3508_5_position[0] = Motor3508_5.getAddAngleDeg(GimbalConfig::HIGHT_MOTOR_CHANNEL);
}

void GimbalControlLoop(bool is_online, const OperatorInput &input, float time_s)
{
    gimbal_controller.ControlLoop(is_online, input, time_s, gimbal_target, gimbal_output);
}
