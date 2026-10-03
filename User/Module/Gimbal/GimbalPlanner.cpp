#include "GimbalPlanner.hpp"

#include <cmath>

/**
 * @brief 规划 Yaw 速度目标
 * 
 * @param state 
 * @param input 
 * @param time_s 
 * @return GimbalYawTarget 
 */
GimbalTarget GimbalYawPlanner::Plan(GimbalYawState state,
                                    const OperatorInput &input,
                                    float time_s)
{
    const float scroll = input.scroll;
    GimbalTarget target;

    // 进入匀速挡或变速挡时，期望速度和方向都从零开始。
    if (state != previous_state_)
    {
        if (state == GimbalYawState::CONSTANT_SPEED)
            constant_speed_target_ = 0.0f;
        else if (state == GimbalYawState::VARIABLE_SPEED)
            variable_direction_ = 0;

        // 切挡时先保持零速度；已拨住的滚轮必须回中后重新拨动。
        scroll_event_latched_ = IsScrollPositive(scroll) || IsScrollNegative(scroll);
        previous_state_ = state;
        return target;
    }

    switch (state)
    {
    case GimbalYawState::CONSTANT_SPEED:
        // 每次滚轮有效拨动改变 0.4 rad/s，最大限幅为 ±0.8 rad/s。
        if (!scroll_event_latched_ && IsScrollPositive(scroll))
        {
            constant_speed_target_ += YAW_CONSTANT_SPEED_STEP_RADPS;
            scroll_event_latched_ = true;
        }
        else if (!scroll_event_latched_ && IsScrollNegative(scroll))
        {
            constant_speed_target_ -= YAW_CONSTANT_SPEED_STEP_RADPS;
            scroll_event_latched_ = true;
        }
        else if (std::fabs(scroll) <= YAW_SCROLL_THRESHOLD)
        {
            scroll_event_latched_ = false;
        }

        constant_speed_target_ = std::clamp(constant_speed_target_,
                                            -YAW_CONSTANT_SPEED_LIMIT_RADPS,
                                            YAW_CONSTANT_SPEED_LIMIT_RADPS);
        target.target_yaw_velocity = constant_speed_target_;
        break;

    case GimbalYawState::VARIABLE_SPEED:
    {
        const int8_t previous_direction = variable_direction_;
        // 第一次反向拨动只停止，第二次同方向拨动才开始反向转动。
        if (!scroll_event_latched_ && IsScrollPositive(scroll))
        {
            variable_direction_ = variable_direction_ < 0 ? 0 : 1;
            scroll_event_latched_ = true;
        }
        else if (!scroll_event_latched_ && IsScrollNegative(scroll))
        {
            variable_direction_ = variable_direction_ > 0 ? 0 : -1;
            scroll_event_latched_ = true;
        }
        else if (std::fabs(scroll) <= YAW_SCROLL_THRESHOLD)
        {
            scroll_event_latched_ = false;
        }

        if (previous_direction == 0 && variable_direction_ != 0)
            variable_start_time_s_ = time_s;
        if (variable_direction_ != 0)
            target.target_yaw_velocity = variable_direction_ *
                GetVariableSpeed(time_s - variable_start_time_s_);
        break;
    }

    case GimbalYawState::STOP:
    case GimbalYawState::STATUS_COUNT:
    default:
        target.target_yaw_velocity = 0.0f;
        break;
    }

    LimitTarget(target);
    return target;
}

/**
 * @brief 规划 Hight 速度目标
 * 
 * @param state 
 * @param time_s 
 * @param upper_limit_active 
 * @param lower_limit_active 
 * @return GimbalTarget 
 */
float GimbalHightPlanner::LimitSpeedByPosition(float speed_radps, float position_deg, bool position_valid)
{
    float speed_limit = HIGHT_LIMIT_SPEED_RADPS;
    if (position_valid)
    {
        const float slowdown_distance = HIGHT_TRAVEL_DEG * (1.0f - HIGHT_SLOWDOWN_START_RATIO);
        const float remaining_distance = speed_radps > 0.0f
            ? HIGHT_TRAVEL_DEG - position_deg : position_deg;
        const float ratio = std::clamp(remaining_distance / slowdown_distance, 0.0f, 1.0f);
        // 最后 30% 行程内按剩余距离线性降速，超出估计终点后仍低速寻找微触开关。
        speed_limit += (std::max(std::fabs(speed_radps), HIGHT_LIMIT_SPEED_RADPS)
                        - HIGHT_LIMIT_SPEED_RADPS) * ratio;
    }
    // 尚未建立位置基准时按低速运行，不能把初始编码器值当作机械零点。
    return std::clamp(speed_radps, -speed_limit, speed_limit);
}

GimbalTarget GimbalHightPlanner::Plan(GimbalHightState state,
                                      float time_s,
                                      bool upper_limit_active,
                                      bool lower_limit_active,
                                      float position_deg,
                                      bool position_valid)
{
    GimbalTarget target{};

    if (!initialized_)
    {
        initialized_ = true;
        previous_time_s_ = time_s;
        previous_state_ = state;
    }

    const float dt = std::clamp(time_s - previous_time_s_, 0.0005f, 0.02f);
    const float target_speed_radps = LimitSpeedByPosition(
        GetTargetSpeed(state), position_deg, position_valid);
    const float max_speed_step = HIGHT_ACCELERATION_RADPS2 * dt;

    if (current_speed_radps_ < target_speed_radps)
    {
        current_speed_radps_ = std::min(
            current_speed_radps_ + max_speed_step,
            target_speed_radps);
    }
    else if (current_speed_radps_ > target_speed_radps)
    {
        current_speed_radps_ = std::max(
            current_speed_radps_ - max_speed_step,
            target_speed_radps);
    }
    current_speed_radps_ = std::clamp(
        current_speed_radps_,
        -HIGHT_MAX_SPEED_RADPS,
        HIGHT_MAX_SPEED_RADPS);

    // 限位开关只能禁止继续向限位方向运动，离开当前限位的方向仍然允许。
    if (upper_limit_active && current_speed_radps_ > 0.0f)
        current_speed_radps_ = 0.0f;
    if (lower_limit_active && current_speed_radps_ < 0.0f)
        current_speed_radps_ = 0.0f;

    // 上下限位同时触发表示机械状态异常，速度目标强制置零。
    if (upper_limit_active && lower_limit_active)
        current_speed_radps_ = 0.0f;

    target.target_hight_velocity = current_speed_radps_;
    previous_state_ = state;
    previous_time_s_ = time_s;
    return target;
}
