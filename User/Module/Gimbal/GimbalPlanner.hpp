#pragma once

#include "GimbalStateMachine.hpp"
#include "GimbalTypes.hpp"
#include "OperatorInput.hpp"

#include <algorithm>
#include <cmath>

class GimbalYawPlanner
{
public:
    // 重置 Yaw 规划器内部状态。
    void Init();

    GimbalTarget Plan(GimbalYawState state,
                      const OperatorInput &input,
                      float time_s);

private:
    float variable_start_time_s_ = 0.0f; // 本次变速转动的起始时间
    // 匀速 Yaw 速度每次拨动滚轮增加 0.4 rad/s，最大限幅为 ±0.8 rad/s。
    static constexpr float YAW_CONSTANT_SPEED_STEP_RADPS = 2.6f;    // 滚轮波动一次 步长为 0.4 rad/s
    static constexpr float YAW_CONSTANT_SPEED_LIMIT_RADPS = 5.0f;   // 匀速 Yaw 速度最大限幅
    static constexpr float YAW_SCROLL_THRESHOLD = 0.7f;             // 滚轮阈值
    
    // 变速 Yaw 速度 B + A1 * sin(w1 * t + φ1) + A2 * sin(w2 * t + φ2)
    static constexpr float YAW_VARIABLE_BASE_SPEED_RADPS = 4.9f;
    static constexpr float YAW_VARIABLE_AMPLITUDE_1_RADPS = 1.5f;
    static constexpr float YAW_VARIABLE_AMPLITUDE_2_RADPS = 0.6f;

    static constexpr float YAW_VARIABLE_FREQUENCY_1_HZ = 1.5f;
    static constexpr float YAW_VARIABLE_FREQUENCY_2_HZ = 4.0f;
    static constexpr float PI = 3.14159265358979323846f;
    static constexpr float YAW_VARIABLE_PHASE_1_RAD = -PI / 2.0f;   // φ1
    static constexpr float YAW_VARIABLE_PHASE_2_RAD = -PI / 2.0f;   // φ2
    static constexpr float YAW_VARIABLE_ANGULAR_FREQUENCY_1_RADPS = // w1
        2.0f * PI * YAW_VARIABLE_FREQUENCY_1_HZ;
    static constexpr float YAW_VARIABLE_ANGULAR_FREQUENCY_2_RADPS = // w2
        2.0f * PI * YAW_VARIABLE_FREQUENCY_2_HZ;

    static bool IsScrollPositive(float scroll);
    static bool IsScrollNegative(float scroll);
    static float GetVariableSpeed(float time_s);
    static void LimitTarget(GimbalTarget &target);

    GimbalYawState previous_state_ = GimbalYawState::STOP;  // 上一周期状态，用于检测状态切换
    float constant_speed_target_ = 0.0f;    // 匀速 Yaw 速度目标，单位为 rad/s
    int8_t variable_direction_ = 0;         // 变速 Yaw 方向：-1 表示反向，0 表示停止，1 表示正向
    bool scroll_event_latched_ = false;     // 滚轮拨动事件锁存，避免连续拨动造成速度跳变
};

inline void GimbalYawPlanner::Init()
{
    variable_start_time_s_ = 0.0f;
    previous_state_ = GimbalYawState::STOP;
    constant_speed_target_ = 0.0f;
    variable_direction_ = 0;
    scroll_event_latched_ = false;
}

/**
 * @brief 判断滚轮是否向正方向拨动
 * 
 * @param scroll 滚轮输入值
 * @return true 
 * @return false 
 */
inline bool GimbalYawPlanner::IsScrollPositive(float scroll)
{
    return scroll > YAW_SCROLL_THRESHOLD;
}

/**
 * @brief 判断滚轮是否向负方向拨动
 * 
 * @param scroll 滚轮输入值
 * @return true 
 * @return false 
 */
inline bool GimbalYawPlanner::IsScrollNegative(float scroll)
{
    return scroll < -YAW_SCROLL_THRESHOLD;
}

/**
 * @brief 获取变速 Yaw 速度
 * 
 * @param time_s 时间，单位为 s
 * @return float 变速 Yaw 速度，单位为 rad/s
 */
inline float GimbalYawPlanner::GetVariableSpeed(float time_s)
{
    return YAW_VARIABLE_BASE_SPEED_RADPS +
           YAW_VARIABLE_AMPLITUDE_1_RADPS *
               std::sin(YAW_VARIABLE_ANGULAR_FREQUENCY_1_RADPS * time_s +
                        YAW_VARIABLE_PHASE_1_RAD) +
           YAW_VARIABLE_AMPLITUDE_2_RADPS *
               std::sin(YAW_VARIABLE_ANGULAR_FREQUENCY_2_RADPS * time_s +
                        YAW_VARIABLE_PHASE_2_RAD);
}

/**
 * @brief 限幅 Yaw 速度目标
 * 
 * @param target Yaw 速度目标
 */
inline void GimbalYawPlanner::LimitTarget(GimbalTarget &target)
{
    target.target_yaw_velocity = std::clamp(target.target_yaw_velocity, -10.0f, 10.0f); // 限幅为 ±10.0 rad/s
}

class GimbalHightPlanner
{
public:
    // 总行程按减速器输出轴累计角度计算，底部为零，上行为正。
    static constexpr float HIGHT_TRAVEL_DEG = 670.0f;
    void Init();
    GimbalTarget Plan(GimbalHightState state,
                      float time_s,
                      bool upper_limit_active,
                      bool lower_limit_active,
                      float position_deg,
                      bool position_valid);

private:
    static constexpr float HIGHT_UP_SPEED_RADPS = 40.0f;
    static constexpr float HIGHT_DOWN_SPEED_RADPS = 40.0f;
    static constexpr float HIGHT_HOME_SPEED_RADPS = -3.0f;
    static constexpr float HIGHT_ACCELERATION_RADPS2 = 5000.0f;
    static constexpr float HIGHT_MAX_SPEED_RADPS = 40.0f;
    static constexpr float HIGHT_SLOWDOWN_START_RATIO = 0.7f; // 走完本方向行程的 70% 后减速
    static constexpr float HIGHT_LIMIT_SPEED_RADPS = 3.0f;    // 靠近终点时的速度绝对值，单位：rad/s

    static float GetTargetSpeed(GimbalHightState state);
    static float LimitSpeedByPosition(float speed_radps, float position_deg, bool position_valid);

    GimbalHightState previous_state_ = GimbalHightState::STOP;
    float current_speed_radps_ = 0.0f;
    float previous_time_s_ = 0.0f;
    bool initialized_ = false;
};

inline void GimbalHightPlanner::Init()
{
    previous_state_ = GimbalHightState::STOP;
    current_speed_radps_ = 0.0f;
    previous_time_s_ = 0.0f;
    initialized_ = false;
}

inline float GimbalHightPlanner::GetTargetSpeed(GimbalHightState state)
{
    switch (state)
    {
    case GimbalHightState::MOVING_UP:
        return HIGHT_UP_SPEED_RADPS;
    case GimbalHightState::MOVING_DOWN:
        return -HIGHT_DOWN_SPEED_RADPS;
    case GimbalHightState::RETURNING_HOME:
        return HIGHT_HOME_SPEED_RADPS;
    case GimbalHightState::STOP:
    case GimbalHightState::STATUS_COUNT:
    default:
        return 0.0f;
    }
}


